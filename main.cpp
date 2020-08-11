#include "mbed.h"
#include "mbed_trace.h"
#include "mbedtls/debug.h"
#include "aws_credentials.h"

#include "rtos/ThisThread.h"
#include <chrono>

extern "C" {
// sdk initialization
#include "iot_init.h"
// mqtt methods
#include "iot_mqtt.h"
}

#include "aws_iot_ota_agent.h"

#include "LittleFileSystem.h"

#include "drivers/DigitalOut.h"

mbed::DigitalOut led(LED1, 1);

// This will take the system's default block device
BlockDevice *bd = BlockDevice::get_default_instance();
LittleFileSystem fs("fs");

// debugging facilities
#define TRACE_GROUP "Main"

using namespace std::chrono;

/**
 * @brief Handle of the MQTT connection used in this demo.
 */
static IotMqttConnection_t connection = IOT_MQTT_CONNECTION_INITIALIZER;

/**
 * @brief Flag used to unset, during disconnection of currently connected network. This will
 * trigger a reconnection from the OTA demo task.
 */
volatile static bool _networkConnected = false;

/**
 * @brief Connection retry interval in seconds.
 */
static int _retryInterval = 5;

static const char * _pStateStr[ eOTA_AgentState_All ] =
{
    "Init",
    "Ready",
    "RequestingJob",
    "WaitingForJob",
    "CreatingFile",
    "RequestingFileBlock",
    "WaitingForFileBlock",
    "ClosingFile",
    "Suspended",
    "ShuttingDown",
    "Stopped"
};

/* Application version info. */
#include "aws_application_version.h"

/* Declare the firmware version structure for all to see. */
const AppVersion32_t xAppFirmwareVersion = { {
                { APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD }
}};

static Mutex trace_mutex;
static void trace_mutex_lock()
{
    trace_mutex.lock();
}
static void trace_mutex_unlock()
{
    trace_mutex.unlock();
}
extern "C" void aws_iot_puts(const char *msg) {
    trace_mutex_lock();
    puts(msg);
    trace_mutex_unlock();
}

#define MQTT_TIMEOUT_MS    15000

// subscription event handler
static void on_message_received(void * pCallbackContext, IotMqttCallbackParam_t *pCallbackParam) {
    auto wait_sem = static_cast<Semaphore*>(pCallbackContext);
    char* payload = (char*)pCallbackParam->u.message.info.pPayload;
    auto payloadLen = pCallbackParam->u.message.info.payloadLength;
    tr_debug("from topic:%s; msg: %.*s", pCallbackParam->u.message.info.pTopicName, payloadLen, payload);

    if (strncmp("Warning", payload, 7) != 0) {
        tr_info("Hello %.*s !", payloadLen, payload);
        wait_sem->release();
    }
}

/**
 * @brief Initialize the libraries required for OTA demo.
 *
 * @return `EXIT_SUCCESS` if all libraries were successfully initialized;
 * `EXIT_FAILURE` otherwise.
 */

static void prvNetworkDisconnectCallback( void * param,
                                          IotMqttCallbackParam_t * mqttCallbackParams )
{
    ( void ) param;

    /* Log the reason for MQTT disconnect.*/
    switch( mqttCallbackParams->u.disconnectReason )
    {
        case IOT_MQTT_DISCONNECT_CALLED:
            tr_info( "Mqtt disconnected due to invoking diconnect function.\r\n" );
            break;

        case IOT_MQTT_BAD_PACKET_RECEIVED:
            tr_info( "Mqtt disconnected due to invalid packet received from the network.\r\n" );
            break;

        case IOT_MQTT_KEEP_ALIVE_TIMEOUT:
            tr_info( "Mqtt disconnected due to Keep-alive response not received.\r\n" );
            break;

        default:
            tr_info( "Mqtt disconnected due to unknown reason." );
            break;
    }

    /* Clear the flag for network connection status.*/
    _networkConnected = false;
}

/**
 * @brief The OTA agent has completed the update job or it is in
 * self test mode. If it was accepted, we want to activate the new image.
 * This typically means we should reset the device to run the new firmware.
 * If now is not a good time to reset the device, it may be activated later
 * by your user code. If the update was rejected, just return without doing
 * anything and we'll wait for another job. If it reported that we should
 * start test mode, normally we would perform some kind of system checks to
 * make sure our new firmware does the basic things we think it should do
 * but we'll just go ahead and set the image as accepted for demo purposes.
 * The accept function varies depending on your platform. Refer to the OTA
 * PAL implementation for your platform in aws_ota_pal.c to see what it
 * does for you.
 *
 * @param[in] eEvent Specify if this demo is running with the AWS IoT
 * MQTT server. Set this to `false` if using another MQTT server.
 * @return None.
 */
static void App_OTACompleteCallback( OTA_JobEvent_t eEvent )
{
    OTA_Err_t xErr = kOTA_Err_Uninitialized;

    DEFINE_OTA_METHOD_NAME( "App_OTACompleteCallback" );

    /* OTA job is completed. so delete the MQTT and network connection. */
    if( eEvent == eOTA_JobEvent_Activate )
    {
        tr_info( "Received eOTA_JobEvent_Activate callback from OTA Agent.\r\n" );

        /* OTA job is completed. so delete the network connection. */
        if( connection != NULL )
        {
            IotMqtt_Disconnect( connection, false );
        }

        /* Activate the new firmware image. */
        OTA_ActivateNewImage();

        /* We should never get here as new image activation must reset the device.*/
        tr_error( "New image activation failed.\r\n" );

        for( ; ; )
        {
        }
    }
    else if( eEvent == eOTA_JobEvent_Fail )
    {
        tr_info( "Received eOTA_JobEvent_Fail callback from OTA Agent.\r\n" );

        /* Nothing special to do. The OTA agent handles it. */
    }
    else if( eEvent == eOTA_JobEvent_StartTest )
    {
        /* This demo just accepts the image since it was a good OTA update and networking
         * and services are all working (or we wouldn't have made it this far). If this
         * were some custom device that wants to test other things before calling it OK,
         * this would be the place to kick off those tests before calling OTA_SetImageState()
         * with the final result of either accepted or rejected. */

        tr_info( "Received eOTA_JobEvent_StartTest callback from OTA Agent.\r\n" );
        xErr = OTA_SetImageState( eOTA_ImageState_Accepted );

        if( xErr != kOTA_Err_None )
        {
            tr_error( " Error! Failed to set image state as accepted.\r\n" );
        }
    }
}

void run_ota_update_demo(void * pNetworkServerInfo,
                         void * pNetworkCredentialInfo,
                         const IotNetworkInterface_t * pNetworkInterface)
{
    OTA_State_t eState;
    static OTA_ConnectionContext_t xOTAConnectionCtx;
    /* Update the connection context shared with OTA Agent.*/
    xOTAConnectionCtx.pxNetworkInterface = ( void * ) pNetworkInterface;
    xOTAConnectionCtx.pvNetworkCredentials = pNetworkCredentialInfo;
    xOTAConnectionCtx.pvControlClient = connection;

    /* Check if OTA Agent is suspended and resume.*/
    if( ( eState = OTA_GetAgentState() ) == eOTA_AgentState_Suspended )
    {
        OTA_Resume( &xOTAConnectionCtx );
    }

    /* Initialize the OTA Agent , if it is resuming the OTA statistics will be cleared for new connection.*/
    OTA_AgentInit( ( void * ) ( &xOTAConnectionCtx ),
                   ( const uint8_t * ) ( MBED_CONF_APP_AWS_CLIENT_IDENTIFIER ),
                   App_OTACompleteCallback,
                   ( TickType_t ) ~0 );

    while( ( ( eState = OTA_GetAgentState() ) != eOTA_AgentState_Stopped ) && _networkConnected )
    {
        /* Wait forever for OTA traffic but allow other tasks to run and output statistics only once per second. */
        rtos::ThisThread::sleep_for(3s);

        //led = !led; // Toggle led

        tr_info( "State: %s  Received: %u   Queued: %u   Processed: %u   Dropped: %u\r\n", _pStateStr[ eState ],
                    OTA_GetPacketsReceived(), OTA_GetPacketsQueued(), OTA_GetPacketsProcessed(), OTA_GetPacketsDropped() );
    }

    /* Check if we got network disconnect callback and suspend OTA Agent.*/
    if( _networkConnected == false )
    {
        /* Suspend OTA agent.*/
        if( OTA_Suspend() == kOTA_Err_None )
        {
            while( ( eState = OTA_GetAgentState() ) != eOTA_AgentState_Suspended )
            {
                /* Wait for OTA Agent to process the suspend event. */
                rtos::ThisThread::sleep_for(1s);
            }
        }
    }
    else
    {
        /* Try to close the MQTT connection. */
        if( connection != NULL )
        {
            IotMqtt_Disconnect( connection, 0 );
        }
    }
}

void fs_init(void) {
    // Try to mount the filesystem
    tr_info("mounting the filesystem... ");
    fflush(stdout);
    int err = fs.mount(bd);
    tr_info("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        // Reformat if we can't mount the filesystem
        tr_info("formatting... ");
        fflush(stdout);
        err = fs.reformat(bd);
        tr_info("%s\n", (err ? "Fail :(" : "OK"));
        if (err) {
            tr_error("error: %s (%d)\n", strerror(-err), err);
        }
    }
}

int main()
{
    mbed_trace_mutex_wait_function_set( trace_mutex_lock ); // only if thread safety is needed
    mbed_trace_mutex_release_function_set( trace_mutex_unlock ); // only if thread safety is needed
    mbed_trace_init();

    tr_info("initialize filesystem...");
    fs_init();

    mbedtls_debug_set_threshold(4);

    tr_info("Connecting to the network...");
    auto eth = NetworkInterface::get_default_instance();
    if (eth == NULL) {
        tr_error("No Network interface found.");
        return -1;
    }
    auto ret = eth->connect();
    if (ret != 0) {
        tr_error("Connection error: %x", ret);
        return -1;
    }
    tr_info("MAC: %s", eth->get_mac_address());
    tr_info("Connection Success");

    // demo :
    // - Init sdk
    if (!IotSdk_Init()) {
        tr_error("AWS Sdk: failed to initialize IotSdk");
        return -1;
    }
    auto init_status = IotMqtt_Init();
    if (init_status != IOT_MQTT_SUCCESS) {
        tr_error("AWS Sdk: Failed to initialize IotMqtt with %u", init_status);
        return -1;
    }
    // - Connect to mqtt broker
    IotMqttNetworkInfo_t network_info = IOT_MQTT_NETWORK_INFO_INITIALIZER;
    network_info.pNetworkInterface = aws::get_iot_network_interface();
    // create nework connection
    network_info.createNetworkConnection = true;
    network_info.u.setup.pNetworkServerInfo = {
        .hostname = MBED_CONF_APP_AWS_ENDPOINT,
        .port = 8883
    };
    network_info.u.setup.pNetworkCredentialInfo = {
        .rootCA = aws::credentials::rootCA,
        .clientCrt = aws::credentials::clientCrt,
        .clientKey = aws::credentials::clientKey
    };
    network_info.disconnectCallback.function = prvNetworkDisconnectCallback;

    IotMqttConnectInfo_t connect_info = IOT_MQTT_CONNECT_INFO_INITIALIZER;
    connect_info.awsIotMqttMode = true; // we are connecting to aws servers
    connect_info.pClientIdentifier = MBED_CONF_APP_AWS_CLIENT_IDENTIFIER;
    connect_info.clientIdentifierLength = strlen(MBED_CONF_APP_AWS_CLIENT_IDENTIFIER);


    auto connect_status = IotMqtt_Connect(&network_info, &connect_info, /* timeout ms */ MQTT_TIMEOUT_MS, &connection);
    if (connect_status != IOT_MQTT_SUCCESS) {
        tr_error("AWS Sdk: Connection to the MQTT broker failed with %u", connect_status);
        return -1;
    }

    _networkConnected = true;

    run_ota_update_demo((void*)&network_info.u.setup.pNetworkServerInfo,
            (void*)&network_info.u.setup.pNetworkCredentialInfo,
            network_info.pNetworkInterface);

    // - Subscribe to sdkTest/sub
    //   On message
    //   - Display on the console: "Hello %s", message
    /* Set the members of the subscription. */
//    static const char topic[] = MBED_CONF_APP_AWS_MQTT_TOPIC;
//    Semaphore wait_sem {/* count */ 0, /* max_count */ 1};
//
//    IotMqttSubscription_t subscription = IOT_MQTT_SUBSCRIPTION_INITIALIZER;
//    subscription.qos = IOT_MQTT_QOS_1;
//    subscription.pTopicFilter = topic;
//    subscription.topicFilterLength = strlen(topic);
//    subscription.callback.function = on_message_received;
//    subscription.callback.pCallbackContext = &wait_sem;
//
//    /* Subscribe to the topic using the blocking SUBSCRIBE
//     * function. */
//    auto sub_status = IotMqtt_SubscribeSync(connection, &subscription,
//                                            /* subscription count */ 1, /* flags */ 0,
//                                            /* timeout ms */ MQTT_TIMEOUT_MS );
//    if (sub_status != IOT_MQTT_SUCCESS) {
//        tr_error("AWS Sdk: Subscribe failed with : %u", sub_status);
//        return -1;
//    }
//
//    /* Set the members of the publish info. */
//    IotMqttPublishInfo_t publish = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
//    publish.qos = IOT_MQTT_QOS_1;
//    publish.pTopicName = topic;
//    publish.topicNameLength = strlen(topic);
//    publish.retryLimit = 3;
//    publish.retryMs = 1000;
//    for (uint32_t i = 0; i < 10; i++) {
//        // - for i in 0..9
//        //  - wait up to 1 sec
//        //  - if no message received Publish: "You have %d sec remaining to say hello...", 10-i
//        //  - other wise, exit
//        if (wait_sem.try_acquire_for(1000)) {
//            break;
//        }
//
//        /* prepare the message */
//        static char message[64];
//        snprintf(message, 64, "Warning: Only %lu second(s) left to say your name !", 10 - i);
//        publish.pPayload = message;
//        publish.payloadLength = strlen(message);
//
//        /* Publish the message. */
//        tr_info("sending warning message: %s", message);
//        auto pub_status = IotMqtt_PublishSync(connection, &publish,
//                                              /* flags */ 0, /* timeout ms */ MQTT_TIMEOUT_MS);
//        if (pub_status != IOT_MQTT_SUCCESS) {
//            tr_warning("AWS Sdk: failed to publish message with %u.", pub_status);
//        }
//    }

    /* Close the MQTT connection. */
    IotMqtt_Disconnect(connection, 0);

    IotMqtt_Cleanup();
    IotSdk_Cleanup();

    tr_info("Done");
    while (true) {
        ThisThread::sleep_for(1000);
    }
    return 0;
}
