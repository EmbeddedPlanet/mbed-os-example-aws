/*
 * aws_ota_flash.cpp
 *
 *  Created on: Aug 11, 2020
 *      Author: gdbeckstein
 */

#include "aws_ota_pal_flash.h"
#include "secondary_bd.h"
#include "bootutil.h"
#include "SlicingBlockDevice.h"

namespace aws
{

namespace ota
{

mbed::BlockDevice* get_update_bd(void) {
    return get_secondary_bd();
}

void flag_update_as_ready(void) {
    boot_set_pending(false);
}

}

}


