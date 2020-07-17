
def main():
    with open('putty.log', 'rb') as f:
        while True:
            l = f.readline()
            if "CONNECT" in l:
                print("PPP data starts at byte offset: {}".format(f.tell()))
                break
            
        with open('pppdump.txt', 'w+') as f2:
            packet_count = 0
            in_packet = False
            while True:
                next_byte = f.read(1)
                if not next_byte:
                    break
                if ord(next_byte) == 0x7E:
                    if in_packet:
                        print("ppp: end 7e flag detected")
                        in_packet = False
                    else:
                        in_packet = True
                        packet_count += 1
                        print("ppp: begin new packet (#{})".format(packet_count))
                        f2.write('\n')
                        f2.write('000000')
                        
                f2.write(" %02x" % ord(next_byte))
                
        
if __name__=="__main__":
    main()
