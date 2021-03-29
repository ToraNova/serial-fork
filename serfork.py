# serial forking script used with socat
# used for uni-directionaly serial + udp link for pppd-pppd communication
# github.com/toranova
# mailto:chia_jason96@live.com
import serial, subprocess, threading, time, socket, argparse, os

def make_ipmasq_cmd(intif, lanif):
    return [
            ['/usr/bin/sysctl','net.ipv4.ip_forward=1'],
            ['/usr/bin/iptables','-F'], # flush all entries
            ['/usr/bin/iptables','-t','nat','-F'], # flush all entries in the NAT
            ['/usr/bin/iptables','-t','nat','-A','POSTROUTING','-o',intif,'-j','MASQUERADE'],
            ['/usr/bin/iptables','-A','FORWARD','-m','conntrack','--ctstate','RELATED,ESTABLISHED','-j','ACCEPT'],
            ['/usr/bin/iptables','-A','FORWARD','-i',lanif,'-o',intif,'-j','ACCEPT'],
    ]

class SerialFork:

    # the debug and none debug functions are split for possible performance issues.
    def udp_transmit(self, src, dest_sock, dest_addr):
        '''src is a serial object, and dest is a udp socket. serial packet is sent to tx-udp'''
        try:
            while True:
                if src.in_waiting > 0:
                    buf = src.read(src.in_waiting)
                    dest_sock.sendto(buf, dest_addr)
                    if self.debug:
                        print("%s -> %s (%d):" %(src.port, dest_addr, len(buf)), buf) #debugging
                else:
                    time.sleep(0.001) # slowdown if no message
        except Exception as e:
            print("udp xmit error:",e)

    def udp_receive(self, src_sock, dest):
        '''src is a udp socket, and dest is a serialport. udp rx is sent to serial tx'''
        try:
            while True:
                msg, addr = src_sock.recvfrom(self.rwbufsz)
                dest.write(msg)
                if self.debug:
                    print("%s -> %s (%d):" %(addr, dest.port, len(msg)), msg) #debugging
        except Exception as e:
            print("udp recv error:", e)

    def serial_forward(self, src, dest):
        '''forwards what we read from src's input buffer into dst (both assumed to be serial)'''
        try:
            while True:
                if src.in_waiting > 0:
                    # there are something in the input buffer to forward
                    buf = src.read(src.in_waiting)
                    dest.write(buf) #forward
                    if self.debug:
                        print("%s -> %s (%d):" %(src.port, dest.port, len(buf)) ,buf) #debugging
                else:
                    time.sleep(0.001) # slowdown if no message
        except Exception as e:
            print("serial forward error:",e)

    def configure_forward(self, method, args):
        t = threading.Thread(target=method, args=args)
        t.daemon = True # set daemon
        self.forwarding.append(t)

    def start_forward(self):
        for f in self.forwarding:
            f.start()

    def stop_forward(self):
        print("stopping all forwarding rules. please wait.")
        for f in self.forwarding:
            f.join(1.0)

    def __init__(self, phy, uplink=True, udpaddr=('0.0.0.0', 21221), baud=115200, debug=False, rwbufsize=1024, timeout = [None, None, None]):
        '''creates the virtual pts using socat'''
        self.phyname = phy
        self.forwarding = []
        self.debug = debug
        self.rwbufsz = rwbufsize
        self.fatal = False
        self.pty0name = '/tmp/serfork0'
        self.pty1name = '/tmp/serfork1'
        self.baud = baud
        self.uplink = uplink
        try:
            self.proc = subprocess.Popen([
                '/usr/bin/socat',
                'pty,rawer,link=%s' % self.pty0name,
                'pty,rawer,link=%s' % self.pty1name,
            ], stderr=subprocess.PIPE)
            time.sleep(1) # wait for pts to be created
            rc = self.proc.poll()
            if rc is not None:
                out, err = self.proc.communicate()
                raise Exception(err)
            self.phy  = serial.Serial(self.phyname,  baudrate = self.baud, timeout = timeout[0])
            self.pty0 = serial.Serial(self.pty0name, baudrate = self.baud, timeout = timeout[1])
            self.pty1 = serial.Serial(self.pty1name, baudrate = self.baud, timeout = timeout[2])
            self.us = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            if self.uplink:
                print("starting uplink device with udp service %s." % str(udpaddr))
                self.us.bind(udpaddr)
                # link pty1     rx - tx physical
                self.configure_forward(self.serial_forward, (self.pty1, self.phy) )
                # link udp serv rx - tx pty1
                self.configure_forward(self.udp_receive, (self.us, self.pty1))
            else:
                print("starting downlink device to udp service %s." % str(udpaddr))
                # link physical rx - tx pty1
                self.configure_forward(self.serial_forward, (self.phy, self.pty1) )
                # link pty1     rx - tx udp client
                self.configure_forward(self.udp_transmit, (self.pty1, self.us, udpaddr))

        except Exception as e:
            print("setup exception has occurred:",e)
            self.fatal = True

    def run_until_interrupt(self, srcaddr, destaddr, iface=None):
        if self.fatal:
            raise Exception("setup exception was fatal.")
        try:
            self.start_forward()
            ppp = subprocess.Popen([
                '/usr/bin/pppd', 'nodetach', 'noauth', 'local',
                '%s:%s' % (srcaddr, destaddr), self.pty0name, str(self.baud),
            ], stdout=subprocess.PIPE, universal_newlines=True)
            print("configuring pppd with addresses %s -> %s" %(srcaddr, destaddr))

            if iface is not None:
                # configure firewall (assume downlink device)
                isrc = 'ppp0'
                idst = iface
                if self.uplink: # uplink means iface is wlan0 or eth0, allow ppp0 to have internet
                    isrc = iface
                    idst = 'ppp0'
                iptab = make_ipmasq_cmd(isrc, idst)
                print("configuring firewall internet sharing rules %s -> %s" %(isrc,idst))
                for cmd in iptab:
                    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    out,err = p.communicate()
                    if p.returncode: # error
                        raise Exception("firewall exception was fatal. (%d) %s"%(p.returncode,err))

            for line in iter(ppp.stdout.readline, ""):
                print(line, end="")
        except KeyboardInterrupt:
            self.stop_forward()
            ppp.stdout.close()

if __name__ == "__main__":
    # this script should be run as root

    parser = argparse.ArgumentParser()
    parser.add_argument("physical", help="the physical serial port to switch")
    parser.add_argument("-b", "--baudrate", help="the baud rate for all serial device", type=int, default=115200)
    parser.add_argument("-i", "--interface", help="configure firewall for interface to share(downlink)/receive(uplink) internet")
    parser.add_argument("-d", "--debug", help="enable debugging mode (might affect performance)", action="store_true")
    parser.add_argument("-u", "--uplink", help="specify if this device is uplink. uplink is the udp server", action="store_true")
    parser.add_argument("-a", "--uaddr", help="uplink udp address (please specify if device is downlink)", default="0.0.0.0")
    parser.add_argument("-p", "--uport", help="uplink udp port", default=21221)
    parser.add_argument("-ds", "--dlink_pppsrc", help="serial downlink src address for PPPoS",default="10.2.1.2")
    parser.add_argument("-us", "--ulink_pppsrc", help="serial uplink src address for PPPoS",default="10.2.1.1")

    args = parser.parse_args()
    src = args.dlink_pppsrc
    dst = args.ulink_pppsrc
    if args.uplink:
        # flip if we're uplink
        src = args.ulink_pppsrc
        dst = args.dlink_pppsrc

    sf = SerialFork(args.physical, args.uplink, (args.uaddr, args.uport), args.baudrate, args.debug)
    sf.run_until_interrupt(src, dst, args.interface)
