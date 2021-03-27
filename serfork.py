# serial forking script used with socat
# used for uni-directionaly serial + udp link for pppd-pppd communication
# github.com/toranova
# mailto:chia_jason96@live.com
import serial, subprocess, threading, time, socket, argparse

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
                        print("%s -> %s (%d):" %(src.port, dest_addr, len(msg))) #debugging
        except Exception as e:
            print("udp xmit error:",e)

    def udp_receive(self, src_sock, dest):
        '''src is a udp socket, and dest is a serialport. udp rx is sent to serial tx'''
        try:
            while True:
                msg, addr = src_sock.recvfrom(1024)
                dest.write(msg)
                if self.debug:
                    print("%s -> %s (%d):" %(addr, dest.port, len(msg)), buf) #debugging
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
                        print("%s -> %s (%d):" %(src.port, dest.port, len(msg)),buf) #debugging
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

    def __init__(self, phy, uplinkaddr=None, name0 = '/tmp/serfork0', name1 = '/tmp/serfork1',\
            baud=115200, udphostaddr=('0.0.0.0', 21221), timeout = [0,0,0], debug=False):
        '''creates the virtual pts using socat'''
        self.phyname = phy
        self.pty0name = name0
        self.pty1name = name1
        self.forwarding = []
        self.debug = debug
        try:
            self.proc = subprocess.Popen([
                '/usr/bin/socat',
                'pty,rawer,link=%s' % self.pty0name,
                'pty,rawer,link=%s' % self.pty1name
            ], stderr=subprocess.PIPE)
            time.sleep(1)
            if self.proc.poll() is not None:
                out, err = self.proc.communicate(timeout=3)
                print("fatal error has occurred: %s" % err)
                return
            self.phy  = serial.Serial(self.phyname,  baudrate = baud, timeout = timeout[0])
            self.pty0 = serial.Serial(self.pty0name, baudrate = baud, timeout = timeout[1])
            self.pty1 = serial.Serial(self.pty1name, baudrate = baud, timeout = timeout[2])
            self.us = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

            if uplinkaddr is None:
                self.us.bind(udphostaddr)
                # link pty1     rx - tx physical
                self.configure_forward(self.serial_forward, (self.pty1, self.phy) )
                # link udp serv rx - tx pty1
                self.configure_forward(self.udp_receive, (self.us, self.pty1))
            else:
                self.uladdr = uplinkaddr
                # link physical rx - tx pty1
                self.configure_forward(self.serial_forward, (self.phy, self.pty1) )
                # link pty1     rx - tx udp client
                self.configure_forward(self.udp_transmit, (self.pty1, self.us, self.uladdr))

        except subprocess.TimeoutExpired:
            print("socat process timed out")

    def run_until_interrupt(self):
        try:
            self.start_forward()
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop_forward()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("physical", help="the physical serial port to switch")
    parser.add_argument("-b", "--baudrate", help="the baud rate for all serial device", type=int, default=115200)
    parser.add_argument("-d", "--debug", help="enable debugging mode (might affect performance)", action="store_true")
    parser.add_argument("-p0", "--pty0name", help="user specified name for pty0", default="/tmp/serfork0")
    parser.add_argument("-p1", "--pty1name", help="user specified name for pty1", default="/tmp/serfork1")
    parser.add_argument("-ua", "--uplinkaddr", help="uplink udp address (for downlink device use)")
    parser.add_argument("-up", "--uplinkport", help="uplink udp port (for downlink device use)", default=21221)
    parser.add_argument("-sa", "--saddr", help="udp server address (for uplink device use)", default='0.0.0.0')
    parser.add_argument("-sp", "--sport", help="udp server port (for uplink device use)",default=21221)

    args = parser.parse_args()
    sf = SerialFork(args.physical, (args.uplinkaddr, args.uplinkport), args.pty0name, args.pty1name, args.baudrate, udphostaddr = (args.saddr, args.sport), debug=args.debug)
    sf.run_until_interrupt()
    #TODO: /tmp/serfork0 cannot be stat?
