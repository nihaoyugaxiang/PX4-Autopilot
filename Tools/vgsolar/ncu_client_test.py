#!/usr/bin/env python3
import json
import os
import pty
import select
import struct
import subprocess
import sys
import threading
import time
import tty
import unittest
from pathlib import Path
import ncu_client as ncu


def feedback(kind, body):
    data = bytes((0xBB,kind,len(body))) + body
    return b'\xa5\x5a' + data + bytes((sum(data)&255,255))


class ClientTest(unittest.TestCase):
    def test_golden_commands(self):
        self.assertEqual(ncu.speed(0,0).hex(), 'a55aaa01050200000000b2ff')
        self.assertEqual(ncu.system(1).hex(), 'a55aaa050101b1ff')
        self.assertEqual(ncu.parameter(0x104,50)[5:-2], b'\x04\x01\x03\x32\x00\x00\x00')
        self.assertEqual(struct.unpack('<BBHH',ncu.turn(9000,3000)[5:-2]),(1,1,9000,3000))
        self.assertEqual(struct.unpack('<BiiHHh',ncu.navigation(3,100,-200,-30,10)[5:-2]),(3,100,-200,65535,10,-30))

    def test_domains(self):
        for fn in (lambda:ncu.speed(501,0),lambda:ncu.speed(0,18001),lambda:ncu.turn(0,1),
                   lambda:ncu.parameter(0x105,1),lambda:ncu.parameter(0x101,2),
                   lambda:ncu.navigation(1,0,900000001),lambda:ncu.navigation(3,radius=50001)):
            with self.assertRaises(ValueError): fn()

    def test_feedbacks_and_ack_meaning(self):
        p=ncu.Parser()
        packets=feedback(2,b'\x02\x01')+feedback(2,b'\x06\x02')+feedback(3,struct.pack('<HBI',0x104,3,50))+feedback(4,struct.pack('<BBIh',1,3,123,-900))
        decoded=p.feed(packets)
        self.assertEqual(decoded[0]['meaning'],'received only')
        self.assertEqual(decoded[1]['meaning'],'rejected')
        self.assertEqual(decoded[2]['value'],50)
        self.assertEqual(decoded[3]['heading_error_cd'],-900)

    def test_status_signed_layout(self):
        values=(70,1200000000,300000000,27000,20,-10,20,-100,100,3,3,0x800,3,10,10,65535,65535)
        packet=feedback(1,struct.pack('<BiiHhhhhhBBHBHHHH',*values))
        result=[]; p=ncu.Parser()
        for byte in packet: result+=p.feed(bytes((byte,)),now=1.)
        self.assertEqual(result[0]['left_track_cms'],-10)
        self.assertEqual(result[0]['right_out_cm'],65535)
        self.assertEqual(result[0]['fault_code'],0x800)

    def test_noise_corruption_timeout(self):
        packet=feedback(2,b'\x05\x01'); bad=bytearray(packet); bad[-2]^=1
        p=ncu.Parser(); self.assertEqual(p.feed(packet[:6],now=1),[])
        self.assertEqual(p.feed(packet[6:],now=1.3),[])
        self.assertEqual(len(p.feed(bytes(bad)+b'\xa5\xa5'+packet,now=1.4)),1)
        self.assertGreaterEqual(p.bad_frames,2)
        p.feed(b'\x33'*100000,now=2); self.assertLess(len(p.data),42)

    def test_partial_writes(self):
        class Port:
            data=b''
            def write(self,b): self.data+=b[:2]; return min(2,len(b))
        port=Port(); packet=ncu.turn(9000,3000); ncu.write_all(port,packet)
        self.assertEqual(port.data,packet)

    def test_stop_after_receive_error(self):
        class Port:
            packets=[]
            def write(self,b): self.packets.append(bytes(b)); return len(b)
            def read(self,n): raise OSError('disconnected')
        port=Port()
        with self.assertRaises(OSError): ncu.run(port,ncu.speed(30,0),1,repeat_speed=True)
        self.assertEqual(port.packets[-1],ncu.speed(0,0))

    def test_cli_default_never_opens_uart(self):
        output=subprocess.check_output([sys.executable,str(Path(ncu.__file__)),'--port','/no/such/device','speed','30','0'],text=True)
        self.assertEqual(output.strip(),ncu.speed(30,0).hex(' '))

    @unittest.skipUnless(sys.platform.startswith('linux'),'Linux PTY test')
    def test_virtual_serial_roundtrip(self):
        master,slave=pty.openpty(); tty.setraw(slave)
        received=bytearray(); ended=threading.Event(); messages=[]
        class Port:
            def write(self,b): return os.write(slave,b[:3])
            def read(self,n):
                return os.read(slave,n) if select.select([slave],[],[],.01)[0] else b''
        def emulator():
            while not ended.is_set():
                if select.select([master],[],[],.01)[0]:
                    received.extend(os.read(master,512))
                    reply=feedback(2,b'\x05\x01')
                    os.write(master,reply[:4]); os.write(master,reply[4:])
        thread=threading.Thread(target=emulator); thread.start()
        try:
            ncu.run(Port(),ncu.speed(30,0),.26,repeat_speed=True,emit=messages.append)
            time.sleep(.03)
        finally:
            ended.set(); thread.join(timeout=1); os.close(master); os.close(slave)
        self.assertGreaterEqual(bytes(received).count(ncu.speed(30,0)),2)
        self.assertTrue(bytes(received).endswith(ncu.speed(0,0)))
        self.assertTrue(any(json.loads(message)['type']=='ack' for message in messages))


if __name__=='__main__': unittest.main(verbosity=2)
