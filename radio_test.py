import logging
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crtp.crtpstack import CRTPPacket

import struct

#   My CF channel number: '80'
uri = 'radio://0/80/2M/E7E7E7E7E7'

if __name__ == '__main__':

    # Initialize everything
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    cf.open_link(uri)
    time.sleep(2)

    for i in range(100):
        m1 = i
        m2 = i+100
        m3 = i+200
        m4 = i+300
        pk = CRTPPacket()
        pk.port = 0x0A
        pk.channel = 0
        pk.data = struct.pack('<HHHH', m1, m2, m3, m4)
        cf.send_packet(pk)

    time.sleep(2)
    cf.close_link()
