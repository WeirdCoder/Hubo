'''
Author: alexc89@mit.edu
SendMessageAchMatlab
Used to multicast Hubo's status on LCM.  It will take the information received on ACH and convert it to LCM.  This is needed for live visualization.

'''

import lcm
import time
import ach
import hubo_ach as ha
import time
from ctypes import *

#Import LCM Messages
from lcmtypes import vicon_ballstate


if __name__ == "__main__":
    #Setup ACH LCM channels
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")
    #Setup ACH

    while True:  #constant Transmission        
        #Grab a frame form ACH
        #ACH to LCM conversion
        msg = vicon_ballstate()
        msg.timestamp =  time.time()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 10.0
        
        #Pushout an LCM message
        lc.publish("ViconBall", msg.encode())
        #Loop Delay
        time.sleep(1)
    #ACH LCM terminate
