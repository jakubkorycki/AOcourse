import okodm_class
import time
import sys

#h=okodm.open("PDM 37ch,30mm","USB DAC 40ch, 12bit")
# h=okodm.open("MMDM 39ch,15mm","USB DAC 40ch, 12bit")
#h=okodm.open("MMDM 37ch,15mm","USB DAC 40ch, 12bit",["D40V2X02"])
#h=okodm.open("MMDM 96ch, embedded control","Embedded HV DAC")
#h=okodm.open("MMDM 79ch,30mm","USB DAC 40ch, 12bit")
h=okodm_class.open("MMDM 17ch,with tip/tilt","Ethernet DAC 40ch, 16bit")

if h==0:
    sys.exit("Error opening OKODM device: "+okodm_class.lasterror())
    
n=okodm_class.chan_n(h)
stage=1   
   
try:
    while True:
        if not okodm_class.set(h, ([1] if stage else [-1])*n):
            sys.exit("Error writing to OKODM device: "+okodm_class.lasterror())
        time.sleep(1)
        stage^=1
        
except KeyboardInterrupt:
    pass        
  
okodm_class.close(h)  

