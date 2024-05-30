import okodm
import time
import sys

h1=okodm.open("MMDM 37ch,15mm","USB DAC 40ch, 12bit",["D40V2X01"])
#h=okodm.open("MMDM 96ch, embedded control","Embedded HV DAC")
#h=okodm.open("MMDM 79ch,30mm","USB DAC 40ch, 12bit")
if h1==0:
    sys.exit("Error opening OKODM device: "+okodm.lasterror())
print(h1)

h2=okodm.open("MMDM 37ch,15mm","USB DAC 40ch, 12bit",["D40V2X02"])
if h2==0:
    sys.exit("Error opening OKODM device: "+okodm.lasterror())
print(h2)    
    
n=okodm.chan_n(h1)
stage=1   
   
try:
    while True:
        if not okodm.set(h1, ([1] if stage else [-1])*n):
            sys.exit("Error writing to OKODM device: "+okodm.lasterror())
        if not okodm.set(h2, ([0.5] if stage else [-1])*n):
            sys.exit("Error writing to OKODM device: "+okodm.lasterror())    
        time.sleep(1)
        stage^=1
        
except KeyboardInterrupt:
    pass        
  
okodm.close(h1)
okodm.close(h2)  

