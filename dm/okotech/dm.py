"""

Deformable mirror class that implements the same API as dm/thorlabs/dm.ThorlabsDM

"""
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import okodm_sdk.python.okodm_class as okodm

class OkoDM:
    def __init__(self, dmtype):
        
        types=[
            ("MMDM 96ch, embedded control","Embedded HV DAC"),
            ("MMDM 17ch,with tip/tilt","Ethernet DAC 40ch, 16bit")
        ]
        
        self.h=okodm.open(types[dmtype][0],types[dmtype][1])
        
        if self.h==0:
            raise ConnectionError(f"Error opening OKODM device: {okodm.lasterror()}")
            
        self.n=okodm.chan_n(self.h)
        
    def __len__(self):
        return self.n
    
    def setActuators(self, act):
        
        assert len(act) == len(self)
        okodm.set(self.h, act)
    def getActuators(self):
        return
    
    def close(self):
        if self.h != 0:
            okodm.close(self.h)
            self.h = None
    
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
    
if __name__ == "__main__":
    import numpy as np
    import time
    
    
    act=np.ones([19])*0.9
    l=np.linspace(-0.9,0.9,5)
    with OkoDM(dmtype=0) as dm:
        while True:
            for i in range(5):
                act=np.ones([19])
                dm.setActuators(act)
                
                time.sleep(0.2)
                
            for i in range(5):
                act=np.ones([19])
                dm.setActuators(act)
                
                time.sleep(0.2)
        