#!/usr/bin/env python

#------------------------------------------------------------------------------
#                 PyuEye example - camera modul
#
# Copyright (c) 2017 by IDS Imaging Development Systems GmbH.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#------------------------------------------------------------------------------

from pyueye import ueye
from .ueye_utils import (uEyeException, Rect, get_bits_per_pixel,
                                  ImageBuffer, check)
from .ueye_utils import ImageData

class uEyeCamera:
    
    # Can't seem to get device_id to work for some reason..
    def __init__(self, device_id=0): #, device_id=0):
        #self.h_cam = ueye.HIDS()#device_id | ueye.IS_USE_DEVICE_ID)
        #self.h_cam = ueye.IS_USE_DEVICE_ID
        self.h_cam = ueye.HIDS(device_id)
        self.img_buffers = []
        self.info = self.init()
        

    def __enter__(self):
        return self

    def __exit__(self, _type, value, traceback):
        self.exit()
        
    def checkReturn(self,ret):
        if ret != ueye.IS_SUCCESS:
            raise 


    def handle(self):
        return self.h_cam

    def alloc(self, buffer_count=3):
        rect = self.get_aoi()
        bpp = get_bits_per_pixel(self.get_colormode())

        for buff in self.img_buffers:
            check(ueye.is_FreeImageMem(self.h_cam, buff.mem_ptr, buff.mem_id))

        if buffer_count>0:
            for i in range(buffer_count):
                buff = ImageBuffer()
                ueye.is_AllocImageMem(self.h_cam,
                                      rect.width, rect.height, bpp,
                                      buff.mem_ptr, buff.mem_id)
                
                check(ueye.is_AddToSequence(self.h_cam, buff.mem_ptr, buff.mem_id))
    
                self.img_buffers.append(buff)
            #
            ueye.is_InitImageQueue(self.h_cam, 0)
        return

    def max_width(self):
        return int(self.info.nMaxWidth)
    
    def max_height(self):
        return int(self.info.nMaxHeight)

    def init(self):
        ret = ueye.is_InitCamera(self.h_cam, None)
        if ret != ueye.IS_SUCCESS:
            self.h_cam = None
            raise uEyeException(ret)
        #
        
        self.set_colormode(ueye.IS_CM_MONO8)#IS_CM_SENSOR_RAW8)#IS_CM_MONO8)
        
        #ueye. IS_SET_ENABLE_AUTO_GAIN
        
        #mode = ueye.INT(1)
        #ret= ueye.is_DeviceFeature(self.h_cam,ueye.IS_DEVICE_FEATURE_CAP_WIDE_DYNAMIC_RANGE, mode, ueye.sizeof(mode))
        #if ret != ueye.IS_SUCCESS:
            #raise uEyeException(ret)
        
        ret = ueye.is_SetHardwareGamma(self.h_cam, ueye.IS_SET_HW_GAMMA_OFF)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
            
        info = ueye.SENSORINFO()
        ret = ueye.is_GetSensorInfo(self.h_cam, info)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
            
            
        #ueye.is_Gamma(self.h_cam, IS_SET_HW_GAMMA_OFF
        
        return info

    def exit(self):
        ret = None
        if self.h_cam is not None:
            ret = ueye.is_ExitCamera(self.h_cam)
        if ret == ueye.IS_SUCCESS:
            self.h_cam = None
        return

    def get_aoi(self):
        rect_aoi = ueye.IS_RECT()
        ueye.is_AOI(self.h_cam, ueye.IS_AOI_IMAGE_GET_AOI, rect_aoi, ueye.sizeof(rect_aoi))

        return Rect(rect_aoi.s32X.value,
                    rect_aoi.s32Y.value,
                    rect_aoi.s32Width.value,
                    rect_aoi.s32Height.value)
    
    def aoi_width(self):
        return self.aoi_w
    
    def aoi_height(self):
        return self.aoi_h

    def set_aoi(self, x, y, width, height):
        rect_aoi = ueye.IS_RECT()
        rect_aoi.s32X = ueye.int(x)
        rect_aoi.s32Y = ueye.int(y)
        rect_aoi.s32Width = ueye.int(width)
        rect_aoi.s32Height = ueye.int(height)
        
        self.aoi_w = width
        self.aoi_h = height

        return ueye.is_AOI(self.h_cam, ueye.IS_AOI_IMAGE_SET_AOI, rect_aoi, ueye.sizeof(rect_aoi))

    def set_software_trigger(self, enable=True):
        ueye.is_SetExternalTrigger(self.h_cam, 
            ueye.IS_SET_TRIGGER_SOFTWARE if  enable else ueye.IS_SET_TRIGGER_OFF)


    def capture_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT  
        # over er pythons variant av ternary operator "?:" i C
        # se Python 3.6 documentation, kap. 6.12. Conditional expressions:
        # https://docs.python.org/3.6/reference/expressions.html#grammar-token-or_test 
        return ueye.is_CaptureVideo(self.h_cam, wait_param)

    def stop_video(self):
        return ueye.is_StopLiveVideo(self.h_cam, ueye.IS_FORCE_VIDEO_STOP)
    
    def freeze_video(self, wait=False):
        wait_param = ueye.IS_WAIT if wait else ueye.IS_DONT_WAIT
        return ueye.is_FreezeVideo(self.h_cam, wait_param)

    def set_colormode(self, colormode):
        check(ueye.is_SetColorMode(self.h_cam, colormode))
        
    def get_colormode(self):
        ret = ueye.is_SetColorMode(self.h_cam, ueye.IS_GET_COLOR_MODE)
        return ret

    def get_format_list(self):
        count = ueye.UINT()
        check(ueye.is_ImageFormat(self.h_cam, ueye.IMGFRMT_CMD_GET_NUM_ENTRIES, count, ueye.sizeof(count)))
        format_list = ueye.IMAGE_FORMAT_LIST(ueye.IMAGE_FORMAT_INFO * count.value)
        format_list.nSizeOfListEntry = ueye.sizeof(ueye.IMAGE_FORMAT_INFO)
        format_list.nNumListElements = count.value
        check(ueye.is_ImageFormat(self.h_cam, ueye.IMGFRMT_CMD_GET_LIST,
                                  format_list, ueye.sizeof(format_list)))
        return format_list
    
    def get_exposure(self):
        exp = ueye.double()
        ret = ueye.is_Exposure(self.h_cam,ueye.IS_EXPOSURE_CMD_GET_EXPOSURE, exp, ueye.sizeof(exp) )
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return exp
    
    def set_subsampling(self, v=1,h=1):
        val = ueye.IS_SUBSAMPLING_DISABLE
        if v==2:
            val |= ueye.IS_SUBSAMPLING_2X_VERTICAL
        elif v==4:
            val |= ueye.IS_SUBSAMPLING_4X_VERTICAL 
            
        if h==2:
            val |= ueye.IS_SUBSAMPLING_2X_HORIZONTAL
        elif h==4:
            val |= ueye.IS_SUBSAMPLING_4X_HORIZONTAL

        ret = ueye.is_SetSubSampling(self.h_cam, val)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return ret

    
    def set_binning(self, v=1, h=1):

        val = ueye.IS_BINNING_DISABLE
        
        if v==2:
            val |= ueye.IS_BINNING_2X_VERTICAL
        elif v==4:
            val |= ueye.IS_BINNING_4X_VERTICAL 
            
        if h==2:
            val |= ueye.IS_BINNING_2X_HORIZONTAL
        elif h==4:
            val |= ueye.IS_BINNING_4X_HORIZONTAL

        ret = ueye.is_SetBinning(self.h_cam, val)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return ret
    
    def get_pixel_clock(self):
        clkInMHz = ueye.UINT()
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_GET, 
                           clkInMHz, ueye.sizeof(clkInMHz))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return clkInMHz
    
    def set_pixel_clock(self,clkRateInMHz):
        clkInMHz = ueye.UINT(clkRateInMHz)
        ret = ueye.is_PixelClock(self.h_cam, ueye.IS_PIXELCLOCK_CMD_SET,
                                 clkInMHz, ueye.sizeof(clkInMHz))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        
    
    def get_exposure_min(self):
        exp = ueye.double()
        ret = ueye.is_Exposure(self.h_cam,ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, exp, ueye.sizeof(exp))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return exp

    def set_exposure(self,exposure):
        exp = ueye.double(exposure)
        ret = ueye.is_Exposure(self.h_cam,ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exp, ueye.sizeof(exp) )
        return ret, exp
    
    def get_gamma(self):
        g = ueye.INT()
        ret = ueye.is_Gamma(self.h_cam,ueye.IS_GAMMA_CMD_GET, g, ueye.sizeof(g))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return g
    
    def set_gamma(self,g):
        g = ueye.INT(g)
        ret = ueye.is_Gamma(self.h_cam,ueye.IS_GAMMA_CMD_SET, g, ueye.sizeof(g))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
    
    def set_master_gain(self, g):
        ret = ueye.is_SetHWGainFactor(self.h_cam, ueye.IS_SET_MASTER_GAIN_FACTOR, ueye.INT(g*100))
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        

    def get_master_gain(self):        
        ret= ueye.is_SetHWGainFactor(self.h_cam, ueye.IS_GET_MASTER_GAIN_FACTOR, ueye.INT(0))
        return ret * 0.01
    
    def get_framerate(self):
        fps=ueye.double()
        ret= ueye.is_GetFramesPerSecond(self.h_cam, fps)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return fps
    
    def set_framerate(self, fps):
        newfps=ueye.double()
        ret=ueye.is_SetFrameRate(self.h_cam,fps,newfps)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)
        return newfps
        
    def set_optimal_camera_timing(self):
        maxclk = ueye.INT()
        maxfps = ueye.double()
        ret= ueye.is_SetOptimalCameraTiming(self.h_cam, ueye.IS_BEST_PCLK_RUN_ONCE, 10000, maxclk, maxfps)
        if ret != ueye.IS_SUCCESS:
            raise uEyeException(ret)

        return maxclk,maxfps

    def grab_single_frame(self, timeout=500):
        ueye.is_FreezeVideo(self.h_cam, ueye.IS_DONT_WAIT)
        return self.grab_frame(timeout)
        #        INT is_FreezeVideo (HIDS hCam, INT Wait)
    
    def grab_frame(self, timeout=500):
        img_buffer = ImageBuffer()
        ret = ueye.is_WaitForNextImage(self.h_cam,
                                       timeout,
                                       img_buffer.mem_ptr,
                                       img_buffer.mem_id)
        
        imgdata = None
        if ret == ueye.IS_SUCCESS:
            imgdata = ImageData(self.h_cam, img_buffer)
            data = imgdata.as_1d_image()
            imgdata.unlock()
            
            return data
        
        return None
        
