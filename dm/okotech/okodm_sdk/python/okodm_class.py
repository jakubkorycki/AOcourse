import ctypes
import os
import sys

# Define a special type for the 'double *' argument
class DoubleArrayType:
    def from_param(self, param):
        typename = type(param).__name__
        if hasattr(self, 'from_' + typename):
            return getattr(self, 'from_' + typename)(param)
        elif isinstance(param, ctypes.Array):
            return param
        else:
            raise TypeError("Can't convert %s" % typename)

    # Cast from array.array objects
    def from_array(self, param):
        if param.typecode != 'd':
            raise TypeError('must be an array of doubles')
        ptr, _ = param.buffer_info()
        return ctypes.cast(ptr, ctypes.POINTER(ctypes.c_double))

    # Cast from lists/tuples
    def from_list(self, param):
        val = ((ctypes.c_double)*len(param))(*param)
        return val

    from_tuple = from_list

    # Cast from a numpy array
    def from_ndarray(self, param):
        return param.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

# load okodm.dll library
if ctypes.sizeof(ctypes.c_void_p)==8:    
    _file='okodm64'
else:
   _file='okodm'         
_path=os.path.join(*(os.path.split(__file__)[:-1]+(_file,)))
_mod=ctypes.cdll.LoadLibrary(_path)


# int okodm_open(const char* mirror_type, const char* dac_type/*, const char** dac_ids*/);
_okodm_open=_mod.okodm_open
_okodm_open.argtypes=[ctypes.c_char_p,ctypes.c_char_p,ctypes.POINTER(ctypes.c_char_p)]
_okodm_open.restype=ctypes.c_int

def from_string(s):
    if sys.version_info[0]==3:
        return ctypes.c_char_p(bytes(s,'utf8'))
    else:
        return ctypes.c_char_p(s)

def open(mirror_type, dac_type, dac_ids=None):
    mirror=from_string(mirror_type)
    dac=from_string(dac_type)
    if dac_ids is None:
        dac_ids=[]
    ids_array_type=ctypes.c_char_p*len(dac_ids)
    ids=ids_array_type()
    for i in range(len(dac_ids)):
        ids[i]=from_string(dac_ids[i])
    return _okodm_open(mirror,dac,ids)

# void okodm_close(int handle);
_okodm_close=_mod.okodm_close
_okodm_close.argtypes=[ctypes.c_int]

def close(handle):
    h=ctypes.c_int(handle)
    _okodm_close(h)

# int okodm_chan_n(int handle);
_okodm_chan_n=_mod.okodm_chan_n
_okodm_chan_n.argtypes=[ctypes.c_int]
_okodm_chan_n.restype=ctypes.c_int    

def chan_n(handle):
    h=ctypes.c_int(handle)
    return _okodm_chan_n(h)    

# int okodm_set(int handle, double *values, int size);
_okodm_set=_mod.okodm_set
DoubleArray=DoubleArrayType()
_okodm_set.argtypes=[ctypes.c_int,DoubleArray,ctypes.c_int]
_okodm_set.restype=ctypes.c_int

def set(handle,values):
    h=ctypes.c_int(handle)
    return _okodm_set(h,values,len(values))
    
# char *okodm_lasterror();    
_okodm_lasterror=_mod.okodm_lasterror
_okodm_lasterror.argtypes=[]
_okodm_lasterror.restype=ctypes.c_char_p
def lasterror():
    return _okodm_lasterror()


