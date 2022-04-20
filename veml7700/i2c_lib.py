# Attention, cette lib est uniquement adapté VEML7700 - à corriger
import struct
from time import ticks_ms


# Classe
class Delay():
    def __init__(self, dt, auto_reset=True):
        self.dt  = dt
        self._old = ticks_ms()
        self._auto_reset = auto_reset
    
    def _timer(self):
        return ticks_ms() - self._old > self.dt
    
    def _reset(self, value=0):
        self._old = ticks_ms() + value
    
    def __call__(self, value=None):
        if value == None :
            if self._auto_reset :
                result = self._timer()
                if result :
                    self._reset()
                return result
            else :
                return self._timer()
        else :
            self._reset(value)


# Classe module iI2C
class I2C_Slave():
    def __init__(self, i2c, addr, size_reg=1):
        self._i2c  = i2c
        self._addr = addr
        self._size = size_reg
        
    def read(self, reg, nb):
        return self._i2c.readfrom_mem(self._addr, reg, self._size*nb)
    
    def write(self, reg, value) :
        self._i2c.writeto_mem(self._addr, reg, value)


# Classe lecture / écriture bits dans registre
class I2C_RegisterBit():
    def __init__(self, Slave, register, position, size=1):
        self._Slave = Slave
        self._reg   = register
        self._pos   = position
        self._size  = size
        self._mask_read  =  (1 << self._size) - 1
        self._mask_write = ((1 << self._size) - 1) << self._pos
    
    def _read(self):
        content = self._Slave.read(self._reg, self._Slave._size)
        return struct.unpack("<H", content)[0]
    
    def _change_bit(self, val, bits):
        return (val & ~self._mask_write) | ((bits << self._pos) & self._mask_write)
        
    @property
    def value(self):
        content = self._read() >> self._pos
        return content & self._mask_read
    
    @value.setter
    def value(self, val):
        content = self._read()
        content = self._change_bit(content, val)
        content = struct.pack("<H", content)
        self._Slave.write(self._reg, content)


# Classe lecture / écriture bytes
class I2C_RegisterByte():
    def __init__(self, Slave, register, size=1):
        self._Slave = Slave
        self._reg   = register
        self._size  = size
        
    @property
    def value(self):
        content = self._Slave.read(self._reg, self._size*self._Slave._size)
        return struct.unpack("<" + self._size*"H", content)
    
    @value.setter
    def value(self, val):
        if type(val) is list :
            content = struct.pack("<" + self._size*"H", *val)
        else :
            content = struct.pack("<" + self._size*"H", val)
        self._Slave.write(self._reg, content)
