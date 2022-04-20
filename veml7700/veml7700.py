#(c) 2022 Adrien Aurore
from micropython import const
from i2c_lib import *


# default address and registers
ADDR = const(0x10)

R_ALS_CONF = const(0x00)
R_ALS_WH   = const(0x01)
R_ALS_WL   = const(0x02)
R_POWER    = const(0x03)
R_ALS      = const(0x04)
R_WHITE    = const(0x05)
R_ALS_INT  = const(0x06)

B_ALS_GAIN         = const(11)
B_ALS_INTEGRATION  = const(6)
B_ALS_PERSISTANCE  = const(4)
B_ALS_INTERRUPTION = const(1)
B_ALS_SHUTDOWN     = const(0)

B_POWER_PSM        = const(1)
B_POWER_PSM_EN     = const(0)


# Write register 0
als_persistance  = {1:0b00, 2:0b01, 4:0b10, 8:0b11}
als_interruption = {"disable":0b0, "enable":0b1}
als_shutdown     = {"on":0b0, "off":0b1}

# Write register 0 (gain et intégration)
LEVEL_REG = const(0)
LEVEL_X   = const(1)
LEVEL_VAL = const(2)

als_level_gain = {      #reg, x, valeur
                    1: [0b10, 4, 0.125],
                    2: [0b11, 3, 0.25],
                    3: [0b00, 1, 1],
                    4: [0b01, 0, 2],
                 }
als_level_integration = {      #reg, x, valeur
                    -2: [0b1100, 5,  25],
                    -1: [0b1000, 4,  50],
                     0: [0b0000, 3, 100],
                     1: [0b0001, 2, 200],
                     2: [0b0010, 1, 400],
                     3: [0b0011, 0, 800],
                 }

# Write register 3
psm_mode = {1:0b00, 2:0b01, 3:0b10, 4:0b11} # PSM haut -> fréquence de rafraichissement bas
psm_en   = {"disable":0b0, "enable":0b1}

# Valeur du bit (ALS et white)
bit_gain        = {2:0, 1:1, 0.25:3, 0.125:4}
bit_integration = {800:0, 400:1, 200:2, 100:3, 50:4, 25:5}
bit_min         = 36 # penser à diviser par 10000
bit_range       = 16 # 16 bits en tout


# Classe principale
class VEML7700() :
    def __init__(self, i2c) :
        self._slave            = I2C_Slave(i2c, ADDR, 2)
        self._reg_gain         = I2C_RegisterBit(self._slave, R_ALS_CONF, B_ALS_GAIN,         2)
        self._reg_integration  = I2C_RegisterBit(self._slave, R_ALS_CONF, B_ALS_INTEGRATION,  4)
        self._reg_persistance  = I2C_RegisterBit(self._slave, R_ALS_CONF, B_ALS_PERSISTANCE,  2)
        self._reg_interruption = I2C_RegisterBit(self._slave, R_ALS_CONF, B_ALS_INTERRUPTION, 1)
        self._reg_shutdown     = I2C_RegisterBit(self._slave, R_ALS_CONF, B_ALS_SHUTDOWN,     1)
        self._reg_psm_mode     = I2C_RegisterBit(self._slave, R_POWER,    B_POWER_PSM,        2)
        self._reg_psm_enable   = I2C_RegisterBit(self._slave, R_POWER,    B_POWER_PSM_EN,     1)
        
        self._reg_als          = I2C_RegisterByte(self._slave, R_ALS_CONF, 1)
        self._reg_interr_high  = I2C_RegisterByte(self._slave, R_ALS_WH,   1)
        self._reg_interr_low   = I2C_RegisterByte(self._slave, R_ALS_WL,   1)
        self._reg_psm          = I2C_RegisterByte(self._slave, R_POWER,    1)
        self._reg_light_als    = I2C_RegisterByte(self._slave, R_ALS,      1)
        self._reg_light_white  = I2C_RegisterByte(self._slave, R_WHITE,    1)
        
        self._gain        = 1
        self._integration = -2
        self._delai       = Delay(1500)
        self._psm_mode    = 3
        
        self._reg_als.value         = 0
        self._reg_interr_high.value = 0
        self._reg_interr_low.value  = 0
        self._reg_psm.value         = 0
        
        self._update = True
        self._auto = True
        self._old_val = 0
        
        self.set_gain(self._gain)
        self.set_integration(self._integration)
        self.set_power("on")
        self.set_psm_mode(self._psm_mode)
        self._update_lsb()
    
    def _auto_adjust(self, value):
        if self._delai() :
            if self._auto :
                if value < 100 :
                    if self._integration < 3 :
                        self.set_integration(self._integration + 1)
                    elif self._gain < 4 :
                        self.set_gain(self._gain + 1)
                elif value > 300 :
                    if self._gain > 1 :
                        self.set_gain(self._gain - 1)
                    elif self._integration > -2 :
                        self.set_integration(self._integration - 1)
        
        return self._update_lsb()
                                
    def _auto_adjust_2(self, value):
        if self._auto :
            if self._old_val != value :
                self._old_val = value
                if value < 100 :
                    if self._integration < 3 :
                        if self._integration >= 0 :
                            if self._gain < 4 :
                                self.set_gain(self._gain + 1)
                            else :
                                self.set_integration(self._integration + 1)
                        else :
                            self.set_integration(self._integration + 1)
                elif value > 300 :
                    if self._integration > -2 :
                        if self._integration > 0 :
                            self.set_integration(self._integration - 1)
                        else :
                            if self._gain > 1 :
                                self.set_gain(self._gain - 1)
                            else :
                                self.set_integration(self._integration - 1)
        
    def _update_lsb(self):
        if self._update:
            self.set_power("off")
            
            self._update = False
            multiplier = als_level_gain[self._gain][LEVEL_X] + als_level_integration[self._integration][LEVEL_X]
            self._lsb = bit_min << multiplier 
            self._lsb = float(self._lsb) / 10000.0
            self._max = self._lsb * (256 ** 2)
            
            self._reg_gain.value        = als_level_gain[self._gain][LEVEL_REG]
            self._reg_integration.value = als_level_integration[self._integration][LEVEL_REG]
            self._reg_light_als.value   = 0
            self._reg_light_white.value = 0
            self.set_power("on")
            
            gain = als_level_gain[self._gain][LEVEL_VAL]
            integration = als_level_integration[self._integration][LEVEL_VAL]
            print("nouvelle correction", gain, integration, "(", self._lsb, self._max, ")")
            
            return True
        return False
    
    def set_auto(self, enable):
        self._auto = bool(enable)
        
    def set_gain(self, value):
        als_level_gain[self._gain][LEVEL_REG]
        self._gain = value
        self._update = True
        
    def set_integration(self, value):
        als_level_integration[self._integration][LEVEL_REG]
        self._integration = value
        self._update = True
        
    def set_power(self, value):
        self._reg_shutdown.value = als_shutdown[value]
        
    def set_psm_mode(self, value):
        self._reg_psm_mode.value   = psm_mode[value]
        self._reg_psm_enable.value = psm_en["enable"]
        self._psm_mode = value
        self._delai.dt = 1500 * value
        
    @property
    def als(self):
        als     = self._reg_light_als.value[0]
        als_lux = self._convert_lux(als, False)
        self._auto_adjust(als)
        return [als, als_lux]
        
    @property
    def white(self):
        white     = self._reg_light_white.value[0]
        white_lux = self._convert_lux(white, False)
        self.als
        return [white, white_lux]

    def _convert_lux(self, value, mode) :
        v = self._lsb * value
        if self._gain <= 2:
            if v > 100 :
                if mode :
                    v = 6.0135e-13*(v**4) - 9.3924e-9*(v**3) + 8.1488e-5*(v**2) + 1.0023*v
                else :
                    v = 2e-15*(v**4) + 4e-12*(v**3) + 9e-6*(v**2) + 1.0179*v - 11.05
        return v


# Exemples
if __name__ == '__main__':
    from time import sleep
    from machine import I2C, Pin
    
    i2c = I2C(1, scl=Pin(22), sda=Pin(21), freq=400000)
    lumiere = VEML7700(i2c)
    
    def test1():
        while True:
            print(lumiere.als, lumiere.white)
            sleep(1)
            
    def test2():
        lumiere.set_auto(False)
        for gain in range(1, 5) :
            for it in range(-2, 4)  : #4
                lumiere.set_gain(gain)
                lumiere.set_integration(it)
                lumiere._update_lsb()
                sleep(1)
                for i in range(3) :
                    sleep(1)
                    print(lumiere.als)