from micropython import const
from ustruct import unpack as unp

# Registres importants du BMP280
_REGISTER_ID = const(0xD0)
_REGISTER_RESET = const(0xE0)
_REGISTER_STATUS = const(0xF3)
_REGISTER_CONTROL = const(0xF4)
_REGISTER_CONFIG = const(0xF5)
_REGISTER_DATA = const(0xF7)

# Valeurs de contrôle
_MODE_SLEEP = const(0x00)
_MODE_FORCED = const(0x01)
_MODE_NORMAL = const(0x03)

# Valeurs de calibration
_OSRS_T = const(0x01)  # Température x1
_OSRS_P = const(0x01)  # Pression x1

# Adresses possibles
_I2C_ADDR_PRIMARY = const(0x76)
_I2C_ADDR_SECONDARY = const(0x77)


class BMP280:
    def __init__(self, i2c, address=_I2C_ADDR_PRIMARY):
        self._i2c = i2c
        self._address = address
        self._load_calibration()

        # Configurer le capteur en mode normal
        self._write(_REGISTER_CONTROL, (_OSRS_T << 5) | (_OSRS_P << 2) | _MODE_NORMAL)

    def _read(self, register, length):
        return self._i2c.readfrom_mem(self._address, register, length)

    def _write(self, register, data):
        self._i2c.writeto_mem(self._address, register, bytearray([data]))

    def _load_calibration(self):
        # Lecture des coefficients de calibration
        coeffs = self._read(0x88, 24)
        self._T1 = unp('<H', coeffs[0:2])[0]
        self._T2 = unp('<h', coeffs[2:4])[0]
        self._T3 = unp('<h', coeffs[4:6])[0]
        self._P1 = unp('<H', coeffs[6:8])[0]
        self._P2 = unp('<h', coeffs[8:10])[0]
        self._P3 = unp('<h', coeffs[10:12])[0]
        self._P4 = unp('<h', coeffs[12:14])[0]
        self._P5 = unp('<h', coeffs[14:16])[0]
        self._P6 = unp('<h', coeffs[16:18])[0]
        self._P7 = unp('<h', coeffs[18:20])[0]
        self._P8 = unp('<h', coeffs[20:22])[0]
        self._P9 = unp('<h', coeffs[22:24])[0]

    def _compensate_temperature(self, adc_T):
        var1 = ((adc_T / 16384.0) - (self._T1 / 1024.0)) * self._T2
        var2 = (((adc_T / 131072.0) - (self._T1 / 8192.0)) ** 2) * self._T3
        self._t_fine = int(var1 + var2)
        return (var1 + var2) / 5120.0

    def _compensate_pressure(self, adc_P):
        var1 = (self._t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self._P6 / 32768.0
        var2 = var2 + var1 * self._P5 * 2.0
        var2 = (var2 / 4.0) + (self._P4 * 65536.0)
        var1 = (self._P3 * var1 * var1 / 524288.0 + self._P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._P1
        if var1 == 0:
            return 0  # Prévention contre la division par zéro
        pressure = 1048576.0 - adc_P
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self._P9 * pressure * pressure / 2147483648.0
        var2 = pressure * self._P8 / 32768.0
        pressure = pressure + (var1 + var2 + self._P7) / 16.0
        return pressure / 100.0

    @property
    def temperature(self):
        # Lecture des données brutes
        data = self._read(_REGISTER_DATA, 6)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        return self._compensate_temperature(adc_T)

    @property
    def pressure(self):
        # Lecture des données brutes
        data = self._read(_REGISTER_DATA, 6)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return self._compensate_pressure(adc_P)

    def altitude(self, sea_level_pressure=1013.25):
        pressure = self.pressure
        return 44330.0 * (1.0 - (pressure / sea_level_pressure) ** (1 / 5.255))

