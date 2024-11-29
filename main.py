from machine import I2C, Pin
from bmp280 import BMP280

# Initialisation de l'I2C (adapter les broches selon votre carte)
i2c = I2C(0, scl=Pin(5), sda=Pin(4))  # GPIO 22 = SCL, GPIO 21 = SDA pour ESP32

# Initialisation du capteur BMP280
bmp = BMP280(i2c)

# Lecture des données
temperature = bmp.temperature
pressure = bmp.pressure
altitude = bmp.altitude()

# Affichage des résultats
print(f"Température : {temperature:.2f} °C")
print(f"Pression : {pressure:.2f} hPa")
print(f"Altitude : {altitude:.2f} m")

