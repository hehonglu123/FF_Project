import time

import board
import busio

import adafruit_vl6180x

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_vl6180x.VL6180X(i2c)


while True:
    print("Range: {0}mm".format(sensor.range))
    time.sleep(1.0)
