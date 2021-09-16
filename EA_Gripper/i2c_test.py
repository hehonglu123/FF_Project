import time
import smbus

i2c_ch = 1

# TMP102 address on the I2C bus
i2c_address = 0x44

# Register addresses
reg_temp = 0x00
reg_config = 0x01


# Initialize I2C (SMBus)
bus = smbus.SMBus(i2c_ch)

# Read the CONFIG register (2 bytes)
val = bus.read_i2c_block_data(i2c_address, reg_config, 2)
print("Old CONFIG:", val)
