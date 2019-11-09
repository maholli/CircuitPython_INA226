import time, board, busio
from ina226 import INA226

i2c = busio.I2C(board.SCL,board.SDA)
ina226 = INA226(i2c, addr=0x40)
print(ina226.shunt_voltage)
print(ina226.bus_voltage)

