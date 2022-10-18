"""
Sensor calibration script
"""

import serial

COMPORT = "/dev/ttyACM0"  # Arduino port on Brooke's Ubuntu install
BAUDRATE = 9600  # Set baud rate for serial connection

# open the serial port
serialPort = serial.Serial(COMPORT, BAUDRATE, timeout=1)

while True:
    print(serialPort.readline().decode())
