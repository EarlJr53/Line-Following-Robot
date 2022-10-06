"""

"""

import serial


COMPORT = "/dev/ttyACM0"  # Arduino port on Brooke's Ubuntu install
BAUDRATE = 9600  # Set baud rate for serial connection

# open the serial port
serialPort = serial.Serial(COMPORT, BAUDRATE, timeout=1)

complete = False
started = False

while not complete:
    decision = input("Press 0 for STOP, 1 for SLOWER, and 2 for FASTER")
    serialPort.write(bytes(decision, 'utf-8'))
    if decision == "0":
        complete = True
    # if decision == "0" and not started:
    #     serialPort.write(bytes("Start"))
    #     started = True
    # elif decision == "0" and not complete and started:
    #     serialPort.write(bytes("Stop"))
    #     complete = True
    # else:
    #     serialPort.write(bytes(decision))