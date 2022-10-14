"""

"""

import serial
import pandas as pd
import numpy as np


COMPORT = "/dev/ttyACM0"  # Arduino port on Brooke's Ubuntu install
BAUDRATE = 9600  # Set baud rate for serial connection

# open the serial port
serialPort = serial.Serial(COMPORT, BAUDRATE, timeout=1)

complete = False
started = False

dictValues = {
    'Time': [],
    'sensorLL': [], 
    'sensorCL': [], 
    'sensorCR': [], 
    'sensorRR': [], 
    'motorL': [], 
    'motorR': []
}

while not complete:
    decision = input("Press 0 for STOP, 1 for SLOWER, and 2 for FASTER")
    serialPort.write(bytes(decision, 'utf-8'))
    if decision == "0":
        complete = True
        # continue

    # Request data line from Arduino
    rawDataLine = serialPort.readline().decode()

    if len(rawDataLine) > 0:
        # Split raw data into sensor, pan, and tilt
        elapsedTime, rawLL, rawCL, rawCR, rawRR, motorL, motorR = (
                    float(x) for x in rawDataLine.split(','))

        dictValues['Time'].append(elapsedTime)
        dictValues['sensorLL'].append(rawLL)
        dictValues['sensorCL'].append(rawCL)
        dictValues['sensorCR'].append(rawCR)
        dictValues['sensorRR'].append(rawRR)
        dictValues['motorL'].append(motorL)
        dictValues['motorR'].append(motorR)
        

df = pd.DataFrame(dictValues)

df.to_csv('./currentrun.csv', index=False)
