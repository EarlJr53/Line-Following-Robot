"""
Python script to control line-following robot and collect data
"""

import serial
import pandas as pd
from pynput import keyboard

COMPORT = "/dev/ttyACM0"  # Arduino port on Brooke's Ubuntu install
BAUDRATE = 9600  # Set baud rate for serial connection

# open the serial port
serialPort = serial.Serial(COMPORT, BAUDRATE, timeout=1)

# Create empty lists of values for data gathered from Arduino
dictValues = {
    'Time': [],
    'sensorLL': [],
    'sensorCL': [],
    'sensorCR': [],
    'sensorRR': [],
    'motorL': [],
    'motorR': []
}


def on_press(key):
    """
    Keyboard listener to command Arduino

    Args:
        key (Key): key that has been pressed

    Returns:
        Key | False
    """
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['0', '1', '2']:  # keys of interest
        # Send command to Arduino
        serialPort.write(bytes(k, 'utf-8'))
        if k == '0':
            # If stop command, write data to CSV
            data = pd.DataFrame(dictValues)
            data.to_csv('./currentrun.csv', index=False)

            return False

# Start keyboard listener
listener = keyboard.Listener(on_press=on_press)
listener.start()  # start to listen on a separate thread

while listener.is_alive:

    # Request data line from Arduino
    rawDataLine = serialPort.readline().decode()

    if len(rawDataLine) > 0:

        # Split raw data into time, sensor values, and motor speeds
        elapsedTime, rawLL, rawCL, rawCR, rawRR, motorL, motorR = (
            float(x) for x in rawDataLine.split(','))

        # Save datapoints to lists
        dictValues['Time'].append(elapsedTime)
        dictValues['sensorLL'].append(rawLL)
        dictValues['sensorCL'].append(rawCL)
        dictValues['sensorCR'].append(rawCR)
        dictValues['sensorRR'].append(rawRR)
        dictValues['motorL'].append(motorL)
        dictValues['motorR'].append(motorR)
