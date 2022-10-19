"""
Plot a CSV dataset for a run of the line follower course
"""

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Needed this for WSL rendering
matplotlib.use('TkAgg')

# Import CSV of full run
df = pd.read_csv('./currentrun.csv')

# Create objects for full run plots
fig1, ax1 = plt.subplots(3, 1)

# Plot sensor readouts for full run
df.plot(kind='line', x='Time', y='sensorLL',
        color='green', ax=ax1[0], ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorCL',
        color='yellow', ax=ax1[0], ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorCR',
        color='orange', ax=ax1[0], ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorRR',
        color='red', ax=ax1[0], ylim=(30, 60))

# Plot reflectivity threshold line
ax1[0].plot(df['Time'], np.zeros_like(df['Time']) +
            40, '--', label='Reflectivity Threshold')

# Add graph labels
ax1[0].legend(loc='upper right')
ax1[0].set_xlabel("Time (ms)")
ax1[0].set_ylabel('Analog Sensor Response (0-1023)')

# Plot motor responses over full run
df.plot(kind='line', x='Time', y='motorL', color='blue', ax=ax1[1])
df.plot(kind='line', x='Time', y='motorR', color='purple', ax=ax1[1])

# Add graph labels
ax1[1].legend(loc='upper right')
ax1[1].set_xlabel("Time (ms)")
ax1[1].set_ylabel('Commanded Motor Speed (0-255)')

# Plot net motor response (left motor minus right motor)
ax1[2].plot(df['Time'], df['motorL']-df['motorR'], label='motorL - motorR')
ax1[2].plot(df['Time'], np.zeros_like(df['Time']), '--', label='0: Straight')

# Add graph labels
ax1[2].legend(loc='upper right')
ax1[2].set_xlabel("Time (ms)")
ax1[2].set_ylabel('Net Motor Speed (left minus right, -255 - 255)')

# Layout subplots
fig1.tight_layout()


# Create objects for smaller time period plots
fig2, ax2 = plt.subplots(2, 1)

# Plot sensor responses over small time period
df.plot(kind='line', x='Time', y='sensorLL', color='green',
        ax=ax2[0], xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorCL', color='yellow',
        ax=ax2[0], xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorCR', color='orange',
        ax=ax2[0], xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line', x='Time', y='sensorRR', color='red',
        ax=ax2[0], xlim=(10000, 12000), ylim=(30, 60))

# Plot reflectivity threshold line
ax2[0].plot(df['Time'], np.zeros_like(df['Time']) +
            40, ':', label='Reflectivity Threshold')

# Add graph labels
ax2[0].legend(loc='upper right')
ax2[0].set_xlabel("Time (ms)")
ax2[0].set_ylabel('Analog Sensor Response (0-1023)')

# Graph motor response over small time period
df.plot(kind='line', x='Time', y='motorL', color='blue',
        ax=ax2[1], xlim=(10000, 12000), style='--')
df.plot(kind='line', x='Time', y='motorR', color='purple',
        ax=ax2[1], xlim=(10000, 12000), style='--')

# Add graph labels
ax2[1].legend(loc='upper right')
ax2[1].set_xlabel("Time (ms)")
ax2[1].set_ylabel('Commanded Motor Speed (0-255)')

# Layout subplots
fig2.tight_layout()

plt.show()
