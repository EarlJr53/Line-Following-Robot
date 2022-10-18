"""
_summary_
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('./currentrun.csv')

fig, axs = plt.subplots(3, 1) 

df.plot(kind='line',x='Time',y='sensorLL', color='green', ax=axs[0], ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorCL', color='yellow', ax=axs[0], ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorCR', color='orange', ax=axs[0], ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorRR', color='red', ax=axs[0], ylim=(30, 60))
axs[0].plot(df['Time'], np.zeros_like(df['Time']) + 40, '--', label='Reflectivity Threshold')
axs[0].legend()

axs[0].set_ylabel('Analog Sensor Response (0-1023)')

df.plot(kind='line',x='Time',y='motorL', color='blue', ax=axs[1])
df.plot(kind='line',x='Time',y='motorR', color='purple', ax=axs[1])

axs[1].set_ylabel('Commanded Motor Speed (0-255)')

axs[2].plot(df['Time'], df['motorL']-df['motorR'], label='motorL - motorR')
axs[2].plot(df['Time'], np.zeros_like(df['Time']), '--', label='0: Straight')
axs[2].legend()

fig.tight_layout()

fig2, ax1 = plt.subplots()

df.plot(kind='line',x='Time',y='sensorLL', color='green', ax=ax1, xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorCL', color='yellow', ax=ax1, xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorCR', color='orange', ax=ax1, xlim=(10000, 12000), ylim=(30, 60))
df.plot(kind='line',x='Time',y='sensorRR', color='red', ax=ax1, xlim=(10000, 12000), ylim=(30, 60))
ax1.plot(df['Time'], np.zeros_like(df['Time']) + 40, ':', label='Reflectivity Threshold')

ax2 = ax1.twinx()
df.plot(kind='line',x='Time',y='motorL', color='blue', ax=ax2, style='--')
df.plot(kind='line',x='Time',y='motorR', color='purple', ax=ax2, style='--')

plt.show()
