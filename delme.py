import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt




sig = np.cumsum(randn(800)) 
plt.plot(sig, color='silver', label='Original')

plt.ylabel('Magnitude [dB]')
plt.xlabel('Frequency [Hz]\nSampling 1kHz')
plt.grid(True)
plt.legend()
plt.show()