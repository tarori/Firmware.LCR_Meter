#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv
# import fireducks.pandas as pd
import pandas as pd
import numpy as np
from scipy import signal
import sys

Fs = 12e+6 * 23 / 250
dt = 1 / Fs
adc_full_scale = 2**15
adc_full_voltage = 3.3

if len(sys.argv) == 1:
    print("ファイルを指定してにょ")
    exit(1)

raw_data = pd.read_csv(sys.argv[1], names=['I', 'V'])
raw_data = raw_data['I']
try:
  raw_data = raw_data.astype(float)
except:
  raw_data = raw_data[1:]
  raw_data = raw_data.astype(float)

adc_data = raw_data / adc_full_scale - 1
adc_data_voltage = adc_data * adc_full_voltage

N = len(adc_data)

print('Data:', min(adc_data), '~', max(adc_data))
print("SNR Full-BW:", 10*np.log10((1/2) / np.mean((adc_data-adc_data.mean())**2)), "dB")

plt.plot(adc_data)
plt.xlabel('t (sample)')
plt.ylabel('Signal (FS)')
plt.show()

#window = signal.windows.kaiser(N, beta=20)
window = signal.windows.boxcar(N)
window_signal_factor = np.mean(window)
window_noise_factor = np.sqrt(np.mean(window**2))
print(window_signal_factor, window_noise_factor, window_noise_factor/window_signal_factor)

data_windowed = adc_data - adc_data.mean()
data_windowed = data_windowed * window / window_signal_factor

dft_val = np.fft.fft(data_windowed)
dft_freq = np.fft.fftfreq(N, d=dt)
dft_amp = np.abs(dft_val) / (N/2)
dft_dB = 20 * np.log10(dft_amp)

SNR_start_freq = 20
SNR_end_freq = 20e+3
SNR_start_index = int(SNR_start_freq/(Fs/N))
SNR_end_index = int(SNR_end_freq/(Fs/N))
noise_power = sum(dft_amp[SNR_start_index:SNR_end_index] ** 2) * (window_signal_factor/window_noise_factor) **2
SNR_dB = 10 * np.log10(1 / noise_power)
print("SNR BW:", SNR_dB, "dB")

plt.plot(dft_freq[1:int(N/2)], dft_dB[1:int(N/2)])
plt.xlabel('Frequency (Hz)')
plt.ylabel('Signal (dBFS)')
plt.xscale('log')
plt.yscale('linear')
plt.xlim([Fs/N*8, Fs/2])
plt.ylim([-140, 0])
plt.grid()
plt.show()

plt.plot(dft_freq[1:int(N/2)], dft_dB[1:int(N/2)])
plt.xlabel('Frequency Hz)')
plt.ylabel('Signal (dBFS)')
plt.xscale('linear')
plt.yscale('linear')
#plt.xticks(np.arange(0, Fs/2+1, Fs/8))
plt.ylim([-140, 0])
plt.grid()
plt.show()
