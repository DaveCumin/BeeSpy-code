from scipy.signal import spectrogram
import numpy as np

def dospectrogram(signal, rate, window_duration=0.2, window_overlap=0):
  nps = max(2, int(round(window_duration * rate)))
  nol = int(round(window_overlap * rate)) 
  if nol >= nps:
        nol = nps - 1  # Ensure that overlap is less than the window size
  fq, times, S = spectrogram(x=signal, fs=rate, window="hamming", nperseg=nps, noverlap=nol, detrend="linear") 
  ##convert from log power for ease of plotting
  mn = S.min()
  if mn <= 0:
      S -= mn - 1e-9      
  S = np.log(S)    
  S[np.isnan(S)] = 0
  return fq, times, S