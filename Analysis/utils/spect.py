from scipy.signal import spectrogram
import numpy as np

def dospectrogram(signal, rate, window_duration=0.2, window_overlap=0):
  nps = max(2, int(round(window_duration * rate)))
  nol = int(round(window_overlap * rate))
  if nol >= nps:
        nol = nps - 1  # Ensure that overlap is less than the window size
  if len(signal) < nps:
      # Signal shorter than one window — return empty arrays so the caller skips this file
      fq = np.linspace(0, rate / 2, nps // 2 + 1)
      return fq, np.array([]), np.zeros((len(fq), 0))
  fq, times, S = spectrogram(x=signal, fs=rate, window="hamming", nperseg=nps, noverlap=nol, detrend="linear")
  if S.size == 0:
      return fq, times, S
  ##convert from log power for ease of plotting
  mn = S.min()
  if mn <= 0:
      S -= mn - 1e-9
  S = np.log(S)
  S[np.isnan(S)] = 0
  return fq, times, S



def plot_spectrograms16(data, sample_rate=2000, window_duration=0.2, 
                     window_overlap=0, max_samples=None):
    """
    Plot spectrograms for all 16 channels using dospectrogram function.
    
    Parameters:
    -----------
    data : numpy.ndarray
        Array with shape (n_samples, n_channels)
    sample_rate : float
        Sampling rate in Hz (default: 2000)
    window_duration : float
        Window duration in seconds (default: 0.2)
    window_overlap : float
        Window overlap in seconds (default: 0)
    max_samples : int, optional
        Maximum number of samples to process (None = use all)
    """
    n_samples, n_channels = data.shape
    
    # Limit samples if specified
    if max_samples is not None and n_samples > max_samples:
        data = data[:max_samples]
        n_samples = max_samples
    
    # Create subplots - 4 rows x 4 columns for 16 channels
    fig, axes = plt.subplots(4, 4, figsize=(16, 12))
    fig.suptitle(f'Spectrograms - 16 Channels (window={window_duration}s)', 
                 fontsize=16, fontweight='bold')
    
    # Flatten axes for easier iteration
    axes_flat = axes.flatten()
    
    for i in range(n_channels):
        ax = axes_flat[i]
        
        # Compute spectrogram using the custom function
        fq, times, S = dospectrogram(data[:, i], sample_rate, 
                                     window_duration, window_overlap)
        
        # Plot spectrogram
        im = ax.pcolormesh(times, fq, S, shading='gouraud', cmap='viridis')
        
        ax.set_title(f'Channel {i}', fontsize=10, fontweight='bold')
        ax.set_ylabel('Frequency (Hz)', fontsize=8)
        ax.set_xlabel('Time (s)', fontsize=8)
        ax.tick_params(labelsize=8)
        
        # Add colorbar
        plt.colorbar(im, ax=ax, label='Log Power', pad=0.01)
    
    plt.tight_layout()
    plt.show()