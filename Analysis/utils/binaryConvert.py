import numpy as np
### FUNCTION TO CONVERT THE ARDUINO BINARY DATA TO CSV
def beespy_arduino_reader(file, offset=64):
  #Data conversion
  if isinstance(file, str):
    with open(file, 'rb') as f:
      f.seek(offset)
      x = np.frombuffer(f.read(), dtype=np.int16)
  else:
    file.seek(offset)
    x = np.frombuffer(file.read(), dtype=np.int16)
  xx = x.reshape(-1, 32)[:, 2:].reshape(-1, 6)
  return xx


def beespy_arduino_reader16(file, offset=64):
    if isinstance(file, str):
        with open(file, 'rb') as f:
            f.seek(offset)
            x = np.frombuffer(f.read(), dtype=np.int16)
    else:
        file.seek(offset)
        x = np.frombuffer(file.read(), dtype=np.int16)
    
    xx = x.reshape(-1, 32)[:, 2:18].flatten()  # Remove headers, flatten to 1D
    
    # Trim to multiple of 16 to get complete samples only
    n_samples = int(np.floor(len(xx) / 16))
    
    # Reshape into 16 channels
    xxx = xx[0:n_samples*16].reshape(-1, 16)
    return xxx