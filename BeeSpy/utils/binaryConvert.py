import numpy as np

### FUNCTION TO CONVERT THE ARDUINO BINARY DATA TO CSV
def beespy_arduino_reader(file, offset=64):
  #Data conversion
  with open(file, "rb") as fh:
    fh.seek(offset)
    x = np.frombuffer(fh.read(), dtype=np.int16)
    x = x.reshape(-1, 32)[:, 2:].reshape(-1, 6)
    return x