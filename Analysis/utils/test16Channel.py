import numpy as np
import matplotlib.pyplot as plt

def beespy_arduino_reader(file, offset=64, block_size=32, skip_values=2, num_channels=16, dtype=np.int16, scale_factor=1.0):
    """
    Read binary file with configurable parameters for debugging.
    
    Parameters:
    - file: Path to binary file
    - offset: Bytes to skip (header)
    - block_size: Number of values per block
    - skip_values: Number of values to skip per block
    - num_channels: Number of channels (16)
    - dtype: Data type (e.g., np.int16, np.uint8)
    - scale_factor: Scaling factor to adjust values (e.g., to match ~220)
    
    Returns:
    - NumPy array of shape (n_samples, num_channels)
    """
    with open(file, "rb") as fh:
        fh.seek(offset)
        # Read all data as specified dtype
        x = np.frombuffer(fh.read(), dtype=dtype)
        print(f"Raw data shape: {x.shape}, min: {x.min()}, max: {x.max()}")
        print(f"Data type: {x.dtype}")
        print(f"Data: {x[:32]}")
        
        # Check if data size is compatible with block size
        if x.size % block_size != 0:
            raise ValueError(f"Data size {x.size} not divisible by block size {block_size}")
        
        # Reshape into blocks
        x = x.reshape(-1, block_size)
        print(f"Blocks shape: {x.shape}")
        
        # Slice channels (skip first `skip_values`, take next `num_channels`)
        start_idx = skip_values
        end_idx = skip_values + num_channels
        if end_idx > block_size:
            raise ValueError(f"Cannot extract {num_channels} channels with skip {skip_values} from block size {block_size}")
        x = x[:, start_idx:end_idx]
        print(f"Channels shape: {x.shape}")
        
        print(f"Data min: {x.min()}, max: {x.max()}")
        
        return x
    
file_path = '/Volumes/testing/2025_09_09_13_40_46.bin'
file_path = '/Volumes/testing/2025_09_09_13_24_58.bin'
data = beespy_arduino_reader(
        file=file_path,
        offset=64,
        block_size=32,        # Adjust if block size differs
        skip_values=2,        # Adjust if metadata values changed
        num_channels=16,
        dtype=np.int16       # Test np.uint8 or np.uint16 if needed
    )

plt.figure(figsize=(12, 6))  # Wide figure to accommodate multiple lines
for channel_idx in range(16):
    plt.plot(data[:, channel_idx], label=f"Channel {channel_idx}", linewidth=1)

# Customize the plot
plt.xlabel("Sample Index")
plt.ylabel("Channel Value")
plt.title("All 16 Channels")
plt.legend(loc="upper right", ncol=4, fontsize=8)  # Compact legend with 4 columns
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Show the plot
plt.show()