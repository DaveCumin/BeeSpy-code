from utils import binaryConvert as bc
from utils import denoiseSignal as denoise
from utils import spect
import numpy as np
import os
import re
import pandas as pd
import matplotlib.pyplot as plt

folder = "/Volumes/Untitled/"
folder_out = folder
files = ["2025_03_18_01_18_19.bin"]


def convertToSpec(folderIN, files, folderOUT, saveRaw=False, saveSpec=True):
    for f in files:
        filepath = os.path.join(folderIN, f)
        ## convert from binary
        x = bc.beespy_arduino_reader(filepath)
        # Denoise and spectrogram each channel, save spec in the processed folder
        for c in range(x.shape[1]):
            if(folderOUT != None & saveRaw):
                ##save the raw data
                file_path = os.path.join(folderOUT, f"{os.path.basename(filepath)}_{str(c)}_raw.csv")
                np.savetxt(file_path, x[:,c], delimiter=",")
            ##denoise
            denoised = denoise.umw_denoise(x[:, c], 5, 3)
            fq, ts, spec = spect.dospectrogram(denoised, 5000, window_duration=0.2)
            # Insert `fq` as the first row and `ts` as the first column of `spec`
            expanded_spec = np.zeros((spec.shape[0] + 1, spec.shape[1] + 1))  # Adding 1 row and 1 column
            expanded_spec[0, 1:] = ts
            expanded_spec[1:, 0] = fq
            expanded_spec[1:, 1:] = spec
            # Save spec in the processed folder
            #file_path = os.path.join(app.config['PROCESSED_FOLDER'], f"{os.path.basename(filepath)}_{str(c)}_spec.csv")
            if(folderOUT != None & saveSpec):
                file_path = os.path.join(folderOUT, f"{os.path.basename(filepath)}_{str(c)}_spec.csv")
                np.savetxt(file_path, expanded_spec, delimiter=",")


# Function to extract start time from filename
def extract_start_time(filename):
    # Regex to extract date and time components from the filename
    pattern = r'(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})(?:\.bin_\d+_spec\.csv)?'
    match = re.search(pattern, filename)
    
    if match:
        year, month, day, hour, minute, second = map(int, match.groups())
        return pd.Timestamp(year, month, day, hour, minute, second)
    else:
        raise ValueError(f"Filename format not recognized: {filename}")

# Load and stitch CSV files
def stitch_files(file_list):
    combined_df = None
    startTime = None
    for file in file_list:
        # Load each CSV file
        df = pd.read_csv(file, delimiter=',', header=None)
        # Extract the start time from the filename
        start_time = extract_start_time(file)
        ##Set the overall start time if first
        if startTime is None:
            startTime = start_time
        ##else, adjust the times accordingly
        else:
            time_offset = (start_time - startTime).total_seconds()
            df.iloc[0, :] = pd.to_numeric(df.iloc[0, :], errors='coerce')  # Convert to numeric, set non-numeric as NaN
            df.iloc[0, :] = df.iloc[0, :] + time_offset  # Add time offset
        # Combine the dataframes, aligning them based on frequency (rows) and time (columns)
        if combined_df is None:
            combined_df = df  # Initialize with the first dataframe
        else:
            # Join on the frequency columns (first column), keeping both time columns
            combined_df = pd.concat([combined_df, df.iloc[:, 1:]], axis=1)
    return combined_df

def plot_spec(data):
    ## get the frequency and time values
    freq_values = data.iloc[1:, 0].values
    time_values = data.iloc[0, 1:].values
    ## get the amplitude data
    amplitude_values = data.iloc[1:, 1:].values
    ##Plot it
    plt.figure(figsize=(10, 8))
    plt.imshow(amplitude_values.T, aspect='auto', extent=[time_values[0], time_values[-1], freq_values[0], freq_values[-1]], cmap='viridis', origin='lower')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Frequency (Hz)')
    plt.title('Spectrogram')
    plt.colorbar(label='Amplitude (dB)')
    # Display the plot
    plt.show()


def plot_multiple_specs(dataframes, titles):
    # Create a figure with a grid of 2 rows and 3 columns
    fig, axs = plt.subplots(2, 3, figsize=(20, 10))  # Adjust figsize as needed
    # Flatten the axs array to iterate over it easily
    axs = axs.flatten()
    for ax, df, title in zip(axs, dataframes, titles):
        # Extract time and frequency
        freq_values = df.iloc[1:, 0].values  # Skip the first row (frequency values)
        time_values = df.iloc[0, 1:].values  # Skip the first column (time values)  
        # Extract the amplitude data (remaining values)
        amplitude_values = df.iloc[1:, 1:].values  # Data starts from row 1 and column 1
        # Use imshow to plot the spectrogram in the current subplot
        im = ax.imshow(amplitude_values, aspect='auto', 
                       cmap='viridis', origin='lower')
        
        time_stamps = pd.to_datetime(time_values, unit='s')
        num_ticks_x = 4
        tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=num_ticks_x, dtype=int)
        tick_labels_x = time_stamps[tick_indices_x].strftime('%Y-%m-%d %H:%M:%S')
        # Apply tick positions and labels to the x-axis
        ax.set_xticks(tick_indices_x)
        ax.set_xticklabels(tick_labels_x, ha='center') 

        # Apply tick positions and labels to the y-axis
        num_ticks_y = 5
        tick_indices_y = np.linspace(0, len(freq_values) - 1, num=num_ticks_y, dtype=int)
        tick_labels_y = freq_values[tick_indices_y]
        ax.set_yticks(tick_indices_y)
        ax.set_yticklabels(tick_labels_y, ha='center')
    
        # Add labels and title for each subplot
        ax.set_xlabel('Time')
        ax.set_ylabel('Frequency (Hz)')
        ax.set_title(title)
        # Add a colorbar to each subplot
        fig.colorbar(im, ax=ax, label='Amplitude (dB')
    # Adjust layout
    plt.tight_layout()
    # Display the plot
    plt.show()



### DO THE BUSINESS
convertToSpec(folder, files, folder_out)


### plot
spec_data_raw = []
for f in files:
    for channel in range(6):
        filename = f"{folder_out}{f}_{channel}_spec.csv"
        print(filename)
        spec_data = pd.read_csv(filename, delimiter=',', header=None)
        freq_values = spec_data.iloc[1:, 0].values  # Skip the first row (frequency values)
        time_values = spec_data.iloc[0, 1:].values  # Skip the first column (time values)
        # Extract the amplitude data (remaining values)
        amplitude_values = spec_data.iloc[1:, 1:].values  # Data starts from row 1 and column 1
        if len(spec_data_raw) < channel + 1:
            spec_data_raw.append(amplitude_values)
        else:
            spec_data_raw[channel] = np.concatenate((spec_data_raw[channel], amplitude_values), axis=1)

## plot it
for channel in range(6):
    axis = plt.subplot(3, 2, channel+1)
    plt.imshow(spec_data_raw[channel], aspect='auto',                 
                    cmap='viridis', origin='lower')
    
    start_time_seconds = extract_start_time(f).timestamp()
    time_delta = pd.Timedelta(seconds=start_time_seconds)
    time_stamps = pd.to_datetime(time_values, unit='s') + time_delta
    num_ticks_x = 2 
    tick_indices_x = np.linspace(0, len(time_stamps) - 1, num=num_ticks_x, dtype=int)
    tick_labels_x = time_stamps[tick_indices_x].strftime('%Y-%m-%d %H:%M:%S')
    # Apply tick positions and labels to the x-axis
    axis.set_xticks(tick_indices_x)
    axis.set_xticklabels(tick_labels_x, ha='center') 

    # Apply tick positions and labels to the y-axis
    freqVals = freq_values
    num_ticks_y = 5
    tick_indices_y = np.linspace(0, len(freqVals) - 1, num=num_ticks_y, dtype=int)
    tick_labels_y = freqVals[tick_indices_y]
    axis.set_yticks(tick_indices_y)
    axis.set_yticklabels(tick_labels_y, ha='center')
plt.show()

