import re
import pandas as pd

# Get the start time of a file from it's name - assuming it follows the convention
def extract_start_time(filename):
    # Regex to extract date and time components from the filename
    pattern = r'(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})(?:\.bin_\d+_spec\.csv)?'
    match = re.search(pattern, filename)
    
    if match:
        year, month, day, hour, minute, second = map(int, match.groups())
        return pd.Timestamp(year, month, day, hour, minute, second)
    else:
        raise ValueError(f"Filename format not recognized: {filename}")
    

# This converts a pd.Timestamp value to the same seconds since epoch as QDateTimeEdit...toSecsSinceEpoch()
def timestamp_to_secs_since_epoch(ts):
    # Get the number of days since year 1
    days_since_1 = ts.toordinal()
    # Unix epoch day (1970-01-01)
    unix_epoch_day = pd.Timestamp('1970-01-01').toordinal()
    # Convert days difference to seconds
    seconds_since_epoch = (days_since_1 - unix_epoch_day) * 86400 + ts.hour * 3600 + ts.minute * 60 + ts.second
    return seconds_since_epoch