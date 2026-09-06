import pandas as pd
import os

"""
The point of this code is to break down the large data sets given
by the pupil labs clouds into folders that we can move around into the box.
I ran this code locally on the Marker_Mapping folder that I got straight from
the pupil clouds to make it simpler rather than having to input the folder paths
"""

# Configuration Variables 
SECTIONS_FILE = "sections.csv"
LARGE_DATA_FILES = ["fixations.csv", "gaze.csv", "surface_positions.csv"]

# Column headers matching sections.csv
WEARER_COLUMN = "wearer name"
TIME_COLUMN = "section start time [ns]"
SECTION_ID_COLUMN = "section id"
RECORDING_ID_COLUMN = "recording id"

# Processing batch size to save on RAM
CHUNK_SIZE = 100000 


def create_folder_mapping(sections_path):
    """
    Reads sections.csv, identifies duplicate wearer sessions, sorts them 
    chronologically by start time, and maps IDs to unified session folders.
    """
    print(f"Loading {sections_path} to build folder mapping...")
    
    if not os.path.exists(sections_path):
        print(f"Error: {sections_path} not found.")
        return None

    sections_df = pd.read_csv(sections_path)

    # Sort chronologically by start time
    if TIME_COLUMN in sections_df.columns:
        sections_df = sections_df.sort_values(by=[WEARER_COLUMN, TIME_COLUMN])

    mapping = {}

    # Group by wearer name to identify repeated recording sessions
    for wearer, group in sections_df.groupby(WEARER_COLUMN, sort=False):
        has_multiple_sessions = len(group) > 1
        
        for idx, (_, row) in enumerate(group.iterrows(), start=1):
            # Format folder name: append _1, _2 if repeated, otherwise use exact wearer name
            folder_name = f"{wearer}_{idx}" if has_multiple_sessions else str(wearer)
            
            # Map both section id and recording id to the unified session folder
            if SECTION_ID_COLUMN in row and pd.notna(row[SECTION_ID_COLUMN]):
                mapping[row[SECTION_ID_COLUMN]] = folder_name
            if RECORDING_ID_COLUMN in row and pd.notna(row[RECORDING_ID_COLUMN]):
                mapping[row[RECORDING_ID_COLUMN]] = folder_name

    return mapping


def process_large_files(data_files, mapping):
    """
    Loops through fixations.csv, gaze.csv, and surface_positions.csv in chunks,
    appending slices into their corresponding unified session folder.
    """
    for file_path in data_files:
        if not os.path.exists(file_path):
            print(f"Skipping {file_path} - File not found.")
            continue
            
        base_filename = os.path.basename(file_path)
        print(f"\nProcessing dataset: {base_filename}...")

        # Detect which ID header column is used in the data file
        sample = pd.read_csv(file_path, nrows=1)
        data_id_col = None
        
        for col in [SECTION_ID_COLUMN, RECORDING_ID_COLUMN]:
            if col in sample.columns:
                data_id_col = col
                break

        # Fallback to the first column if no explicit matching header name is found
        if data_id_col is None:
            data_id_col = sample.columns[0]

        # Stream giant file in chunks
        chunk_iterator = pd.read_csv(file_path, chunksize=CHUNK_SIZE)
        
        for chunk_number, chunk in enumerate(chunk_iterator, start=1):
            print(f"  -> Processing chunk {chunk_number} for {base_filename}")
            
            for id_val, group_data in chunk.groupby(data_id_col):
                if id_val in mapping:
                    target_folder = mapping[id_val]
                    
                    # Create the unified session directory if it does not exist
                    os.makedirs(target_folder, exist_ok=True)
                    
                    # Save as fixations.csv, gaze.csv, or surface_positions.csv inside the session folder
                    out_path = os.path.join(target_folder, base_filename)
                    
                    write_header = not os.path.exists(out_path)
                    group_data.to_csv(out_path, mode='a', index=False, header=write_header)


if __name__ == "__main__":
    folder_mapping = create_folder_mapping(SECTIONS_FILE)
    
    if folder_mapping:
        process_large_files(LARGE_DATA_FILES, folder_mapping)
        print("\nAll files sliced and organized successfully into session folders.")