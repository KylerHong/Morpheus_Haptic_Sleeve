import pandas as pd
import os
import argparse
from pathlib import Path


def get_recording_start_time(folder_path):
    """
    Extracts the starting timestamp from a recording folder to determine its chronological order.
    Checks 'fixations.csv' first, then falls back to 'gaze.csv'.
    """
    # Check if fixations.csv exists, otherwise try gaze.csv
    fix_path = os.path.join(folder_path, "fixations.csv")
    if not os.path.exists(fix_path):
        gaze_path = os.path.join(folder_path, "gaze.csv")
        if os.path.exists(gaze_path):
            fix_path = gaze_path
        else:
            return float('inf')

    # Read the first row to get the start timestamp
    try:
        df = pd.read_csv(fix_path, nrows=1)
        if 'start timestamp [ns]' in df.columns:
            return df['start timestamp [ns]'].iloc[0]
        elif 'timestamp [ns]' in df.columns:
            return df['timestamp [ns]'].iloc[0]
        else:
            return float('inf')
    except Exception:
        return float('inf')


def merge_csv_type(folders, filename, output_dir, time_col=None):
    """
    Iterates through all recording folders, combines a specific CSV file type,
    sorts it by time, and saves the merged result.
    """
    dfs = []
    print(f"   -> Processing {filename}...")

    found_any = False

    # Loop through each folder to collect the specific file (e.g., gaze.csv)
    for folder in folders:
        filepath = os.path.join(folder, filename)

        if os.path.exists(filepath):
            found_any = True
            try:
                df = pd.read_csv(filepath)
                dfs.append(df)
            except Exception as e:
                print(f"      [WARN] Could not read {filepath}: {e}")
        else:
            pass

    # Handle case where file isn't found in any folder
    if not found_any:
        print(f"      [INFO] File {filename} not found in any recordings. Skipping.")
        return

    # If data was found, concatenate, sort, and save
    if dfs:
        merged_df = pd.concat(dfs, ignore_index=True)

        # Sort by timestamp if the column is present
        if time_col and time_col in merged_df.columns:
            merged_df = merged_df.sort_values(by=time_col)

        # Ensure output directory exists and save the file
        out_path = os.path.join(output_dir, filename)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)

        merged_df.to_csv(out_path, index=False)
        print(f"      [SUCCESS] Merged {len(merged_df)} rows to {out_path}")


def main():
    """
    Main entry point. Handles argument parsing, folder discovery, chronological sorting,
    and executes the merge for standard Pupil Labs files.
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Merge multiple Pupil Labs recordings into one chronological dataset.")
    parser.add_argument("--input-root", required=True, help="Parent folder containing all the recording folders.")
    parser.add_argument("--output-dir", required=False,
                        help="Optional. Defaults to 'Merged_Recording' inside input-root.")
    args = parser.parse_args()

    input_root = os.path.abspath(args.input_root)

    # Determine output path logic
    if not args.output_dir:
        output_dir = os.path.join(input_root, "Merged_Recording")
    else:
        provided_out = os.path.abspath(args.output_dir)
        if provided_out == input_root:
            output_dir = os.path.join(input_root, "Merged_Recording")
        else:
            output_dir = provided_out

    print(f"Scanning for recordings in: {input_root}")
    print(f"Output will be saved to:    {output_dir}")

    # Scan the root directory for valid subfolders (excluding the output dir itself)
    root = Path(input_root)
    subfolders = []

    output_dir_abs = os.path.abspath(output_dir)

    for f in os.scandir(root):
        if f.is_dir():
            f_abs = os.path.abspath(f.path)
            if f_abs == output_dir_abs:
                continue
            subfolders.append(f.path)

    if not subfolders:
        print(f"[ERROR] No subfolders found in {input_root}")
        return

    # Sort the found folders chronologically based on their internal timestamps
    print(f"Found {len(subfolders)} folders. Sorting chronologically...")

    sorted_folders_data = []
    for f in subfolders:
        t = get_recording_start_time(f)
        if t != float('inf'):
            sorted_folders_data.append((t, f))
            print(f"   - Found timestamp {t} in: {os.path.basename(f)}")
        else:
            print(f"   [WARN] Could not find valid timestamp in {os.path.basename(f)}. Skipping.")

    sorted_folders_data.sort(key=lambda x: x[0])
    sorted_folders = [x[1] for x in sorted_folders_data]

    if not sorted_folders:
        print("[ERROR] No valid recordings found to merge.")
        return

    print("\nMerge Order:")
    for f in sorted_folders:
        print(f" -> {os.path.basename(f)}")

    os.makedirs(output_dir, exist_ok=True)

    # Define the standard list of CSV files to look for and their timestamp columns
    standard_files = [
        ('fixations.csv', 'start timestamp [ns]'),
        ('gaze.csv', 'timestamp [ns]'),
        ('imu.csv', 'timestamp [ns]'),
        ('blinks.csv', 'start timestamp [ns]'),
        ('saccades.csv', 'start timestamp [ns]'),
        ('3d_eye_states.csv', 'timestamp [ns]'),
        ('events.csv', 'timestamp [ns]')
    ]

    # Execute merge for standard files
    print("\n--- Merging Main Data ---")
    for fname, t_col in standard_files:
        merge_csv_type(sorted_folders, fname, output_dir, time_col=t_col)

    # Execute merge for surface data (located in subfolders)
    print("\n--- Merging Surface Data ---")
    merge_csv_type(sorted_folders, os.path.join('surfaces', 'fixations.csv'), output_dir,
                   time_col='start timestamp [ns]')
    merge_csv_type(sorted_folders, os.path.join('surfaces', 'gaze_on_surfaces.csv'), output_dir,
                   time_col='timestamp [ns]')

    print(f"\n[DONE] Successfully created Merged Recording.")
    print(f"Location: {output_dir}")


if __name__ == "__main__":
    main()