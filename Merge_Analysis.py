import os
import glob
import pandas as pd
import argparse
import re

def get_trial_number(filepath):
    """Extracts the first number found in the folder name to enforce numerical sorting."""
    basename = os.path.basename(filepath)
    match = re.search(r'\d+', basename)
    return int(match.group()) if match else 99999

def main():
    parser = argparse.ArgumentParser(description="Merge Analysis Output Folders")
    parser.add_argument("--analysis-dir", required=True, help="Path to the master Analysis_Output folder")
    args = parser.parse_args()

    master_dir = args.analysis_dir
    
    # Locate the parent directory of analysis-dir and create Merged_Data there
    parent_dir = os.path.dirname(os.path.normpath(master_dir))
    merged_dir = os.path.join(parent_dir, "Merged_Data")
    os.makedirs(merged_dir, exist_ok=True)
    print(f"Output directory ready at: {merged_dir}")

    # Find all trial folders and sort them NUMERICALLY using the helper function
    trial_folders = glob.glob(os.path.join(master_dir, "*_Analysis"))
    trial_folders = sorted(trial_folders, key=get_trial_number)
    print(f"Found {len(trial_folders)} trial folders. Merging in numerical order...")

    # Define the file patterns to merge
    file_types = [
        "full_timeseries",
        "raw_gaze_slice",
        "normalized_gaze_slice",
        "raw_fixations_slice",
        "normalized_fixation_slice",
        "raw_3d_eye_states_slice",
        "raw_blinks_slice",
        "raw_saccades_slice",
        "key_metrics"
    ]

    for ftype in file_types:
        all_dfs = []
        for folder in trial_folders:
            # Find matching file in this specific trial folder
            file_path = glob.glob(os.path.join(folder, f"*{ftype}.csv"))
            if file_path:
                try:
                    df = pd.read_csv(file_path[0])
                    # Stamp the data with the trial it came from before merging
                    trial_name = os.path.basename(folder).replace('_Analysis', '')
                    df.insert(0, 'Source_Trial', trial_name)
                    all_dfs.append(df)
                except Exception as e:
                    print(f"Could not read {file_path[0]}: {e}")

        # Concatenate and save to the Merged_Data folder
        if all_dfs:
            merged_df = pd.concat(all_dfs, ignore_index=True)
            out_path = os.path.join(merged_dir, f"merged_{ftype}.csv")
            merged_df.to_csv(out_path, index=False)
            print(f"Saved {out_path} ({len(merged_df)} rows)")

if __name__ == "__main__":
    main()