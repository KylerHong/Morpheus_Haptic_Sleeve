import pandas as pd
import os
import re
import argparse
from pathlib import Path


def get_base_session_name(folder_name):
    """
    Strips trailing session indices (e.g., '_1', '_2') from folder names to determine 
    the parent session group.
    Examples:
        '12022025_SNHF06_Session1_1' -> '12022025_SNHF06_Session1'
        'Session2'                   -> 'Session2'
    """
    match = re.match(r"^(.*)_\d+$", folder_name)
    if match:
        return match.group(1)
    return folder_name


def get_recording_start_time(folder_path):
    """
    Extracts the starting timestamp from a split recording folder to determine 
    its chronological order. Checks 'fixations.csv', 'gaze.csv', then 'surface_positions.csv'.
    """
    candidates = [
        os.path.join(folder_path, "fixations.csv"),
        os.path.join(folder_path, "gaze.csv"),
        os.path.join(folder_path, "surface_positions.csv")
    ]

    target_file = None
    for cand in candidates:
        if os.path.exists(cand):
            target_file = cand
            break

    if not target_file:
        return float('inf')

    try:
        df = pd.read_csv(target_file, nrows=1)
        # Check standard timestamp headers
        for time_col in ['start timestamp [ns]', 'timestamp [ns]', 'section start time [ns]']:
            if time_col in df.columns:
                return df[time_col].iloc[0]
        return float('inf')
    except Exception:
        return float('inf')


def merge_csv_type(folders, filename, session_output_dir, time_col=None):
    """
    Combines instances of a specific CSV file across split subfolders for a single session,
    sorts rows by time, and writes out the merged CSV.
    """
    dfs = []
    found_any = False

    for folder in folders:
        filepath = os.path.join(folder, filename)

        if os.path.exists(filepath):
            found_any = True
            try:
                df = pd.read_csv(filepath)
                dfs.append(df)
            except Exception as e:
                print(f"      [WARN] Could not read {filepath}: {e}")

    if not found_any:
        return

    if dfs:
        merged_df = pd.concat(dfs, ignore_index=True)

        # Sort combined dataset chronologically if timestamp column exists
        if time_col and time_col in merged_df.columns:
            merged_df = merged_df.sort_values(by=time_col)
        elif 'start timestamp [ns]' in merged_df.columns:
            merged_df = merged_df.sort_values(by='start timestamp [ns]')
        elif 'timestamp [ns]' in merged_df.columns:
            merged_df = merged_df.sort_values(by='timestamp [ns]')

        out_path = os.path.join(session_output_dir, filename)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)

        merged_df.to_csv(out_path, index=False)
        print(f"      [SUCCESS] Saved {filename} ({len(merged_df)} total rows)")


def process_session_group(session_name, split_folders, root_output_dir, target_files):
    """
    Sorts split folders for a base session chronologically and merges target CSVs.
    """
    print(f"\n==========================================")
    print(f"Processing Session: {session_name}")
    print(f"==========================================")

    # Sort split folders chronologically based on internal start timestamps
    folders_with_time = []
    for folder in split_folders:
        t = get_recording_start_time(folder)
        folders_with_time.append((t, folder))
        folder_bname = os.path.basename(folder)
        if t != float('inf'):
            print(f"   - Split part: {folder_bname} (Start Time: {t})")
        else:
            print(f"   - Split part: {folder_bname} (Timestamp not found, using folder position)")

    folders_with_time.sort(key=lambda x: x[0])
    ordered_folders = [x[1] for x in folders_with_time]

    # Save output directly inside Merged_Sessions without an extra subfolder
    session_target_dir = root_output_dir
    os.makedirs(session_target_dir, exist_ok=True)

    # Merge each target CSV type across the ordered split folders
    for fname, t_col in target_files:
        merge_csv_type(ordered_folders, fname, session_target_dir, time_col=t_col)


def main():
    parser = argparse.ArgumentParser(
        description="Merge split session recordings (_1, _2) back into single unified session folders."
    )
    parser.add_argument("--input-root", required=True, help="Parent directory containing split session folders.")
    parser.add_argument("--output-dir", required=False, help="Optional output root. Defaults to 'Merged_Sessions' inside input-root.")
    args = parser.parse_args()

    input_root = os.path.abspath(args.input_root)

    if not args.output_dir:
        output_dir = os.path.join(input_root, "Merged_Sessions")
    else:
        output_dir = os.path.abspath(args.output_dir)

    print(f"Scanning root path: {input_root}")
    print(f"Destination path:   {output_dir}")

    # Discover valid recording subfolders
    root = Path(input_root)
    subfolders = []
    output_dir_abs = os.path.abspath(output_dir)

    for f in os.scandir(root):
        if f.is_dir():
            f_abs = os.path.abspath(f.path)
            if f_abs != output_dir_abs:
                subfolders.append(f.path)

    if not subfolders:
        print(f"[ERROR] No subfolders found in {input_root}")
        return

    # Group split folders by base session name
    session_groups = {}
    for folder_path in subfolders:
        folder_name = os.path.basename(folder_path)
        base_name = get_base_session_name(folder_name)
        session_groups.setdefault(base_name, []).append(folder_path)

    print(f"Identified {len(session_groups)} unique session group(s) across {len(subfolders)} subfolder(s).")

    # Primary target datasets for eye tracking & surface position data
    target_files = [
        ('fixations.csv', 'start timestamp [ns]'),
        ('gaze.csv', 'timestamp [ns]'),
        ('surface_positions.csv', 'timestamp [ns]'),
        (os.path.join('surfaces', 'fixations.csv'), 'start timestamp [ns]'),
        (os.path.join('surfaces', 'gaze_on_surfaces.csv'), 'timestamp [ns]'),
        (os.path.join('surfaces', 'surface_positions.csv'), 'timestamp [ns]')
    ]

    # Run re-merging logic per session
    for session_name, split_folders in session_groups.items():
        process_session_group(session_name, split_folders, output_dir, target_files)

    print(f"\n[DONE] All session splits successfully merged into: {output_dir}")


if __name__ == "__main__":
    main()