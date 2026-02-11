import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import seaborn as sns
import os
import argparse
import glob


def standardize_columns(df):
    """
    Standardizes coordinate column names to 'x_norm' and 'y_norm'.
    """
    renaming_map = {
        'gaze position on surface x [normalized]': 'x_norm',
        'gaze position on surface y [normalized]': 'y_norm',
        'norm_pos_x': 'x_norm',
        'norm_pos_y': 'y_norm',
        'gaze_normal0_x': 'x_norm',
        'gaze_normal0_y': 'y_norm',
    }
    df = df.rename(columns=renaming_map)
    return df


def generate_heatmap(df, title, subtitle_info, output_path):
    """
    Generates and saves the final heatmap using a Kernel Density Estimate (KDE).
    """
    if 'x_norm' not in df.columns or 'y_norm' not in df.columns:
        print(f"Error: Missing coordinate columns. Found: {list(df.columns)}")
        return

    if df.empty:
        print("Error: Dataset is empty. Cannot generate heatmap.")
        return

    plt.figure(figsize=(10, 8))

    # Filter out invalid coordinates
    df = df[(df['x_norm'] >= 0) & (df['x_norm'] <= 1) &
            (df['y_norm'] >= 0) & (df['y_norm'] <= 1)]

    # Optimization: Downsample if dataset is huge
    MAX_POINTS = 30000
    if len(df) > MAX_POINTS:
        stride = len(df) // MAX_POINTS
        print(f"   -> Dataset large ({len(df)} points). Downsampling to ~{MAX_POINTS}...")
        df = df.iloc[::stride]

    if len(df) < 10:
        print("Not enough valid data points to plot.")
        plt.close()
        return

    # Flip Y axis (Screen coordinates 0,0 is top-left)
    x = df['x_norm']
    y = 1 - df['y_norm']

    try:
        colors = ['#4b0082', '#0000ff', '#00ffff', '#00ff00', '#ffff00', '#ff8c00', '#ff0000']
        weather_cmap = mcolors.LinearSegmentedColormap.from_list("weather_map", colors, N=256)
        sns.kdeplot(x=x, y=y, fill=True, cmap=weather_cmap, alpha=0.8, levels=60, thresh=0.05)
    except Exception as e:
        print(f"Plotting Error: {e}")
        plt.close()
        return

    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.suptitle(title, fontsize=18, y=0.98, fontweight='bold')
    plt.title(subtitle_info, fontsize=10)
    plt.axis('off')

    # Draw screen border and crosshair
    plt.gca().add_patch(plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor='black', lw=3))
    plt.axvline(x=0.5, color='black', linestyle='-', linewidth=2)
    plt.axhline(y=0.5, color='black', linestyle='-', linewidth=2)
    plt.tight_layout(rect=[0, 0, 1, 1])

    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\nSuccess! Heatmap saved to: {output_path}")


def get_surface_data_for_trial(trial_folder, surface_df):
    """
    Extracts the specific chunk of surface data that corresponds to a single trial
    by matching timestamps from the analysis slice.
    """
    gaze_files = glob.glob(os.path.join(trial_folder, "*_raw_gaze_slice.csv"))
    if not gaze_files:
        return None

    try:
        gaze_slice = pd.read_csv(gaze_files[0])
        if gaze_slice.empty:
            return None

        start_ns = gaze_slice['timestamp [ns]'].min()
        end_ns = gaze_slice['timestamp [ns]'].max()

        t_col = 'timestamp [ns]'
        if t_col not in surface_df.columns and 'gaze_timestamp' in surface_df.columns:
            t_col = 'gaze_timestamp'

        mask = (surface_df[t_col] >= start_ns) & (surface_df[t_col] <= end_ns)
        return surface_df[mask].copy()
    except:
        return None


def main():
    parser = argparse.ArgumentParser(description="Generate a custom aggregate heatmap.")

    # 1. Location for Raw Surface Data (Original Session Folders)
    parser.add_argument("--root-dir", required=True,
                        help="Path to the main data folder containing raw session folders (e.g., BEAR Lab)")

    # 2. Location for Analysis Output (The new folder in your screenshot)
    parser.add_argument("--analysis-dir", required=True,
                        help="Path to the folder containing all the _Analysis_Output folders")

    parser.add_argument("--ids", nargs='+', required=True, help="List of participant IDs (e.g., 01 03 04)")
    parser.add_argument("--sessions", nargs='+', required=True, help="List of sessions (e.g., 1 2)")
    parser.add_argument("--trial-range", nargs=2, type=int, required=True, help="Start and End trial numbers")
    parser.add_argument("--exclude-trials", nargs=2, type=int, default=None,
                        help="Optional: Start and End trial numbers to EXCLUDE")
    parser.add_argument("--output-name", default="Custom_Heatmap.png", help="Filename for the output image")

    args = parser.parse_args()

    start_t, end_t = args.trial_range
    trial_list = list(range(start_t, end_t + 1))

    if args.exclude_trials:
        ex_start, ex_end = args.exclude_trials
        exclude_set = set(range(ex_start, ex_end + 1))
        trial_list = [t for t in trial_list if t not in exclude_set]
        exclusion_str = f"(Excluded: {ex_start}-{ex_end})"
    else:
        exclusion_str = ""

    print("-" * 40)
    print("Custom Heatmap Generator (Split Directory Mode)")
    print(f"Participants: {args.ids}")
    print(f"Sessions:     {args.sessions}")
    print(f"Trials:       {trial_list} {exclusion_str}")
    print("-" * 40)

    if not trial_list:
        print("Error: No trials selected.")
        return

    collected_data = []

    for pid in args.ids:
        for sess in args.sessions:
            # --- 1. Find the Analysis Folder (in the new location) ---
            # Pattern matches the folder names seen in your screenshot: "SHF01_Session1_Analysis_Output"
            analysis_search_pattern = os.path.join(args.analysis_dir, f"*{pid}_Session{sess}_Analysis_Output")
            analysis_matches = glob.glob(analysis_search_pattern)

            if not analysis_matches:
                print(f"Warning: No Analysis folder found for {pid} Session {sess} in {args.analysis_dir}")
                continue

            analysis_path = analysis_matches[0]
            session_name = os.path.basename(analysis_path).replace("_Analysis_Output", "")
            print(f"Processing: {session_name}...")

            # --- 2. Find the Raw Surface Data (in the original root location) ---
            # We search the root dir for the session folder to get the raw 'gaze.csv'
            raw_session_pattern = os.path.join(args.root_dir, f"*{pid}_Session{sess}*")
            raw_session_folders = glob.glob(raw_session_pattern)

            # Filter out the analysis folder if it happens to be inside root and match the pattern
            raw_session_folders = [f for f in raw_session_folders if "_Analysis_Output" not in f]

            if not raw_session_folders:
                print(f"   -> Skipped (Could not find original raw data folder in {args.root_dir})")
                continue

            raw_session_path = raw_session_folders[0]

            # Look for the Surface csv folder inside the raw session path
            surf_folder_pattern = os.path.join(raw_session_path, "*_Surface_csv")
            surf_folders = glob.glob(surf_folder_pattern)

            if not surf_folders:
                print("   -> Skipped (No Surface CSV folder found in raw directory)")
                continue

            surf_folder = surf_folders[0]
            surf_file = os.path.join(surf_folder, "gaze.csv")
            if not os.path.exists(surf_file):
                # Try finding just 'gaze' with no extension
                surf_file = os.path.join(surf_folder, "gaze")

            if not os.path.exists(surf_file):
                print("   -> Skipped (No 'gaze.csv' file found)")
                continue

            try:
                surf_df = pd.read_csv(surf_file)
                surf_df = standardize_columns(surf_df)
            except Exception as e:
                print(f"   -> Error reading CSV: {e}")
                continue

            # --- 3. Extract Data for Each Trial ---
            points_found = 0
            for trial_num in trial_list:
                # Look inside the Analysis Output folder we found in step 1
                trial_pattern = os.path.join(analysis_path, f"*_Trial_{trial_num}_Analysis")
                trial_folders = glob.glob(trial_pattern)

                if not trial_folders:
                    continue

                trial_data = get_surface_data_for_trial(trial_folders[0], surf_df)

                if trial_data is not None and not trial_data.empty:
                    collected_data.append(trial_data)
                    points_found += len(trial_data)

            print(f"   -> Collected {points_found} points.")

    if not collected_data:
        print("\nError: No valid data found for the specified criteria.")
        return

    print(f"\nMerging {len(collected_data)} datasets...")
    master_df = pd.concat(collected_data, ignore_index=True)

    # Save output to the analysis directory so it's with the plots
    out_file = os.path.join(args.analysis_dir, args.output_name)
    plot_title = os.path.splitext(args.output_name)[0]

    if len(trial_list) < 10:
        trial_display = str(trial_list)
    else:
        trial_display = f"{start_t}-{end_t} (Excl: {args.exclude_trials})"

    subtitle = f"Participants: {args.ids} | Session: {args.sessions} | Trials: {trial_display}"

    generate_heatmap(master_df, plot_title, subtitle, out_file)


if __name__ == "__main__":
    main()