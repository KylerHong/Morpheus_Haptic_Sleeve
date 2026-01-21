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
    Pupil Labs exports can sometimes vary in naming (e.g., 'norm_pos_x' vs 'gaze_normal0_x'),
    so this ensures consistent access regardless of the export version.
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
    # Sanity check: Ensure we have the data columns we need
    if 'x_norm' not in df.columns or 'y_norm' not in df.columns:
        print(f"Error: Missing coordinate columns. Found: {list(df.columns)}")
        return

    if df.empty:
        print("Error: Dataset is empty. Cannot generate heatmap.")
        return

    # Set up the figure size
    plt.figure(figsize=(10, 8))

    # Filter out any invalid coordinates (outside the 0-1 normalized range)
    # This removes noise or tracking errors that fall outside the screen.
    df = df[(df['x_norm'] >= 0) & (df['x_norm'] <= 1) &
            (df['y_norm'] >= 0) & (df['y_norm'] <= 1)]

    # --- Optimization Step ---
    # Plotting hundreds of thousands of points is slow and visually unnecessary.
    # If the dataset is huge, we downsample it to ~30k points to keep runtimes fast (seconds vs minutes).
    MAX_POINTS = 30000
    if len(df) > MAX_POINTS:
        stride = len(df) // MAX_POINTS
        print(f"Dataset too large ({len(df)} points). Downsampling to ~{MAX_POINTS} for performance...")
        df = df.iloc[::stride]

    # Double check we still have data after filtering
    if len(df) < 10:
        print("Not enough valid data points to plot.")
        plt.close()
        return

    # Flip Y axis because standard graphs have (0,0) at bottom-left,
    # but screen coordinates usually have (0,0) at top-left.
    x = df['x_norm']
    y = 1 - df['y_norm']

    try:
        # Define the custom color map (Dark Purple -> Red)
        # This creates the "weather map" style gradient.
        colors = ['#4b0082', '#0000ff', '#00ffff', '#00ff00', '#ffff00', '#ff8c00', '#ff0000']
        weather_cmap = mcolors.LinearSegmentedColormap.from_list("weather_map", colors, N=256)

        # Plot the KDE (Kernel Density Estimate)
        # levels=60 makes the gradient look smooth rather than banded.
        sns.kdeplot(x=x, y=y, fill=True, cmap=weather_cmap, alpha=0.8, levels=60, thresh=0.05)

    except Exception as e:
        print(f"Plotting Error: {e}")
        plt.close()
        return

    # Formatting: Set limits to the normalized screen space (0 to 1)
    plt.xlim(0, 1)
    plt.ylim(0, 1)

    # Add Titles
    plt.suptitle(title, fontsize=18, y=0.98, fontweight='bold')
    plt.title(subtitle_info, fontsize=10)

    # Remove axis ticks for a cleaner look
    plt.axis('off')

    # Draw the screen border
    plt.gca().add_patch(plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor='black', lw=3))

    # Draw quadrant lines (center crosshair)
    plt.axvline(x=0.5, color='black', linestyle='-', linewidth=2)
    plt.axhline(y=0.5, color='black', linestyle='-', linewidth=2)

    # Adjust layout so the title doesn't overlap the graph
    plt.tight_layout(rect=[0, 0, 1, 1])

    # Save to disk
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\nSuccess! Heatmap saved to: {output_path}")


def get_surface_data_for_trial(trial_folder, surface_df):
    """
    Extracts the specific chunk of surface data that corresponds to a single trial.
    It works by finding the start/end timestamps of the trial analysis file
    and slicing the master surface file to match those times.
    """
    # Find the 'raw_gaze_slice' file to get exact timestamps
    gaze_files = glob.glob(os.path.join(trial_folder, "*_raw_gaze_slice.csv"))
    if not gaze_files:
        return None

    try:
        gaze_slice = pd.read_csv(gaze_files[0])
        if gaze_slice.empty:
            return None

        # Get start and end times
        start_ns = gaze_slice['timestamp [ns]'].min()
        end_ns = gaze_slice['timestamp [ns]'].max()

        # Handle potential column name differences for timestamps
        t_col = 'timestamp [ns]'
        if t_col not in surface_df.columns and 'gaze_timestamp' in surface_df.columns:
            t_col = 'gaze_timestamp'

        # Slice the data
        mask = (surface_df[t_col] >= start_ns) & (surface_df[t_col] <= end_ns)
        return surface_df[mask].copy()
    except:
        return None


def main():
    # Setup command line arguments
    parser = argparse.ArgumentParser(description="Generate a custom aggregate heatmap.")
    parser.add_argument("--root-dir", required=True, help="Path to the main data folder (e.g., BEAR Lab)")
    parser.add_argument("--ids", nargs='+', required=True, help="List of participant IDs (e.g., 01 03 04)")
    parser.add_argument("--sessions", nargs='+', required=True, help="List of sessions (e.g., 1 2)")

    parser.add_argument("--trial-range", nargs=2, type=int, required=True,
                        help="Start and End trial numbers (e.g., 30 38)")

    parser.add_argument("--exclude-trials", nargs=2, type=int, default=None,
                        help="Optional: Start and End trial numbers to EXCLUDE (e.g., 32 36)")

    parser.add_argument("--output-name", default="Custom_Heatmap.png", help="Filename for the output image")
    args = parser.parse_args()

    # Create the full list of trials to process
    start_t, end_t = args.trial_range
    trial_list = list(range(start_t, end_t + 1))

    # Apply exclusions if requested
    if args.exclude_trials:
        ex_start, ex_end = args.exclude_trials
        exclude_set = set(range(ex_start, ex_end + 1))
        trial_list = [t for t in trial_list if t not in exclude_set]
        exclusion_str = f"(Excluded: {ex_start}-{ex_end})"
    else:
        exclusion_str = ""

    print("-" * 40)
    print("Custom Heatmap Generator")
    print(f"Participants: {args.ids}")
    print(f"Sessions:     {args.sessions}")
    print(f"Trials:       {trial_list} {exclusion_str}")
    print("-" * 40)

    if not trial_list:
        print("Error: No trials selected. Check your ranges.")
        return

    collected_data = []

    # Main Loop: Iterate through every requested Participant and Session
    for pid in args.ids:
        for sess in args.sessions:
            # Find the session folder (handles variations like SHF01 vs SNHF01)
            search_pattern = os.path.join(args.root_dir, f"*{pid}_Session{sess}*")
            matching_folders = glob.glob(search_pattern)

            if not matching_folders:
                print(f"Warning: Could not find folder for Participant {pid}, Session {sess}")
                continue

            session_path = matching_folders[0]
            session_name = os.path.basename(session_path)
            print(f"Processing: {session_name}...")

            # --- Locate the Surface Data ---
            # This is usually in a separate folder ending in '_Surface_csv'
            surf_folder_pattern = os.path.join(session_path, "*_Surface_csv")
            surf_folders = glob.glob(surf_folder_pattern)

            if not surf_folders:
                print("   -> Skipped (No Surface CSV folder found)")
                continue

            surf_folder = surf_folders[0]
            # The file is typically named 'gaze' or 'gaze.csv'
            surf_file = os.path.join(surf_folder, "gaze")
            if not os.path.exists(surf_file):
                surf_file = os.path.join(surf_folder, "gaze.csv")

            if not os.path.exists(surf_file):
                print("   -> Skipped (No 'gaze' file found)")
                continue

            try:
                # Load the big surface file
                surf_df = pd.read_csv(surf_file)
                surf_df = standardize_columns(surf_df)
            except Exception as e:
                print(f"   -> Error reading CSV: {e}")
                continue

            # --- Locate the Analysis Output Folder ---
            # We check both naming conventions: "Timeseries Data + Scene Video" and just "Timeseries Data"
            ts_folder = os.path.join(session_path, "Timeseries Data + Scene Video")
            if not os.path.exists(ts_folder):
                ts_folder = os.path.join(session_path, "Timeseries Data")

            if not os.path.exists(ts_folder):
                print("   -> Skipped (No Timeseries Data folder found)")
                continue

            # Find the analysis subfolder
            analysis_pattern = os.path.join(ts_folder, "*_Analysis_Output")
            analysis_folders = glob.glob(analysis_pattern)

            if not analysis_folders:
                print("   -> Skipped (No Analysis Output found)")
                continue

            analysis_path = analysis_folders[0]

            # --- Extract Data for Each Trial ---
            points_found = 0
            for trial_num in trial_list:
                trial_pattern = os.path.join(analysis_path, f"*_Trial_{trial_num}_Analysis")
                trial_folders = glob.glob(trial_pattern)

                if not trial_folders:
                    continue

                # Get the slice of data for this specific trial
                trial_data = get_surface_data_for_trial(trial_folders[0], surf_df)

                if trial_data is not None and not trial_data.empty:
                    collected_data.append(trial_data)
                    points_found += len(trial_data)

            print(f"   -> Collected {points_found} points.")

    # Generate the final plot if we found data
    if not collected_data:
        print("\nError: No valid data found for the specified criteria.")
        return

    print(f"\nMerging {len(collected_data)} datasets...")
    master_df = pd.concat(collected_data, ignore_index=True)

    out_file = os.path.join(args.root_dir, args.output_name)
    plot_title = os.path.splitext(args.output_name)[0]

    # Format subtitle
    if len(trial_list) < 10:
        trial_display = str(trial_list)
    else:
        trial_display = f"{start_t}-{end_t} (Excl: {args.exclude_trials})"

    subtitle = f"Participants: {args.ids} | Session: {args.sessions} | Trials: {trial_display}"

    generate_heatmap(master_df, plot_title, subtitle, out_file)


if __name__ == "__main__":
    main()