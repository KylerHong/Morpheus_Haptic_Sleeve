import pandas as pd
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import numpy as np
import glob
import traceback


def get_quadrant(x, y):
    """
    Determines which quadrant of the screen the user is looking at based on normalized X/Y coordinates.

    Args:
        x (float): Normalized X coordinate (0.0 to 1.0).
        y (float): Normalized Y coordinate (0.0 to 1.0).

    Returns:
        str: 'Top-Left', 'Top-Right', 'Bottom-Left', 'Bottom-Right', or None if off-screen.
    """
    try:
        x_val = float(x)
        y_val = float(y)
    except (ValueError, TypeError):
        return None

    # Check if gaze is within valid screen bounds
    if not (0.0 <= x_val <= 1.0 and 0.0 <= y_val <= 1.0):
        return None

    # Determine specific quadrant
    if x_val < 0.5:
        if y_val < 0.5:
            return "Top-Left"
        else:
            return "Bottom-Left"
    else:
        if y_val < 0.6:
            return "Top-Right"
        else:
            return "Bottom-Right"


def find_lsl_file_for_trial(lsl_dir, trial_num):
    """
    Locates the specific LSL synchronization CSV file for a given trial number.

    Args:
        lsl_dir (str): Directory containing LSL files.
        trial_num (int): The trial number to look for.

    Returns:
        str: Path to the matching LSL file, or None if not found.
    """
    try:
        clean_trial_num = int(trial_num)
        pattern = os.path.join(lsl_dir, f"*_{clean_trial_num:03d}_lsl.csv")
        matches = glob.glob(pattern)

        if not matches:
            return None
        return matches[0]
    except (ValueError, TypeError):
        return None


def generate_summary_stats(merged_df, eye_df, saccades_df, imu_df, output_dir, trial_name):
    """
    Calculates statistical averages and peaks for key metrics (pupil, aperture, head velocity)
    and saves them to a single-row CSV summary file.

    Args:
        merged_df (pd.DataFrame): Dataframe containing fixation events.
        eye_df (pd.DataFrame): Raw eye data (aperture/pupil).
        saccades_df (pd.DataFrame): Saccade events.
        imu_df (pd.DataFrame): Head movement data.
        output_dir (str): Folder to save the output CSV.
        trial_name (str): Identifier for the file name.

    Returns:
        pd.DataFrame: A single-row dataframe containing the calculated stats.
    """
    stats = {}
    try:
        # Eye Metrics
        if eye_df is not None and not eye_df.empty:
            stats['avg_eyelid_aperture_mm'] = float(eye_df['eyelid_aperture_avg'].mean())
            stats['peak_eyelid_aperture_mm'] = float(eye_df['eyelid_aperture_avg'].max())
            stats['avg_pupil_diameter_mm'] = float(eye_df['pupil_diameter_avg'].mean())
            stats['peak_pupil_diameter_mm'] = float(eye_df['pupil_diameter_avg'].max())

        # Fixation Metrics
        if merged_df is not None and not merged_df.empty:
            stats['avg_fixation_duration_s'] = float(merged_df['duration_sec'].mean())
            stats['peak_fixation_duration_s'] = float(merged_df['duration_sec'].max())

        # Saccade Metrics
        if saccades_df is not None and not saccades_df.empty and 'peak velocity [px/s]' in saccades_df.columns:
            stats['avg_saccadic_velocity_px_s'] = float(saccades_df['peak velocity [px/s]'].mean())
            stats['peak_saccadic_velocity_px_s'] = float(saccades_df['peak velocity [px/s]'].max())

        # Head Movement Metrics
        if imu_df is not None and not imu_df.empty and 'head_angular_velocity' in imu_df.columns:
            stats['avg_head_velocity_deg_s'] = float(imu_df['head_angular_velocity'].mean())
            stats['peak_head_velocity_deg_s'] = float(imu_df['head_angular_velocity'].max())

        if not stats:
            return None

        # Save to CSV
        summary_df = pd.DataFrame([stats])
        summary_filepath = os.path.join(output_dir, f"{trial_name}_key_metrics.csv")
        summary_df.to_csv(summary_filepath, index=False)
        return summary_df

    except Exception as e:
        print(f"   [ERROR] Failed to generate summary stats: {e}")
        return None


def analyze_trial(
        trial_name: str,
        main_df: pd.DataFrame,
        surface_df: pd.DataFrame,
        eye_df: pd.DataFrame,
        blinks_df: pd.DataFrame,
        saccades_df: pd.DataFrame,
        imu_df: pd.DataFrame,
        trial_keys: pd.DataFrame,
        output_dir: str
):
    """
    Core analysis function for a single trial (or baseline period).

    1. Pre-processes all data streams to start at t=0.0s.
    2. Merges distinct sensor streams (Eye, IMU, Keys) into one continuous timeseries CSV.
    3. Generates a visual timeline plot of attention and physiological metrics.
    4. Computes summary statistics.
    """

    if main_df.empty:
        print("   [CRITICAL WARNING] Trial Slice is EMPTY (No fixations found).")
        return None

    try:

        # Determine "Time Zero": Use the very first eye frame in the slice as t=0.0
        start_time_ns = 0.0
        if eye_df is not None and not eye_df.empty:
            start_time_ns = float(eye_df['timestamp [ns]'].min())
        elif not main_df.empty:
            start_time_ns = float(main_df['start timestamp [ns]'].min())

        # Process Eye Data (Calculate Left/Right Averages)
        if eye_df is not None and not eye_df.empty:
            eye_df = eye_df.copy()
            eye_df['time_sec'] = (eye_df['timestamp [ns]'] - start_time_ns) * 1e-9
            eye_df['pupil_diameter_avg'] = eye_df[['pupil diameter left [mm]', 'pupil diameter right [mm]']].mean(
                axis=1)
            eye_df['eyelid_aperture_avg'] = eye_df[['eyelid aperture left [mm]', 'eyelid aperture right [mm]']].mean(
                axis=1)

        # Process IMU Data (Calculate Angular Velocity Magnitude)
        head_velocity_available = False
        if imu_df is not None and not imu_df.empty:
            imu_df = imu_df.copy()
            imu_df['time_sec'] = (imu_df['timestamp [ns]'] - start_time_ns) * 1e-9
            gyro_cols = ['gyro x [deg/s]', 'gyro y [deg/s]', 'gyro z [deg/s]']
            if all(col in imu_df.columns for col in gyro_cols):
                imu_df['head_angular_velocity'] = np.linalg.norm(imu_df[gyro_cols].values, axis=1)
                head_velocity_available = True

        # Process Keystroke Data
        keys_available = False
        if trial_keys is not None and not trial_keys.empty:
            trial_keys = trial_keys.copy()
            trial_keys['time_sec'] = (trial_keys['timestamp [ns]'] - start_time_ns) * 1e-9
            # Normalize column names
            if 'event' in trial_keys.columns:
                trial_keys = trial_keys[['time_sec', 'event']].rename(columns={'event': 'button_pressed'})
            elif 'key' in trial_keys.columns:
                trial_keys = trial_keys[['time_sec', 'key']].rename(columns={'key': 'button_pressed'})
            keys_available = True

        # Process Fixations & Map Quadrants
        surface_df = surface_df.copy()
        main_df = main_df.copy()

        surface_df['quadrant'] = surface_df.apply(
            lambda r: get_quadrant(r['fixation x [normalized]'], r['fixation y [normalized]']), axis=1
        )

        # Merge quadrant info onto main fixation data
        surface_quadrants = surface_df[['fixation id', 'quadrant']]
        merged_df = pd.merge(main_df, surface_quadrants, on='fixation id', how='left')
        merged_df['quadrant'] = merged_df['quadrant'].fillna('Off-Surface')
        merged_df.dropna(subset=['quadrant'], inplace=True)

        if not merged_df.empty:
            merged_df['start_time_sec'] = (merged_df['start timestamp [ns]'] - start_time_ns) * 1e-9
            merged_df['end_time_sec'] = (merged_df['end timestamp [ns]'] - start_time_ns) * 1e-9
            merged_df['duration_sec'] = merged_df['duration [ms]'] * 1e-3


        # Start with Eye Data as the "Master Clock" (Highest Frequency ~200Hz)
        timeseries_df = eye_df[['time_sec', 'pupil_diameter_avg', 'eyelid_aperture_avg']].copy()
        timeseries_df.rename(
            columns={'pupil_diameter_avg': 'pupil_diameter_mm', 'eyelid_aperture_avg': 'eyelid_aperture_mm'},
            inplace=True
        )

        # Merge IMU Data (Backward Fill)
        # We align IMU data to Eye timestamps by taking the most recent previous IMU value.
        if head_velocity_available:
            imu_stream = imu_df[['time_sec', 'head_angular_velocity']].copy()
            imu_stream.sort_values('time_sec', inplace=True)
            timeseries_df.sort_values('time_sec', inplace=True)

            timeseries_df = pd.merge_asof(timeseries_df, imu_stream, on='time_sec', direction='backward')
            timeseries_df.rename(columns={'head_angular_velocity': 'head_angular_velocity_deg_s'}, inplace=True)

        # Merge Keystrokes (Nearest Neighbor with Tolerance)
        # We only assign a button press if it happened within 50ms of the eye frame.
        if keys_available:
            trial_keys.sort_values('time_sec', inplace=True)
            timeseries_df.sort_values('time_sec', inplace=True)

            timeseries_df = pd.merge_asof(
                timeseries_df,
                trial_keys,
                on='time_sec',
                direction='nearest',
                tolerance=0.05
            )

        # Map Fixation IDs to Timestamps
        # Vectorized lookup to check which fixation interval each timestamp falls into.
        if not merged_df.empty:
            intervals = pd.IntervalIndex.from_arrays(merged_df['start_time_sec'], merged_df['end_time_sec'],
                                                     closed='both')
            try:
                indices = intervals.get_indexer(timeseries_df['time_sec'].values)

                # Retrieve source data arrays
                fix_ids = merged_df['fixation id'].values
                quadrants = merged_df['quadrant'].values

                # Initialize result arrays
                mapped_ids = np.full(len(timeseries_df), np.nan)
                mapped_quads = np.full(len(timeseries_df), None, dtype=object)

                # Fill valid matches
                valid_mask = indices != -1
                valid_indices = indices[valid_mask]
                mapped_ids[valid_mask] = fix_ids[valid_indices]
                mapped_quads[valid_mask] = quadrants[valid_indices]

                timeseries_df['current_fixation_id'] = mapped_ids
                timeseries_df['current_quadrant'] = mapped_quads
            except Exception as e:
                print(f"   [WARN] Fixation mapping failed: {e}")

        # Save Clean CSV (Ordered as requested)
        desired_order = [
            'time_sec',
            'eyelid_aperture_mm',
            'pupil_diameter_mm',
            'head_angular_velocity_deg_s',
            'button_pressed',
            'current_fixation_id',
            'current_quadrant'
        ]
        final_cols = [c for c in desired_order if c in timeseries_df.columns]

        raw_csv_path = os.path.join(output_dir, f"{trial_name}_full_timeseries.csv")
        timeseries_df[final_cols].to_csv(raw_csv_path, index=False)

        # Determine necessary subplots
        plots_to_create = {
            'aperture': eye_df is not None and not eye_df.empty,
            'pupil': eye_df is not None and not eye_df.empty,
            'fix_duration': not merged_df.empty,
            'saccades': saccades_df is not None and not saccades_df.empty,
            'blinks': blinks_df is not None and not blinks_df.empty,
            'head_velocity': head_velocity_available
        }
        num_plots = 1 + sum(plots_to_create.values())

        fig, axs = plt.subplots(num_plots, 1, figsize=(15, 2.0 * num_plots), sharex=True)
        if num_plots == 1: axs = [axs]
        fig.suptitle(f"Cognitive Load Analysis - {trial_name}", fontsize=16)

        # Plot Config (Colors/Positions)
        categories = {
            "Top-Right": {'y': 4, 'color': '#1f77b4'}, "Top-Left": {'y': 3, 'color': '#ff7f0e'},
            "Bottom-Right": {'y': 2, 'color': '#2ca02c'}, "Bottom-Left": {'y': 1, 'color': '#d62728'},
            "Off-Surface": {'y': 0, 'color': '#7f7f7f'}
        }

        # Subplot 0: Visual Attention Timeline
        if not merged_df.empty:
            merged_df['y_pos'] = merged_df['quadrant'].apply(lambda q: categories.get(q, {}).get('y'))
            merged_df['color'] = merged_df['quadrant'].apply(lambda q: categories.get(q, {}).get('color'))
            axs[0].barh(y=merged_df['y_pos'], width=merged_df['duration_sec'], left=merged_df['start_time_sec'],
                        color=merged_df['color'], height=0.7)

        axs[0].set_yticks(ticks=[cat['y'] for cat in categories.values()])
        axs[0].set_yticklabels(labels=categories.keys())
        axs[0].set_title("Timeline of Visual Attention")
        axs[0].grid(axis='x', linestyle='--', alpha=0.6)

        # Add Legend
        legend_patches = [mpatches.Patch(color=cat['color'], label=name) for name, cat in categories.items()]
        axs[0].legend(handles=legend_patches, bbox_to_anchor=(1.02, 1.02), loc='upper left')

        # Add colored background spans to all other plots
        plot_idx = 1
        if not merged_df.empty:
            for row in merged_df.itertuples():
                if pd.notna(row.color):
                    t_start, t_dur = float(row.start_time_sec), float(row.duration_sec)
                    for i in range(1, num_plots):
                        axs[i].axvspan(xmin=t_start, xmax=t_start + t_dur, color=row.color, alpha=0.15, zorder=0)

        # Draw Metric Plots
        if plots_to_create['aperture']:
            axs[plot_idx].plot(eye_df['time_sec'], eye_df['eyelid_aperture_avg'], color='purple', linewidth=1.5)
            axs[plot_idx].set_ylabel("Aperture (mm)")
            plot_idx += 1
        if plots_to_create['pupil']:
            axs[plot_idx].plot(eye_df['time_sec'], eye_df['pupil_diameter_avg'], color='teal', linewidth=1.5)
            axs[plot_idx].set_ylabel("Diameter (mm)")
            plot_idx += 1
        if plots_to_create['fix_duration']:
            axs[plot_idx].stem(merged_df['start_time_sec'], merged_df['duration_sec'], linefmt='grey', markerfmt='o',
                               basefmt=" ")
            axs[plot_idx].set_ylabel("Duration (s)")
            plot_idx += 1
        if plots_to_create['saccades']:
            saccades_df = saccades_df.copy()
            saccades_df['start_time_sec'] = (saccades_df['start timestamp [ns]'] - start_time_ns) * 1e-9
            axs[plot_idx].plot(saccades_df['start_time_sec'], saccades_df['peak velocity [px/s]'], color='darkorange',
                               marker='.', linestyle='')
            axs[plot_idx].set_ylabel("Velocity (px/s)")
            plot_idx += 1
        if plots_to_create['head_velocity']:
            axs[plot_idx].plot(imu_df['time_sec'], imu_df['head_angular_velocity'], color='black', linewidth=1.5)
            axs[plot_idx].set_ylabel("Velocity (deg/s)")
            plot_idx += 1
        if plots_to_create['blinks']:
            for _, blink_row in blinks_df.iterrows():
                t_b_start = (blink_row['start timestamp [ns]'] - start_time_ns) * 1e-9
                t_b_end = (blink_row['end timestamp [ns]'] - start_time_ns) * 1e-9
                axs[plot_idx].axvspan(xmin=t_b_start, xmax=t_b_end, color='crimson', alpha=0.5, label='Blink')
            axs[plot_idx].set_yticks([])
            axs[plot_idx].set_ylabel("Blinks")

        axs[-1].set_xlabel("Time (seconds)")
        fig.tight_layout(rect=(0, 0, 1, 0.98))

        timeline_plot_path = os.path.join(output_dir, f"{trial_name}_timeline_plot.png")
        fig.savefig(timeline_plot_path)
        plt.close(fig)

        return generate_summary_stats(merged_df, eye_df, saccades_df, imu_df, output_dir, trial_name)

    except Exception as e:
        print(f"   [ERROR] Crash during analysis of {trial_name}: {e}")
        traceback.print_exc()
        return None


def generate_trend_plots(df, output_dir, session_label=""):
    """
    Generates a 2x5 grid of trend plots showing how metrics change across trial numbers.
    This visualizes the "Learning Curve" of the user.
    """
    try:
        metric_config = {
            'pupil_diameter': {'color': 'teal', 'unit': 'mm'},
            'eyelid_aperture': {'color': 'purple', 'unit': 'mm'},
            'fixation_duration': {'color': 'grey', 'unit': 's'},
            'saccadic_velocity': {'color': 'darkorange', 'unit': 'px/s'},
            'head_velocity': {'color': 'black', 'unit': 'deg/s'}
        }

        # Setup Grid
        base_names = list(metric_config.keys())
        nrows = 2
        ncols = 5
        fig, axs = plt.subplots(nrows, ncols, figsize=(22, 8), sharex=True)
        title_text = f"Metrics Across Trials (Learning Curve) - {session_label}" if session_label else "Metrics Across Trials"
        fig.suptitle(title_text, fontsize=16)

        # Loop through metrics and plot Peak (Row 0) and Average (Row 1)
        for col_idx, base_name in enumerate(base_names):
            config = metric_config[base_name]
            color = config['color']
            unit = config['unit']
            peak_metric = f"peak_{base_name}_{unit.replace('/', '_')}"
            avg_metric = f"avg_{base_name}_{unit.replace('/', '_')}"

            # Plot Peak Trend
            ax_peak = axs[0, col_idx]
            if peak_metric in df.columns:
                x_data = df['trial_num']
                y_data = df[peak_metric].fillna(df[peak_metric].mean())
                ax_peak.plot(x_data, y_data, marker='o', linestyle='--', color=color)
                # Trendline
                if len(x_data) > 1:
                    m, b = np.polyfit(x_data, y_data, 1)
                    ax_peak.plot(x_data, m * x_data + b, color=color, linestyle=':', alpha=0.7)
                ax_peak.set_title(f"Peak {base_name.replace('_', ' ').title()}")
                ax_peak.set_ylabel(unit)
                ax_peak.grid(True, linestyle='--')
            else:
                ax_peak.axis('off')

            # Plot Average Trend
            ax_avg = axs[1, col_idx]
            if avg_metric in df.columns:
                x_data = df['trial_num']
                y_data = df[avg_metric].fillna(df[avg_metric].mean())
                ax_avg.plot(x_data, y_data, marker='o', linestyle='-', color=color)
                # Trendline
                if len(x_data) > 1:
                    m, b = np.polyfit(x_data, y_data, 1)
                    ax_avg.plot(x_data, m * x_data + b, color=color, linestyle=':', alpha=0.7)
                ax_avg.set_title(f"Average {base_name.replace('_', ' ').title()}")
                ax_avg.set_ylabel(unit)
                ax_avg.grid(True, linestyle='--')
                ax_avg.set_xlabel("Trial Number")
            else:
                ax_avg.axis('off')

        # Clean up empty columns
        for col_idx in range(len(base_names), ncols):
            axs[0, col_idx].axis('off')
            axs[1, col_idx].axis('off')

        fig.tight_layout(rect=(0, 0, 1, 0.95))
        plot_path = os.path.join(output_dir,
                                 f"{session_label}_trend_plots.png" if session_label else "all_trials_trend_plots.png")
        fig.savefig(plot_path)
        print(f"Trend Plots saved to: {plot_path}")
        plt.close(fig)
    except Exception as e:
        print(f"[ERROR] Failed to generate trend plots: {e}")


def main():
    parser = argparse.ArgumentParser(description="Process Pupil Labs Eye Tracking data synced with ROS/LSL.")
    parser.add_argument("--main-dir", required=True, help="Path to main Pupil Export (folder containing fixations.csv)")
    parser.add_argument("--surface-dir", required=True, help="Path to surface folder")
    parser.add_argument("--keystroke-file", required=True, help="Path to Master Keystrokes CSV (Ground Truth)")
    parser.add_argument("--lsl-dir", required=True, help="Path to FOLDER containing all LSL files")
    parser.add_argument("--exclude", nargs='+', type=int, default=[], help="Trial numbers to exclude from report")
    parser.add_argument("--session-id", type=str, default="", help="Optional identifier string for file naming")
    args = parser.parse_args()

    try:
        # Define paths
        fixations_file = os.path.join(args.main_dir, "fixations.csv")
        surface_fixations_file = os.path.join(args.surface_dir, "fixations.csv")
        eye_states_file = os.path.join(args.main_dir, "3d_eye_states.csv")
        blinks_file = os.path.join(args.main_dir, "blinks.csv")
        saccades_file = os.path.join(args.main_dir, "saccades.csv")
        imu_file = os.path.join(args.main_dir, "imu.csv")

        # Load Dataframes
        all_main_df = pd.read_csv(fixations_file)
        all_surface_df = pd.read_csv(surface_fixations_file)
        all_eye_df = pd.read_csv(eye_states_file)
        # Load optional files safely
        all_blinks_df = pd.read_csv(blinks_file) if os.path.exists(blinks_file) else None
        all_saccades_df = pd.read_csv(saccades_file) if os.path.exists(saccades_file) else None
        all_imu_df = pd.read_csv(imu_file) if os.path.exists(imu_file) else None

        # Load Master Keystrokes
        keystrokes_df = pd.read_csv(args.keystroke_file)

    except FileNotFoundError as e:
        sys.exit(f"[FATAL ERROR] Missing file: {e.filename}")
    except Exception as e:
        sys.exit(f"[FATAL ERROR] Error reading file: {e}")

    root_dir = os.path.dirname(args.main_dir)
    folder_name = f"{args.session_id}_Analysis_Output" if args.session_id else "Analysis_Output"
    master_output_dir = os.path.join(root_dir, folder_name)
    os.makedirs(master_output_dir, exist_ok=True)
    print(f"Output will be saved to: {master_output_dir}")

    # Helper function for time slicing
    def slice_df(df, start, end, time_col):
        if df is None: return None
        return df[(df[time_col] >= start) & (df[time_col] <= end)].copy()

    start_events = keystrokes_df[keystrokes_df['event'] == 'start_recording'].copy()
    all_trial_summaries = []

    try:
        trial0_row = start_events[start_events['trial'] == 0]

        if not trial0_row.empty:
            # Use Trial 0's start time as the END of the baseline period.
            trial0_ros_time = float(trial0_row.iloc[0]['ros_time'])

            lsl_file_path = find_lsl_file_for_trial(args.lsl_dir, 0)
            if lsl_file_path:
                lsl_df = pd.read_csv(lsl_file_path)
                # Find closest LSL row to Trial 0 Start for sync offset
                time_diffs = (lsl_df['ros_time'] - trial0_ros_time).abs()
                lsl_sync_row = lsl_df.iloc[int(time_diffs.idxmin())]

                # Offset = Glasses_Time - Robot_Time
                offset_ns = lsl_sync_row['sys_utc_ns'].item() - (lsl_sync_row['ros_time'].item() * 1e9)

                # Baseline End = Start of Trial 0 (in Glasses Time)
                baseline_end_ns = int((trial0_ros_time * 1e9) + offset_ns)
                # Baseline Start = Very first recorded timestamp in eye data
                baseline_start_ns = int(all_eye_df['timestamp [ns]'].min().item())

                # Create Directory
                baseline_name = f"{args.session_id}_Baseline" if args.session_id else "Baseline"
                baseline_output_dir = os.path.join(master_output_dir, f"{baseline_name}_Analysis")
                os.makedirs(baseline_output_dir, exist_ok=True)

                # Slice Data
                base_main_df = slice_df(all_main_df, baseline_start_ns, baseline_end_ns, 'start timestamp [ns]')
                base_surface_df = slice_df(all_surface_df, baseline_start_ns, baseline_end_ns, 'start timestamp [ns]')
                base_eye_df = slice_df(all_eye_df, baseline_start_ns, baseline_end_ns, 'timestamp [ns]')
                base_blinks_df = slice_df(all_blinks_df, baseline_start_ns, baseline_end_ns, 'start timestamp [ns]')
                base_saccades_df = slice_df(all_saccades_df, baseline_start_ns, baseline_end_ns, 'start timestamp [ns]')
                base_imu_df = slice_df(all_imu_df, baseline_start_ns, baseline_end_ns, 'timestamp [ns]')

                # Keys during baseline
                base_keys = keystrokes_df.copy()
                base_keys['timestamp [ns]'] = (base_keys['ros_time'] * 1e9) + offset_ns
                base_keys = slice_df(base_keys, baseline_start_ns, baseline_end_ns, 'timestamp [ns]')

                # Analyze
                if not base_eye_df.empty:
                    analyze_trial(
                        baseline_name, base_main_df, base_surface_df, base_eye_df,
                        base_blinks_df, base_saccades_df, base_imu_df, base_keys,
                        baseline_output_dir
                    )
                    print("Baseline Analysis Complete.")
                else:
                    print("[ERROR] No eye data found for baseline period.")
            else:
                print("[ERROR] Missing LSL file for Trial 0. Cannot sync baseline.")
        else:
            print("[ERROR] Trial 0 start event not found.")

    except Exception as e:
        print(f"[ERROR] Failed to process baseline: {e}")
        traceback.print_exc()

    print(f"Found {len(start_events)} trials.")

    for start_row in start_events.itertuples(index=False):
        trial_name = "Unknown_Trial"
        try:
            # Identification
            trial_num = int(start_row.trial)
            base_trial_name = f"Trial_{trial_num}"
            trial_name = f"{args.session_id}_{base_trial_name}" if args.session_id else base_trial_name

            ros_start_time = float(start_row.ros_time)
            print(f"Processing {trial_name}")

            # Synchronization
            lsl_file_path = find_lsl_file_for_trial(args.lsl_dir, trial_num)
            if not lsl_file_path:
                print(f"[ERROR] No LSL file found for Trial {trial_num}. Skipping.")
                continue

            lsl_df = pd.read_csv(lsl_file_path)
            time_diffs = (lsl_df['ros_time'] - ros_start_time).abs()
            lsl_sync_row = lsl_df.iloc[int(time_diffs.idxmin())]

            offset_ns = lsl_sync_row['sys_utc_ns'].item() - (lsl_sync_row['ros_time'].item() * 1e9)

            # Boundaries
            start_ns = int((ros_start_time * 1e9) + offset_ns)

            stop_rows = keystrokes_df[
                (keystrokes_df['event'] == 'stop_recording') &
                (keystrokes_df['trial'] == trial_num)
                ]

            if not stop_rows.empty:
                ros_end_time = float(stop_rows['ros_time'].values[0])
                end_ns = int((ros_end_time * 1e9) + offset_ns)
            else:
                # Default to 60s if stop event missing
                end_ns = start_ns + int(60 * 1e9)

            # Keys for this trial
            trial_keys = keystrokes_df[keystrokes_df['trial'] == trial_num].copy()
            trial_keys['timestamp [ns]'] = (trial_keys['ros_time'] * 1e9) + offset_ns

            # Slicing
            trial_output_dir = os.path.join(master_output_dir, f"{trial_name}_Analysis")
            os.makedirs(trial_output_dir, exist_ok=True)

            trial_main_df = slice_df(all_main_df, start_ns, end_ns, 'start timestamp [ns]')
            if trial_main_df.empty:
                print(f"[ERROR] Slice returned 0 rows! Check Sync.")
                continue

            trial_surface_df = slice_df(all_surface_df, start_ns, end_ns, 'start timestamp [ns]')
            trial_eye_df = slice_df(all_eye_df, start_ns, end_ns, 'timestamp [ns]')
            trial_blinks_df = slice_df(all_blinks_df, start_ns, end_ns, 'start timestamp [ns]')
            trial_saccades_df = slice_df(all_saccades_df, start_ns, end_ns, 'start timestamp [ns]')
            trial_imu_df = slice_df(all_imu_df, start_ns, end_ns, 'timestamp [ns]')

            # Analysis
            summary_data = analyze_trial(
                trial_name, trial_main_df, trial_surface_df, trial_eye_df,
                trial_blinks_df, trial_saccades_df, trial_imu_df,
                trial_keys,
                trial_output_dir
            )

            if summary_data is not None:
                if trial_num in args.exclude:
                    print(f"Trial {trial_num} excluded from Master Report.")
                else:
                    summary_data['trial_num'] = trial_num
                    all_trial_summaries.append(summary_data)

        except Exception as e:
            print(f"[ERROR] Skipping {trial_name} due to unexpected error: {e}")
            traceback.print_exc()

    if all_trial_summaries:
        master_summary_df = pd.concat(all_trial_summaries).sort_values('trial_num')
        summary_filename = f"{args.session_id}_summary_report.csv" if args.session_id else "all_trials_summary_report.csv"
        master_summary_path = os.path.join(master_output_dir, summary_filename)
        master_summary_df.to_csv(master_summary_path, index=False)
        print(f"Master Summary saved to: {master_summary_path}")

        generate_trend_plots(master_summary_df, master_output_dir, args.session_id)
    else:
        print("[WARN] No trials were successfully analyzed.")


if __name__ == "__main__":
    main()