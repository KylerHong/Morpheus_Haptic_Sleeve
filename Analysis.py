import pandas as pd
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import numpy as np
import glob
import traceback


# --- HELPERS ---

def find_lsl_file_for_trial(lsl_dir, trial_num):
    try:
        clean_trial_num = int(trial_num)
        # Look for standard LSL naming convention
        pattern = os.path.join(lsl_dir, f"*_{clean_trial_num:03d}_lsl.csv")
        matches = glob.glob(pattern)
        return matches[0] if matches else None
    except:
        return None


def calculate_offset_via_fingerprint(lsl_path, gaze_df, sequence_length=500):
    """
    Finds the exact time offset between LSL (ROS time) and Glasses (ns)
    using a coarse-to-fine shape matching approach.
    """
    try:
        lsl_df = pd.read_csv(lsl_path)
        if lsl_df.empty: return None

        # Identify columns based on unit type
        lsl_x, lsl_y = 'neon_ch0', 'neon_ch1'
        if 'gaze x [normalized]' in gaze_df.columns:
            gaze_x, gaze_y = 'gaze x [normalized]', 'gaze y [normalized]'
        else:
            gaze_x, gaze_y = 'gaze x [px]', 'gaze y [px]'

        lsl_arr = lsl_df[[lsl_x, lsl_y]].values
        gaze_arr = gaze_df[[gaze_x, gaze_y]].values

        # Grab a template chunk from LSL.
        # Offset by 200 samples to avoid startup flatlines/noise.
        start_offset = 200
        if len(lsl_arr) < start_offset + sequence_length:
            return None

        template = lsl_arr[start_offset: start_offset + sequence_length]

        # Normalize template (0-1) to handle unit mismatch
        t_min, t_max = template.min(axis=0), template.max(axis=0)
        template_norm = (template - t_min) / (t_max - t_min + 1e-9)

        gaze_len = len(gaze_arr)

        # --- PASS 1: Coarse Search (Speed) ---
        # Skip 100 samples at a time to find the rough neighborhood
        coarse_stride = 100
        best_coarse_err = float('inf')
        best_coarse_idx = -1

        for i in range(0, gaze_len - sequence_length, coarse_stride):
            segment = gaze_arr[i: i + sequence_length]

            # Local normalization
            s_min, s_max = segment.min(axis=0), segment.max(axis=0)
            segment_norm = (segment - s_min) / (s_max - s_min + 1e-9)

            error = np.mean(np.abs(segment_norm - template_norm))

            if error < best_coarse_err:
                best_coarse_err = error
                best_coarse_idx = i

        if best_coarse_idx == -1: return None

        # --- PASS 2: Fine Search (Precision) ---
        # Search every sample around the coarse match
        radius = coarse_stride
        start_fine = max(0, best_coarse_idx - radius)
        end_fine = min(gaze_len - sequence_length, best_coarse_idx + radius)

        best_fine_err = float('inf')
        best_fine_idx = -1

        for i in range(start_fine, end_fine):
            segment = gaze_arr[i: i + sequence_length]

            # Re-normalize for exact precision
            s_min, s_max = segment.min(axis=0), segment.max(axis=0)
            segment_norm = (segment - s_min) / (s_max - s_min + 1e-9)

            error = np.mean(np.abs(segment_norm - template_norm))

            if error < best_fine_err:
                best_fine_err = error
                best_fine_idx = i

        # Calculate final offset if match is found
        if best_fine_idx != -1:
            lsl_ros_time = lsl_df.iloc[start_offset]['ros_time']
            gaze_ns = gaze_df.iloc[best_fine_idx]['timestamp [ns]']
            ros_ns = int(lsl_ros_time * 1e9)

            return gaze_ns - ros_ns

        return None
    except Exception:
        return None


def get_quadrant(x, y):
    # Maps normalized coords (0-1) to screen quadrants
    try:
        x, y = float(x), float(y)
    except:
        return None

    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0): return None

    if x < 0.5:
        return "Top-Left" if y < 0.5 else "Bottom-Left"
    else:
        return "Top-Right" if y < 0.6 else "Bottom-Right"


# --- ANALYSIS & PLOTTING ---

def generate_summary_stats(merged_df, eye_df, saccades_df, imu_df, output_dir, trial_name):
    # Saves a small CSV with average durations, diameters, velocities, etc.
    stats = {}
    try:
        if eye_df is not None and not eye_df.empty:
            stats['avg_eyelid_aperture_mm'] = float(eye_df['eyelid_aperture_avg'].mean())
            stats['avg_pupil_diameter_mm'] = float(eye_df['pupil_diameter_avg'].mean())

        if merged_df is not None and not merged_df.empty:
            stats['avg_fixation_duration_s'] = float(merged_df['duration_sec'].mean())
            stats['peak_fixation_duration_s'] = float(merged_df['duration_sec'].max())
            stats['total_fixations'] = len(merged_df)

        if saccades_df is not None and not saccades_df.empty:
            # Handle variable column names for velocity
            vel_col = next((c for c in ['peak velocity [px/s]', 'peak velocity [deg/s]', 'velocity']
                            if c in saccades_df.columns), None)
            if vel_col:
                stats['avg_saccadic_velocity'] = float(saccades_df[vel_col].mean())
                stats['peak_saccadic_velocity'] = float(saccades_df[vel_col].max())
                stats['velocity_unit'] = vel_col

        if not stats: return None

        pd.DataFrame([stats]).to_csv(os.path.join(output_dir, f"{trial_name}_key_metrics.csv"), index=False)
        return True
    except Exception as e:
        print(f"   Stats generation failed: {e}")
        return None


def analyze_trial(trial_name, main_df, surface_df, eye_df, blinks_df, saccades_df, imu_df, gaze_df, trial_keys,
                  output_dir):
    if main_df.empty:
        print("   Warning: Fixation slice is empty.")

    try:
        # Determine T=0 (start of slice)
        start_time_ns = 0.0
        if eye_df is not None and not eye_df.empty:
            start_time_ns = float(eye_df['timestamp [ns]'].min())
        elif not main_df.empty:
            start_time_ns = float(main_df['start timestamp [ns]'].min())

        # --- 1. PREP STREAMS ---

        # Eye states (Pupil/Aperture)
        if eye_df is not None and not eye_df.empty:
            eye_df = eye_df.copy()
            eye_df['time_sec'] = (eye_df['timestamp [ns]'] - start_time_ns) * 1e-9
            # Average L/R eyes
            eye_df['pupil_diameter_avg'] = eye_df[['pupil diameter left [mm]', 'pupil diameter right [mm]']].mean(
                axis=1)
            eye_df['eyelid_aperture_avg'] = eye_df[['eyelid aperture left [mm]', 'eyelid aperture right [mm]']].mean(
                axis=1)

        # Raw Gaze
        gaze_ok = False
        if gaze_df is not None and not gaze_df.empty:
            gaze_df = gaze_df.copy()
            gaze_df['time_sec'] = (gaze_df['timestamp [ns]'] - start_time_ns) * 1e-9

            cols = ['time_sec']
            if 'gaze x [px]' in gaze_df.columns:
                cols.extend(['gaze x [px]', 'gaze y [px]'])
            elif 'gaze x [normalized]' in gaze_df.columns:
                cols.extend(['gaze x [normalized]', 'gaze y [normalized]'])

            gaze_stream = gaze_df[cols].sort_values('time_sec')
            gaze_ok = True

        # IMU
        head_vel_ok = False
        if imu_df is not None and not imu_df.empty:
            imu_df = imu_df.copy()
            imu_df['time_sec'] = (imu_df['timestamp [ns]'] - start_time_ns) * 1e-9

            gyro_cols = ['gyro x [deg/s]', 'gyro y [deg/s]', 'gyro z [deg/s]']
            if all(c in imu_df.columns for c in gyro_cols):
                imu_df['head_angular_velocity'] = np.linalg.norm(imu_df[gyro_cols].values, axis=1)
                head_vel_ok = True

        # Keys
        keys_ok = False
        if trial_keys is not None and not trial_keys.empty:
            trial_keys = trial_keys.copy()
            trial_keys['time_sec'] = (trial_keys['timestamp [ns]'] - start_time_ns) * 1e-9
            # Prioritize event label over key code
            if 'event' in trial_keys.columns:
                trial_keys['button_pressed'] = trial_keys['event']
                keys_ok = True
            elif 'key' in trial_keys.columns:
                trial_keys['button_pressed'] = trial_keys['key']
                keys_ok = True

        # Fixations & Surfaces
        merged_df = pd.DataFrame()
        if not main_df.empty and not surface_df.empty:
            surface_df = surface_df.copy()
            # Map coords to quadrants
            surface_df['quadrant'] = surface_df.apply(
                lambda r: get_quadrant(r['fixation x [normalized]'], r['fixation y [normalized]']), axis=1)

            # Merge quadrant info back to main fixation list
            merged_df = pd.merge(main_df, surface_df[['fixation id', 'quadrant']], on='fixation id', how='left')
            merged_df['quadrant'] = merged_df['quadrant'].fillna('Off-Surface')
            merged_df.dropna(subset=['quadrant'], inplace=True)

            if not merged_df.empty:
                merged_df['start_time_sec'] = (merged_df['start timestamp [ns]'] - start_time_ns) * 1e-9
                merged_df['duration_sec'] = merged_df['duration [ms]'] * 1e-3
                merged_df['end_time_sec'] = merged_df['start_time_sec'] + merged_df['duration_sec']

        # Saccades
        sacc_ok = False
        sacc_vel_col = None
        if saccades_df is not None and not saccades_df.empty:
            saccades_df = saccades_df.copy()
            saccades_df['time_sec'] = (saccades_df['start timestamp [ns]'] - start_time_ns) * 1e-9
            # Auto-detect velocity column
            for c in ['peak velocity [px/s]', 'peak velocity [deg/s]', 'velocity']:
                if c in saccades_df.columns:
                    sacc_vel_col = c
                    sacc_ok = True
                    break

        # Blinks
        blinks_ok = False
        if blinks_df is not None and not blinks_df.empty:
            blinks_df = blinks_df.copy()
            blinks_df['start_time_sec'] = (blinks_df['start timestamp [ns]'] - start_time_ns) * 1e-9
            blinks_df['duration_sec'] = blinks_df['duration [ms]'] * 1e-3
            blinks_ok = True

        # --- 2. GENERATE FULL TIMESERIES CSV ---

        # Base timeline on eye data frequency
        timeseries_df = eye_df[['time_sec', 'pupil_diameter_avg', 'eyelid_aperture_avg']].copy()
        timeseries_df.rename(
            columns={'pupil_diameter_avg': 'pupil_diameter_mm', 'eyelid_aperture_avg': 'eyelid_aperture_mm'},
            inplace=True)
        timeseries_df.sort_values('time_sec', inplace=True)

        # Merge streams
        if gaze_ok: timeseries_df = pd.merge_asof(timeseries_df, gaze_stream, on='time_sec', direction='nearest',
                                                  tolerance=0.01)

        if head_vel_ok:
            timeseries_df = pd.merge_asof(timeseries_df, imu_df[['time_sec', 'head_angular_velocity']], on='time_sec',
                                          direction='backward')
            timeseries_df.rename(columns={'head_angular_velocity': 'head_angular_velocity_deg_s'}, inplace=True)

        if keys_ok:
            timeseries_df = pd.merge_asof(timeseries_df, trial_keys[['time_sec', 'button_pressed']], on='time_sec',
                                          direction='nearest', tolerance=0.05)

        # Map active fixation/quadrant to timestamp
        if not merged_df.empty:
            try:
                intervals = pd.IntervalIndex.from_arrays(merged_df['start_time_sec'], merged_df['end_time_sec'],
                                                         closed='both')
                indices = intervals.get_indexer(timeseries_df['time_sec'])

                valid = indices != -1
                mapped_ids = np.full(len(timeseries_df), np.nan)
                mapped_quads = np.full(len(timeseries_df), None, dtype=object)

                # Assign values where time overlaps
                mapped_ids[valid] = merged_df['fixation id'].values[indices[valid]]
                mapped_quads[valid] = merged_df['quadrant'].values[indices[valid]]

                timeseries_df['current_fixation_id'] = mapped_ids
                timeseries_df['current_quadrant'] = mapped_quads
            except:
                pass

        timeseries_df.to_csv(os.path.join(output_dir, f"{trial_name}_full_timeseries.csv"), index=False)

        # --- 3. PLOTTING ---

        color_map = {
            "Top-Right": '#1f77b4', "Top-Left": '#ff7f0e',
            "Bottom-Right": '#2ca02c', "Bottom-Left": '#d62728',
            "Off-Surface": '#7f7f7f'
        }

        # Helper to stripe background based on quadrants
        def add_shading(ax):
            if merged_df.empty: return
            for _, row in merged_df.iterrows():
                color = color_map.get(row['quadrant'], 'white')
                ax.axvspan(row['start_time_sec'], row['end_time_sec'], color=color, alpha=0.15, lw=0)

        plots_to_create = {
            'timeline': not merged_df.empty,
            'aperture': eye_df is not None and not eye_df.empty,
            'pupil': eye_df is not None and not eye_df.empty,
            'fix_duration': not merged_df.empty,
            'saccade_vel': sacc_ok,
            'head_velocity': head_vel_ok,
            'blinks': blinks_ok
        }
        active_plots = [k for k, v in plots_to_create.items() if v]
        num_plots = len(active_plots)

        if num_plots > 0:
            # Wide layout (24), slightly compressed height
            fig, axs = plt.subplots(num_plots, 1, figsize=(24, 2.5 * num_plots), sharex=True)
            if num_plots == 1: axs = [axs]

            fig.suptitle(f"Cognitive Load Analysis - {trial_name}", fontsize=18, y=0.95)
            curr_ax_idx = 0

            def stylize(ax, ylabel):
                ax.set_ylabel(ylabel, fontsize=12)
                ax.tick_params(axis='both', which='major', labelsize=10)
                ax.grid(True, alpha=0.3)
                add_shading(ax)

            # A. Timeline Bar
            if 'timeline' in active_plots:
                ax = axs[curr_ax_idx]
                y_map = {"Top-Right": 4, "Top-Left": 3, "Bottom-Right": 2, "Bottom-Left": 1, "Off-Surface": 0}
                merged_df['y_pos'] = merged_df['quadrant'].apply(lambda q: y_map.get(q, 0))
                merged_df['color'] = merged_df['quadrant'].apply(lambda q: color_map.get(q, 'grey'))

                ax.barh(y=merged_df['y_pos'], width=merged_df['duration_sec'], left=merged_df['start_time_sec'],
                        color=merged_df['color'], height=0.6)
                ax.set_yticks(list(y_map.values()))
                ax.set_yticklabels(list(y_map.keys()), fontsize=10)
                ax.set_title("Timeline of Visual Attention", fontsize=14)
                ax.grid(axis='x', linestyle=':', alpha=0.5)
                # No shading on top plot (cleaner look)
                curr_ax_idx += 1

            # B. Aperture
            if 'aperture' in active_plots:
                ax = axs[curr_ax_idx]
                ax.plot(eye_df['time_sec'], eye_df['eyelid_aperture_avg'], color='purple', linewidth=2.5)
                stylize(ax, "Aperture (mm)")
                curr_ax_idx += 1

            # C. Pupil
            if 'pupil' in active_plots:
                ax = axs[curr_ax_idx]
                ax.plot(eye_df['time_sec'], eye_df['pupil_diameter_avg'], color='teal', linewidth=2.5)
                stylize(ax, "Diameter (mm)")
                curr_ax_idx += 1

            # D. Duration (Lollipops)
            if 'fix_duration' in active_plots:
                ax = axs[curr_ax_idx]
                ax.vlines(merged_df['start_time_sec'], 0, merged_df['duration_sec'], color='grey', alpha=0.5,
                          linewidth=1.5)
                ax.scatter(merged_df['start_time_sec'], merged_df['duration_sec'], color='grey', s=50)
                stylize(ax, "Duration (s)")
                curr_ax_idx += 1

            # E. Saccade Velocity
            if 'saccade_vel' in active_plots:
                ax = axs[curr_ax_idx]
                ax.plot(saccades_df['time_sec'], saccades_df[sacc_vel_col], color='darkorange', marker='.',
                        linestyle='None', alpha=0.8, markersize=8)
                stylize(ax, f"Velocity\n({sacc_vel_col.split('[')[-1][:-1]})")
                curr_ax_idx += 1

            # F. Head Velocity
            if 'head_velocity' in active_plots:
                ax = axs[curr_ax_idx]
                ax.plot(imu_df['time_sec'], imu_df['head_angular_velocity'], color='black', linewidth=2)
                stylize(ax, "Head Vel (deg/s)")
                curr_ax_idx += 1

            # G. Blinks
            if 'blinks' in active_plots:
                ax = axs[curr_ax_idx]
                for _, row in blinks_df.iterrows():
                    ax.axvspan(row['start_time_sec'], row['start_time_sec'] + row['duration_sec'], color='red',
                               alpha=0.5)
                    ax.axvline(row['start_time_sec'], color='red', linewidth=0.5, alpha=0.8)
                ax.set_yticks([])
                ax.set_ylabel("Blinks", fontsize=12)
                add_shading(ax)
                curr_ax_idx += 1

            axs[-1].set_xlabel("Time (seconds)", fontsize=12)
            legend_patches = [mpatches.Patch(color=c, label=l) for l, c in color_map.items()]
            axs[0].legend(handles=legend_patches, bbox_to_anchor=(1.01, 1.05), loc='upper left', fontsize=10)

            # Pad top to ensure title visibility
            fig.tight_layout(rect=[0, 0.02, 1, 0.92])

            fig.savefig(os.path.join(output_dir, f"{trial_name}_timeline_plot.png"), dpi=100)
            plt.close(fig)

        return generate_summary_stats(merged_df, eye_df, saccades_df, imu_df, output_dir, trial_name)

    except Exception as e:
        print(f"   Analysis failed: {e}")
        traceback.print_exc()
        plt.close('all')
        return None


# --- MAIN ---

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--main-dir", required=True)
    parser.add_argument("--surface-dir", required=True)
    parser.add_argument("--keystroke-file", required=True)
    parser.add_argument("--lsl-dir", required=True)
    parser.add_argument("--session-id", default="")
    args = parser.parse_args()

    print("Loading data...")
    try:
        gaze_path = os.path.join(args.main_dir, "gaze.csv")
        if not os.path.exists(gaze_path): sys.exit(f"Error: gaze.csv not found in {args.main_dir}")

        all_gaze_df = pd.read_csv(gaze_path)
        all_eye_df = pd.read_csv(os.path.join(args.main_dir, "3d_eye_states.csv"))
        all_main_df = pd.read_csv(os.path.join(args.main_dir, "fixations.csv"))
        all_surface_df = pd.read_csv(os.path.join(args.surface_dir, "fixations.csv"))
        keystrokes_df = pd.read_csv(args.keystroke_file)

        # Optional files
        sacc_path = os.path.join(args.main_dir, "saccades.csv")
        all_saccades_df = pd.read_csv(sacc_path) if os.path.exists(sacc_path) else None

        blinks_path = os.path.join(args.main_dir, "blinks.csv")
        all_blinks_df = pd.read_csv(blinks_path) if os.path.exists(blinks_path) else None

        imu_path = os.path.join(args.main_dir, "imu.csv")
        all_imu_df = pd.read_csv(imu_path) if os.path.exists(imu_path) else None

    except Exception as e:
        sys.exit(f"Load error: {e}")

    # Output setup
    root_dir = os.path.dirname(args.main_dir)
    out_folder = f"{args.session_id}_Analysis_Output" if args.session_id else "Analysis_Output"
    master_out = os.path.join(root_dir, out_folder)
    os.makedirs(master_out, exist_ok=True)

    # Get chronologically sorted trials
    start_events = keystrokes_df[keystrokes_df['event'] == 'start_recording'].sort_values('ros_time')
    print(f"Found {len(start_events)} trials.")

    # --- BASELINE LOGIC ---
    # Find the first "Meaningful" trial (>5s) to act as the anchor.
    # The baseline will run from recording start -> start of Anchor Trial.
    if not start_events.empty:
        print("\nRunning Baseline...")

        anchor_row = None
        anchor_trial_num = -1

        for row in start_events.itertuples():
            t_num = row.trial
            # Calculate duration
            stop_row = keystrokes_df[(keystrokes_df['event'] == 'stop_recording') & (keystrokes_df['trial'] == t_num)]

            duration = 0
            if not stop_row.empty:
                duration = float(stop_row.iloc[0]['ros_time']) - float(row.ros_time)

            # If valid trial found, lock it
            if duration > 5.0:
                anchor_row = row
                anchor_trial_num = int(t_num)
                print(f"   Anchor: Trial {anchor_trial_num} ({duration:.1f}s)")
                break
            else:
                print(f"   Skipping Trial {t_num} (too short: {duration:.1f}s)")

        if anchor_row is not None:
            anchor_start_ros = float(anchor_row.ros_time)
            lsl_file = find_lsl_file_for_trial(args.lsl_dir, anchor_trial_num)

            if lsl_file:
                # Sync based on anchor
                baseline_offset_ns = calculate_offset_via_fingerprint(lsl_file, all_gaze_df)

                if baseline_offset_ns is not None:
                    # Define Window
                    baseline_end_ns = int((anchor_start_ros * 1e9) + baseline_offset_ns)
                    baseline_start_ns = int(all_gaze_df.iloc[0]['timestamp [ns]'])

                    baseline_name = f"{args.session_id}_Baseline" if args.session_id else "Baseline"
                    trial_out_dir = os.path.join(master_out, f"{baseline_name}_Analysis")
                    os.makedirs(trial_out_dir, exist_ok=True)

                    # Helper to slice dataframe by timestamp
                    def slice_baseline(df, t_col='timestamp [ns]'):
                        return df[(df[t_col] >= baseline_start_ns) & (
                                    df[t_col] <= baseline_end_ns)] if df is not None else None

                    # Slice & Save raw chunks
                    sub_gaze = slice_baseline(all_gaze_df)
                    if sub_gaze is not None: sub_gaze.to_csv(
                        os.path.join(trial_out_dir, f"{baseline_name}_raw_gaze_slice.csv"), index=False)

                    sub_eye = slice_baseline(all_eye_df)
                    if sub_eye is not None: sub_eye.to_csv(
                        os.path.join(trial_out_dir, f"{baseline_name}_raw_3d_eye_states_slice.csv"), index=False)

                    sub_main = slice_baseline(all_main_df, 'start timestamp [ns]')
                    if sub_main is not None: sub_main.to_csv(
                        os.path.join(trial_out_dir, f"{baseline_name}_raw_fixations_slice.csv"), index=False)

                    sub_sacc = slice_baseline(all_saccades_df, 'start timestamp [ns]')
                    if sub_sacc is not None: sub_sacc.to_csv(
                        os.path.join(trial_out_dir, f"{baseline_name}_raw_saccades_slice.csv"), index=False)

                    # Slice streams for analysis
                    sub_surf = slice_baseline(all_surface_df, 'start timestamp [ns]')
                    sub_imu = slice_baseline(all_imu_df)
                    sub_blinks = slice_baseline(all_blinks_df, 'start timestamp [ns]')

                    sub_keys = keystrokes_df[keystrokes_df['ros_time'] < anchor_start_ros].copy()
                    sub_keys['timestamp [ns]'] = (sub_keys['ros_time'] * 1e9) + baseline_offset_ns

                    analyze_trial(baseline_name, sub_main, sub_surf, sub_eye, sub_blinks, sub_sacc, sub_imu, sub_gaze,
                                  sub_keys, trial_out_dir)
                    print("   Baseline complete.")
                else:
                    print(f"   Baseline skipped: Sync failed for Anchor Trial {anchor_trial_num}.")
            else:
                print(f"   Baseline skipped: No LSL file for Anchor Trial {anchor_trial_num}.")
        else:
            print("   Baseline skipped: No valid anchor trial found.")

    # --- PROCESS TRIALS ---
    for row in start_events.itertuples(index=False):
        trial_num = int(row.trial)
        trial_name = f"{args.session_id}_Trial_{trial_num}" if args.session_id else f"Trial_{trial_num}"
        print(f"\nProcessing {trial_name}...")

        lsl_file = find_lsl_file_for_trial(args.lsl_dir, trial_num)
        if not lsl_file:
            print("   Skipping: No LSL file.")
            continue

        offset_ns = calculate_offset_via_fingerprint(lsl_file, all_gaze_df)
        if offset_ns is None:
            print("   Skipping: Sync failed.")
            continue

        # Define time window
        ros_start = float(row.ros_time)
        start_ns = int((ros_start * 1e9) + offset_ns)

        stop_row = keystrokes_df[(keystrokes_df['event'] == 'stop_recording') & (keystrokes_df['trial'] == trial_num)]
        if not stop_row.empty:
            end_ns = int((float(stop_row.iloc[0]['ros_time']) * 1e9) + offset_ns)
        else:
            # Fallback duration if stop event missing
            end_ns = start_ns + int(60 * 1e9)

        trial_out_dir = os.path.join(master_out, f"{trial_name}_Analysis")
        os.makedirs(trial_out_dir, exist_ok=True)

        def slice_df(df, t_col='timestamp [ns]'):
            return df[(df[t_col] >= start_ns) & (df[t_col] <= end_ns)] if df is not None else None

        # Slice and save raw data
        sub_gaze = slice_df(all_gaze_df)
        if sub_gaze is not None: sub_gaze.to_csv(os.path.join(trial_out_dir, f"{trial_name}_raw_gaze_slice.csv"),
                                                 index=False)

        sub_eye = slice_df(all_eye_df)
        if sub_eye is not None: sub_eye.to_csv(os.path.join(trial_out_dir, f"{trial_name}_raw_3d_eye_states_slice.csv"),
                                               index=False)

        sub_main = slice_df(all_main_df, 'start timestamp [ns]')
        if sub_main is not None: sub_main.to_csv(os.path.join(trial_out_dir, f"{trial_name}_raw_fixations_slice.csv"),
                                                 index=False)

        sub_sacc = slice_df(all_saccades_df, 'start timestamp [ns]')
        if sub_sacc is not None: sub_sacc.to_csv(os.path.join(trial_out_dir, f"{trial_name}_raw_saccades_slice.csv"),
                                                 index=False)

        sub_surf = slice_df(all_surface_df, 'start timestamp [ns]')
        sub_imu = slice_df(all_imu_df)
        sub_blinks = slice_df(all_blinks_df, 'start timestamp [ns]')

        sub_keys = keystrokes_df[keystrokes_df['trial'] == trial_num].copy()
        sub_keys['timestamp [ns]'] = (sub_keys['ros_time'] * 1e9) + offset_ns

        analyze_trial(trial_name, sub_main, sub_surf, sub_eye, sub_blinks, sub_sacc, sub_imu, sub_gaze, sub_keys,
                      trial_out_dir)

    print("\nAll tasks completed.")


if __name__ == "__main__":
    main()