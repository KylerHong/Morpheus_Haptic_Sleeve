import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import seaborn as sns
import argparse
import re
from matplotlib.ticker import MaxNLocator

def get_quadrant(x, y):
    """Maps coordinates to quadrants using strict 0-1 boundaries."""
    try:
        x, y = float(x), float(y)
    except (ValueError, TypeError):
        return "Off-Surface"

    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return "Off-Surface"

    if x < 0.5:
        return "Top-Left" if y < 0.5 else "Bottom-Left"
    else:
        return "Top-Right" if y < 0.5 else "Bottom-Right"

def get_trial_number(trial_name):
    """Extracts the first number found in the trial string to enforce numerical plotting."""
    match = re.search(r'\d+', str(trial_name))
    return int(match.group()) if match else 99999

def calculate_gaze_entropy(df, bins):
    """Calculates Shannon Entropy for gaze based on fixation durations within an N x N grid."""
    if df.empty or 'fixation x [normalized]' not in df.columns or 'fixation y [normalized]' not in df.columns:
        return np.nan
        
    x = df['fixation x [normalized]'].values
    y = df['fixation y [normalized]'].values
    dur = (df['duration [ms]'] / 1000.0).values
    
    # Filter valid on-screen points
    valid = (x >= 0) & (x <= 1) & (y >= 0) & (y <= 1)
    x, y, dur = x[valid], y[valid], dur[valid]
    
    if np.sum(dur) == 0: 
        return np.nan
    
    # Map to grid indices
    x_idx = np.clip((x * bins).astype(int), 0, bins - 1)
    y_idx = np.clip((y * bins).astype(int), 0, bins - 1)
    idx = y_idx * bins + x_idx
    
    # Calculate probability distribution (p) based on duration
    df_bins = pd.DataFrame({'idx': idx, 'dur': dur})
    p = df_bins.groupby('idx')['dur'].sum() / np.sum(dur)
    
    # Shannon Entropy formula
    return -np.sum(p * np.log2(p))

def main():
    parser = argparse.ArgumentParser(description="Analyze all trial metrics (Counts, Normalized, Quadrants, Heatmap, Entropy).")
    parser.add_argument("--merged-dir", required=True, help="Path to the Merged_Data folder")
    args = parser.parse_args()

    merged_dir = args.merged_dir
    
    # OUTPUT FOLDER LOGIC
    parent_dir = os.path.dirname(os.path.normpath(merged_dir))
    output_dir = os.path.join(parent_dir, "Metrics_Output")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory ready at: {output_dir}\n")

    # Load datasets
    timeseries_path = os.path.join(merged_dir, "merged_full_timeseries.csv")
    blinks_path = os.path.join(merged_dir, "merged_raw_blinks_slice.csv")
    saccades_path = os.path.join(merged_dir, "merged_raw_saccades_slice.csv")
    fixations_path = os.path.join(merged_dir, "merged_normalized_fixation_slice.csv")

    if not os.path.exists(timeseries_path):
        print("Timeseries data not found. Run the merge_data.py script first.")
        return

    ts_df = pd.read_csv(timeseries_path)
    blinks_df = pd.read_csv(blinks_path) if os.path.exists(blinks_path) else pd.DataFrame()
    saccades_df = pd.read_csv(saccades_path) if os.path.exists(saccades_path) else pd.DataFrame()
    fix_df = pd.read_csv(fixations_path) if os.path.exists(fixations_path) else pd.DataFrame()

    # ==========================================
    # BASELINE CALCULATION (Pupil)
    # ==========================================
    all_trials = ts_df['Source_Trial'].unique()
    baseline_trials = [t for t in all_trials if "baseline" in str(t).lower()]
    
    baseline_trimmed_mean = np.nan
    if baseline_trials:
        print(f"Found baseline trial(s): {baseline_trials}")
        baseline_ts = ts_df[ts_df['Source_Trial'].isin(baseline_trials)]
        
        if 'pupil_diameter_mm' in baseline_ts.columns:
            pupils = baseline_ts['pupil_diameter_mm'].dropna().values
            if len(pupils) > 0:
                lower = np.percentile(pupils, 10)
                upper = np.percentile(pupils, 90)
                trimmed_pupils = pupils[(pupils >= lower) & (pupils <= upper)]
                
                if len(trimmed_pupils) > 0:
                    baseline_trimmed_mean = np.mean(trimmed_pupils)
                    print(f"Calculated Baseline Pupil Trimmed Mean: {baseline_trimmed_mean:.4f} mm\n")
    else:
        print("Warning: No baseline trial found. Pupil percentage change will output as NaN.\n")

    # Get active trials and sort NUMERICALLY
    trials = [t for t in all_trials if "baseline" not in str(t).lower()]
    trials = sorted(trials, key=get_trial_number)
    
    metrics = []

    for trial in trials:
        trial_num = get_trial_number(trial)
        trial_ts = ts_df[ts_df['Source_Trial'] == trial]
        
        # Duration
        duration = trial_ts['time_sec'].max() - trial_ts['time_sec'].min() if not trial_ts.empty else 0

        # ==========================================
        # EXCLUSION CRITERIA: Short Trials (< 10s)
        # ==========================================
        if duration < 10:
            print(f"Excluding {trial} metrics: Duration too short ({duration:.2f}s)")
            metrics.append({
                'Trial': trial,
                'Trial_Num': trial_num,
                'Duration_sec': np.nan,
                'Blink_Count': np.nan,
                'Blink_Rate_BPM': np.nan,
                'Saccade_Count': np.nan,
                'Avg_Saccade_Peak_Velocity': np.nan,
                'Raw_Pupil_Mean_mm': np.nan,
                'Trimmed_Pupil_Mean_mm': np.nan,
                'Pupil_Change_From_Baseline_%': np.nan,
                'Total_Valid_Fixation_Sec': np.nan,
                'Top-Left_sec': np.nan,
                'Top-Right_sec': np.nan,
                'Bottom-Left_sec': np.nan,
                'Bottom-Right_sec': np.nan,
                'Top-Left_%': np.nan,
                'Top-Right_%': np.nan,
                'Bottom-Left_%': np.nan,
                'Bottom-Right_%': np.nan,
                'Gaze_Entropy_4Quad': np.nan,
                'Gaze_Entropy_10x10': np.nan
            })
            continue  # Skips directly to the next trial without altering the raw DataFrames

        # Blinks (Count & Rate)
        if not blinks_df.empty and 'Source_Trial' in blinks_df.columns:
            trial_blinks = blinks_df[blinks_df['Source_Trial'] == trial]
            blink_count = len(trial_blinks)
        else:
            blink_count = 0
        blink_rate_bpm = (blink_count / duration) * 60 if duration > 0 else 0

        # Saccades (Count & Peak Velocity)
        saccade_count = 0
        avg_peak_vel = np.nan
        if not saccades_df.empty and 'Source_Trial' in saccades_df.columns:
            trial_saccades = saccades_df[saccades_df['Source_Trial'] == trial]
            saccade_count = len(trial_saccades)
            
            vel_col = 'peak velocity [px/s]' if 'peak velocity [px/s]' in trial_saccades.columns else None
            if not vel_col and 'peak velocity [deg/s]' in trial_saccades.columns:
                vel_col = 'peak velocity [deg/s]'
                
            if vel_col:
                vels = trial_saccades[vel_col].dropna().values
                if len(vels) > 0:
                    avg_peak_vel = np.mean(vels)

        # Pupil (Raw, Trimmed, % Change)
        raw_mean, trimmed_mean, pupil_change_pct = np.nan, np.nan, np.nan
        if 'pupil_diameter_mm' in trial_ts.columns:
            pupils = trial_ts['pupil_diameter_mm'].dropna().values
            if len(pupils) > 0:
                raw_mean = np.mean(pupils)
                lower = np.percentile(pupils, 10)
                upper = np.percentile(pupils, 90)
                trimmed_pupils = pupils[(pupils >= lower) & (pupils <= upper)]
                if len(trimmed_pupils) > 0:
                    trimmed_mean = np.mean(trimmed_pupils)
                    if not np.isnan(baseline_trimmed_mean) and baseline_trimmed_mean > 0:
                        pupil_change_pct = ((trimmed_mean - baseline_trimmed_mean) / baseline_trimmed_mean) * 100

        # Quadrants & Gaze Entropy
        total_valid_fixation_sec = 0.0
        top_left_pct, top_right_pct, bottom_left_pct, bottom_right_pct = 0.0, 0.0, 0.0, 0.0
        top_left_sec, top_right_sec, bottom_left_sec, bottom_right_sec = 0.0, 0.0, 0.0, 0.0
        entropy_4quad, entropy_10x10 = np.nan, np.nan
        
        if not fix_df.empty and 'fixation x [normalized]' in fix_df.columns and 'fixation y [normalized]' in fix_df.columns:
            trial_fix_data = fix_df[fix_df['Source_Trial'] == trial].copy()
            if not trial_fix_data.empty:
                # Quadrant Percentages
                trial_fix_data['Calculated_Quadrant'] = trial_fix_data.apply(
                    lambda row: get_quadrant(row['fixation x [normalized]'], row['fixation y [normalized]']), axis=1
                )
                trial_fix_data['duration_sec'] = trial_fix_data['duration [ms]'] / 1000.0
                quadrant_totals = trial_fix_data.groupby('Calculated_Quadrant')['duration_sec'].sum().reset_index()
                
                valid_quadrants = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right"]
                valid_data = quadrant_totals[quadrant_totals['Calculated_Quadrant'].isin(valid_quadrants)]
                total_valid_fixation_sec = valid_data['duration_sec'].sum()

                if total_valid_fixation_sec > 0:
                    for _, row in valid_data.iterrows():
                        quad = row['Calculated_Quadrant']
                        pct = (row['duration_sec'] / total_valid_fixation_sec) * 100
                        if quad == "Top-Left": 
                            top_left_pct = pct
                            top_left_sec = row['duration_sec']
                        elif quad == "Top-Right": 
                            top_right_pct = pct
                            top_right_sec = row['duration_sec']
                        elif quad == "Bottom-Left": 
                            bottom_left_pct = pct
                            bottom_left_sec = row['duration_sec']
                        elif quad == "Bottom-Right":
                            bottom_right_pct = pct
                            bottom_right_sec = row['duration_sec']
                            
                    # Calculate 4-Quadrant Entropy perfectly aligned with AOIs
                    quad_durs = np.array([top_left_sec, top_right_sec, bottom_left_sec, bottom_right_sec])
                    p_quad = quad_durs[quad_durs > 0] / total_valid_fixation_sec
                    entropy_4quad = -np.sum(p_quad * np.log2(p_quad))

                # Calculate Grid Entropies
                entropy_10x10 = calculate_gaze_entropy(trial_fix_data, bins=10)

        metrics.append({
            'Trial': trial,
            'Trial_Num': trial_num,
            'Duration_sec': round(duration, 2),
            'Blink_Count': blink_count,
            'Blink_Rate_BPM': round(blink_rate_bpm, 2),
            'Saccade_Count': saccade_count,
            'Avg_Saccade_Peak_Velocity': round(avg_peak_vel, 2) if not np.isnan(avg_peak_vel) else np.nan,
            'Raw_Pupil_Mean_mm': round(raw_mean, 4) if not np.isnan(raw_mean) else np.nan,
            'Trimmed_Pupil_Mean_mm': round(trimmed_mean, 4) if not np.isnan(trimmed_mean) else np.nan,
            'Pupil_Change_From_Baseline_%': round(pupil_change_pct, 2) if not np.isnan(pupil_change_pct) else np.nan,
            'Total_Valid_Fixation_Sec': round(total_valid_fixation_sec, 2),
            'Top-Left_sec': round(top_left_sec, 2),
            'Top-Right_sec': round(top_right_sec, 2),
            'Bottom-Left_sec': round(bottom_left_sec, 2),
            'Bottom-Right_sec': round(bottom_right_sec, 2),
            'Top-Left_%': round(top_left_pct, 2),
            'Top-Right_%': round(top_right_pct, 2),
            'Bottom-Left_%': round(bottom_left_pct, 2),
            'Bottom-Right_%': round(bottom_right_pct, 2),
            'Gaze_Entropy_4Quad': round(entropy_4quad, 4) if not np.isnan(entropy_4quad) else np.nan,
            'Gaze_Entropy_10x10': round(entropy_10x10, 4) if not np.isnan(entropy_10x10) else np.nan
        })

    metrics_df = pd.DataFrame(metrics)
    
    # Save combined metrics summary
    metrics_csv_path = os.path.join(output_dir, "metrics_summary.csv")
    metrics_df.to_csv(metrics_csv_path, index=False)
    print(f"Saved tabular data to: {metrics_csv_path}")

    # ==========================================
    # VISUALIZATIONS
    # ==========================================
    x_labels = metrics_df['Trial_Num']

    plots = [
        ('Blink_Count', 'Blinks per Trial', 'Total Blinks', '#1f77b4', 'metrics_blinks.png'),
        ('Blink_Rate_BPM', 'Blink Rate per Trial', 'Blinks Per Minute (BPM)', '#1f77b4', 'metrics_blink_rate.png'),
        ('Duration_sec', 'Trial Durations', 'Duration (Seconds)', '#2ca02c', 'metrics_durations.png'),
        ('Saccade_Count', 'Saccades per Trial', 'Total Saccades', '#9467bd', 'metrics_saccades.png'),
        ('Avg_Saccade_Peak_Velocity', 'Average Saccade Peak Velocity', 'Peak Velocity', '#9467bd', 'metrics_saccade_velocity.png'),
    ]

    for col, title, ylabel, color, filename in plots:
        plt.figure(figsize=(10, 5))
        plt.plot(x_labels, metrics_df[col], marker='o', color=color, linewidth=2)
        plt.title(title, fontweight='bold')
        plt.xlabel('Trial', fontweight='bold')
        plt.ylabel(ylabel, fontweight='bold')
        plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, filename), dpi=150)
        plt.close()

    # Pupil Diameter (Raw vs. Trimmed)
    plt.figure(figsize=(10, 5))
    plt.plot(x_labels, metrics_df['Raw_Pupil_Mean_mm'], label='Raw Average', marker='s', color='#d62728', linestyle='--')
    plt.plot(x_labels, metrics_df['Trimmed_Pupil_Mean_mm'], label='Trimmed Avg (Middle 80%)', marker='^', color='#ff7f0e', linewidth=2)
    plt.title('Pupil Diameter (Raw vs. Trimmed)', fontweight='bold')
    plt.xlabel('Trial', fontweight='bold')
    plt.ylabel('Diameter (mm)', fontweight='bold')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "metrics_pupil.png"), dpi=150)
    plt.close()
    
    # Pupil % Change
    if not metrics_df['Pupil_Change_From_Baseline_%'].isna().all():
        plt.figure(figsize=(10, 5))
        plt.plot(x_labels, metrics_df['Pupil_Change_From_Baseline_%'], marker='^', color='#ff7f0e', linewidth=2)
        plt.axhline(0, color='red', linestyle='--', alpha=0.6, label='Baseline (0%)')
        plt.title('Pupil Diameter % Change from Baseline', fontweight='bold')
        plt.xlabel('Trial', fontweight='bold')
        plt.ylabel('% Change', fontweight='bold')
        plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
        plt.legend()
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, "metrics_pupil_change.png"), dpi=150)
        plt.close()

    # Quadrants Trend Graph
    plt.figure(figsize=(12, 7))
    plt.plot(metrics_df['Trial_Num'], metrics_df['Top-Left_%'], marker='o', linewidth=2, color='#ff7f0e', label='Top-Left')
    plt.plot(metrics_df['Trial_Num'], metrics_df['Top-Right_%'], marker='s', linewidth=2, color='#1f77b4', label='Top-Right')
    plt.plot(metrics_df['Trial_Num'], metrics_df['Bottom-Left_%'], marker='^', linewidth=2, color='#d62728', label='Bottom-Left')
    plt.title('Attention Distribution Trends Across Trials (Active Quadrants Only)', fontsize=16, fontweight='bold')
    plt.xlabel('Trial Number', fontsize=12, fontweight='bold')
    plt.ylabel('Percentage of Valid Fixation Time (%)', fontsize=12, fontweight='bold')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    plt.ylim(-5, 105) 
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(title='Quadrants', fontsize=10, title_fontsize=12, loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "metrics_quadrants.png"), dpi=150)
    plt.close()

    # Gaze Entropy Trend Graph
    plt.figure(figsize=(12, 6))
    plt.plot(metrics_df['Trial_Num'], metrics_df['Gaze_Entropy_4Quad'], marker='o', linewidth=2, label='4-Quadrant (AOI)')
    plt.plot(metrics_df['Trial_Num'], metrics_df['Gaze_Entropy_10x10'], marker='^', linewidth=2, label='10x10 Grid')
    plt.title('Gaze Entropy Across Trials', fontsize=16, fontweight='bold')
    plt.xlabel('Trial Number', fontsize=12, fontweight='bold')
    plt.ylabel('Entropy (Bits)', fontsize=12, fontweight='bold')
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(title='Entropy Type', fontsize=10, loc='best')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "metrics_gaze_entropy.png"), dpi=150)
    plt.close()

    # ==========================================
    # HEATMAP GENERATION
    # ==========================================
    if not fix_df.empty and 'fixation x [normalized]' in fix_df.columns:
        print("Generating session heatmap...")
        heatmap_df = fix_df.copy()
        x_col = 'fixation x [normalized]'
        y_col = 'fixation y [normalized]'
        
        if 'Source_Trial' in heatmap_df.columns:
            heatmap_df = heatmap_df[~heatmap_df['Source_Trial'].astype(str).str.lower().str.contains("baseline")]
        heatmap_df = heatmap_df[(heatmap_df[x_col] >= 0) & (heatmap_df[x_col] <= 1) & (heatmap_df[y_col] >= 0) & (heatmap_df[y_col] <= 1)]
        
        if len(heatmap_df) >= 10:
            MAX_POINTS = 30000
            if len(heatmap_df) > MAX_POINTS:
                stride = len(heatmap_df) // MAX_POINTS
                heatmap_df = heatmap_df.iloc[::stride]
            
            plt.figure(figsize=(10, 8))
            x = heatmap_df[x_col]
            y = 1 - heatmap_df[y_col] # Flip Y for screen orientation
            
            try:
                colors = ['#4b0082', '#0000ff', '#00ffff', '#00ff00', '#ffff00', '#ff8c00', '#ff0000']
                weather_cmap = mcolors.LinearSegmentedColormap.from_list("weather_map", colors, N=256)
                sns.kdeplot(x=x, y=y, fill=True, cmap=weather_cmap, alpha=0.8, levels=60, thresh=0.05)
                
                plt.xlim(0, 1)
                plt.ylim(0, 1)
                session_name = os.path.basename(parent_dir)
                plt.suptitle("Overall Session Attention Heatmap", fontsize=18, y=0.98, fontweight='bold')
                plt.title(f"Source: {session_name} | Active Trials Only", fontsize=10)
                plt.axis('off')
                
                plt.gca().add_patch(plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor='black', lw=3))
                plt.axvline(x=0.5, color='black', linestyle='-', linewidth=2) 
                plt.axhline(y=0.5, color='black', linestyle='-', linewidth=2) 
                
                plt.tight_layout(rect=[0, 0, 1, 1])
                heatmap_path = os.path.join(output_dir, "metrics_heatmap.png")
                plt.savefig(heatmap_path, dpi=300, bbox_inches='tight')
                plt.close()
                print(f"Saved Session Heatmap to: {heatmap_path}")
            except Exception as e:
                print(f"Heatmap Plotting Error: {e}")
                plt.close()

    print(f"Saved all visualizations to: {output_dir}\n")

if __name__ == "__main__":
    main()