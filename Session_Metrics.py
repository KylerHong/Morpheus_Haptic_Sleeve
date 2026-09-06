import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import argparse

def load_and_prep(filepath, session_num):
    """Loads the metrics_summary.csv and tags it with a session identifier."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Could not find {filepath}")
    
    df = pd.read_csv(filepath)
    df['Session'] = f"Session {session_num}"
    df['Session_Short'] = f"S{session_num}"
    return df

def main():
    parser = argparse.ArgumentParser(description="Visualize metrics and quadrant trends across multiple sessions")
    parser.add_argument("--session-1", required=True, help="Path to Session 1 metrics_summary.csv")
    parser.add_argument("--session-2", required=True, help="Path to Session 2 metrics_summary.csv")
    parser.add_argument("--session-3", required=True, help="Path to Session 3 metrics_summary.csv")
    args = parser.parse_args()

    print("Loading session data...")
    df1 = load_and_prep(args.session_1, 1)
    df2 = load_and_prep(args.session_2, 2)
    df3 = load_and_prep(args.session_3, 3)

    combined_df = pd.concat([df1, df2, df3], ignore_index=True)
    combined_df['Absolute_Trial'] = combined_df.index + 1

    parent_dir = os.path.dirname(os.path.dirname(os.path.normpath(args.session_1)))
    output_dir = os.path.join(parent_dir, "Session_Output")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory ready at: {output_dir}\n")

    # Combine Old and New Metrics
    metrics_to_plot = {
        'Blink_Count': ('Total Blinks Across All Sessions', 'Total Blinks', '#1f77b4'),
        'Blink_Rate_BPM': ('Blink Rate Across All Sessions', 'Blinks Per Minute (BPM)', '#1f77b4'),
        'Saccade_Count': ('Total Saccades Across All Sessions', 'Total Saccades', '#9467bd'),
        'Avg_Saccade_Peak_Velocity': ('Average Saccade Peak Velocity Across Sessions', 'Peak Velocity', '#9467bd'),
        'Duration_sec': ('Trial Durations Across All Sessions', 'Duration (Seconds)', '#2ca02c'),
        'Trimmed_Pupil_Mean_mm': ('Trimmed Pupil Diameter Across Sessions', 'Diameter (mm)', '#ff7f0e'),
        'Pupil_Change_From_Baseline_%': ('Pupil Diameter % Change Across Sessions', '% Change from Baseline', '#ff7f0e'),
        'Gaze_Entropy_4Quad': ('4-Quadrant Gaze Entropy Across Sessions', 'Entropy (Bits)', '#2ca02c'),
        'Gaze_Entropy_10x10': ('10x10 Grid Gaze Entropy Across Sessions', 'Entropy (Bits)', '#9467bd')
    }

    s1_end = len(df1) + 0.5
    s2_end = len(df1) + len(df2) + 0.5

    s1_center = len(df1) / 2 + 0.5
    s2_center = len(df1) + len(df2) / 2 + 0.5
    s3_center = len(df1) + len(df2) + len(df3) / 2 + 0.5

    # Generate single-line metric plots
    for col, (title, ylabel, color) in metrics_to_plot.items():
        if col not in combined_df.columns or combined_df[col].isna().all():
            print(f"Skipping {col}, not found or empty in data.")
            continue

        plt.figure(figsize=(14, 6))
        plt.plot(combined_df['Absolute_Trial'], combined_df[col], marker='o', color=color, linewidth=2, label=col)
        
        if col == 'Pupil_Change_From_Baseline_%':
            plt.axhline(0, color='red', linestyle='--', alpha=0.6, label='Baseline (0%)')
            plt.axvline(x=s1_end, color='red', linestyle='--', alpha=0.6)
            plt.axvline(x=s2_end, color='red', linestyle='--', alpha=0.6)
        else:
            plt.axvline(x=s1_end, color='red', linestyle='--', alpha=0.6, label='Session Boundary')
            plt.axvline(x=s2_end, color='red', linestyle='--', alpha=0.6)

        y_max = combined_df[col].max()
        y_min = combined_df[col].min()
        if pd.isna(y_max) or pd.isna(y_min):
            continue
        if y_max == y_min:
            y_max += 1
            y_min -= 1

        text_y = y_max + (y_max - y_min) * 0.1
        plt.text(s1_center, text_y, 'Session 1', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s2_center, text_y, 'Session 2', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s3_center, text_y, 'Session 3', horizontalalignment='center', fontweight='bold', color='black')

        plt.title(title, fontweight='bold')
        plt.xlabel('Trials')
        plt.ylabel(ylabel)
        plt.ylim(y_min - (y_max - y_min) * 0.1, text_y + (y_max - y_min) * 0.05)
        plt.xticks([])
        plt.grid(alpha=0.3)
        if col == 'Pupil_Change_From_Baseline_%':
            plt.legend(loc='lower right')
        plt.tight_layout()
        
        safe_col_name = col.lower().replace('%', 'pct')
        plot_path = os.path.join(output_dir, f"session_{safe_col_name}.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()
        print(f"Saved visualization to: {plot_path}")

    # Generate multi-line Quadrants Trend Plot
    quadrant_cols = ['Top-Left_%', 'Top-Right_%', 'Bottom-Left_%']
    if all(q in combined_df.columns for q in quadrant_cols):
        plt.figure(figsize=(14, 6))
        
        plt.plot(combined_df['Absolute_Trial'], combined_df['Top-Left_%'], marker='o', linewidth=2, color='#ff7f0e', label='Top-Left')
        plt.plot(combined_df['Absolute_Trial'], combined_df['Top-Right_%'], marker='s', linewidth=2, color='#1f77b4', label='Top-Right')
        plt.plot(combined_df['Absolute_Trial'], combined_df['Bottom-Left_%'], marker='^', linewidth=2, color='#d62728', label='Bottom-Left')

        plt.axvline(x=s1_end, color='red', linestyle='--', alpha=0.6, label='Session Boundary')
        plt.axvline(x=s2_end, color='red', linestyle='--', alpha=0.6)

        text_y = 105
        plt.text(s1_center, text_y, 'Session 1', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s2_center, text_y, 'Session 2', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s3_center, text_y, 'Session 3', horizontalalignment='center', fontweight='bold', color='black')

        plt.title('Attention Distribution Trends Across All Sessions (Active Quadrants Only)', fontsize=14, fontweight='bold')
        plt.xlabel('Trials')
        plt.ylabel('Percentage of Valid Fixation Time (%)')
        plt.ylim(-5, 115)
        plt.xticks([])
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(title='Quadrants', fontsize=10, title_fontsize=12, loc='upper right')
        plt.tight_layout()

        quad_plot_path = os.path.join(output_dir, "session_quadrants_trends.png")
        plt.savefig(quad_plot_path, dpi=150)
        plt.close()
        
    # Generate Multi-line Entropy Trend Plot
    entropy_cols = ['Gaze_Entropy_4Quad', 'Gaze_Entropy_10x10']
    if all(e in combined_df.columns for e in entropy_cols):
        plt.figure(figsize=(14, 6))
        
        plt.plot(combined_df['Absolute_Trial'], combined_df['Gaze_Entropy_4Quad'], marker='o', linewidth=2, label='4-Quadrant (AOI)')
        plt.plot(combined_df['Absolute_Trial'], combined_df['Gaze_Entropy_10x10'], marker='^', linewidth=2, label='10x10 Grid')

        plt.axvline(x=s1_end, color='red', linestyle='--', alpha=0.6, label='Session Boundary')
        plt.axvline(x=s2_end, color='red', linestyle='--', alpha=0.6)

        y_max = combined_df[entropy_cols].max().max()
        y_min = combined_df[entropy_cols].min().min()
        text_y = y_max + (y_max - y_min) * 0.1

        plt.text(s1_center, text_y, 'Session 1', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s2_center, text_y, 'Session 2', horizontalalignment='center', fontweight='bold', color='black')
        plt.text(s3_center, text_y, 'Session 3', horizontalalignment='center', fontweight='bold', color='black')

        plt.title('Gaze Entropy Trends Across All Sessions', fontsize=14, fontweight='bold')
        plt.xlabel('Trials')
        plt.ylabel('Entropy (Bits)')
        plt.ylim(y_min - (y_max - y_min) * 0.1, text_y + (y_max - y_min) * 0.05)
        plt.xticks([])
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(title='Entropy Type', fontsize=10, title_fontsize=12, loc='lower right')
        plt.tight_layout()

        ent_plot_path = os.path.join(output_dir, "session_gaze_entropy_trends.png")
        plt.savefig(ent_plot_path, dpi=150)
        plt.close()

    # Generate Vertical Box and Whisker Plots for Entropies
    for ent_col, title in zip(entropy_cols, ['4-Quadrant AOI Gaze Entropy', '10x10 Grid Gaze Entropy']):
        if ent_col in combined_df.columns and not combined_df[ent_col].isna().all():
            plt.figure(figsize=(8, 6))
            sns.boxplot(data=combined_df, x='Session_Short', y=ent_col, color='#1f77b4', width=0.5, showfliers=False)
            plt.title(f'{title} Distribution by Session', fontweight='bold')
            plt.xlabel('Session', fontweight='bold')
            plt.ylabel('Entropy (Bits)', fontweight='bold')
            plt.grid(axis='y', alpha=0.3)
            plt.tight_layout()
            
            box_plot_path = os.path.join(output_dir, f"session_boxplot_{ent_col.lower()}.png")
            plt.savefig(box_plot_path, dpi=150)
            plt.close()

    combined_csv_path = os.path.join(output_dir, "session_metrics_summary.csv")
    combined_df.to_csv(combined_csv_path, index=False)
    print(f"\nSaved combined tabular data to: {combined_csv_path}")

if __name__ == "__main__":
    main()