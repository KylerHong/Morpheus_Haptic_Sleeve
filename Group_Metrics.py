import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import glob
import numpy as np

def get_group_average_and_spread(folder_path):
    if not os.path.exists(folder_path):
        print(f"Error: Could not find folder {folder_path}")
        return None, 0, None, None

    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))
    if not csv_files:
        print(f"Error: No CSV files found in {folder_path}")
        return None, 0, None, None

    all_dfs = []
    session_info = None

    for f in csv_files:
        try:
            df = pd.read_csv(f)
            if session_info is None and 'Session' in df.columns and 'Absolute_Trial' in df.columns:
                session_info = df[['Absolute_Trial', 'Session']].copy()
            
            # Extract numeric data, but retain session identifiers for the boxplots if they exist
            numeric_cols = df.select_dtypes(include='number').copy()
            if 'Absolute_Trial' not in numeric_cols.columns:
                numeric_cols['Absolute_Trial'] = numeric_cols.index + 1
                
            if 'Session_Short' in df.columns:
                numeric_cols['Session_Short'] = df['Session_Short']
            elif 'Session' in df.columns:
                numeric_cols['Session_Short'] = df['Session'].str.replace('Session ', 'S')
                
            all_dfs.append(numeric_cols)
        except Exception as e:
            print(f"Failed to read {f}: {e}")

    if not all_dfs:
        return None, 0, None, None

    combined_df = pd.concat(all_dfs, ignore_index=True)
    grouped = combined_df.groupby('Absolute_Trial')
    
    # Enforce numeric_only=True to prevent errors when skipping over the Session_Short strings
    mean_df = grouped.mean(numeric_only=True).add_suffix('_mean').reset_index()
    std_df = grouped.std(numeric_only=True).add_suffix('_std').reset_index()
    
    mean_df = mean_df.rename(columns={'Absolute_Trial_mean': 'Absolute_Trial'})
    std_df = std_df.rename(columns={'Absolute_Trial_std': 'Absolute_Trial'})
    averaged_df = pd.merge(mean_df, std_df, on='Absolute_Trial')

    return averaged_df, len(csv_files), session_info, combined_df

def main():
    parser = argparse.ArgumentParser(description="Average and compare metrics with error spread.")
    parser.add_argument("--haptic-dir", required=True, help="Path to Haptic CSVs")
    parser.add_argument("--non-haptic-dir", required=True, help="Path to Non-Haptic CSVs")
    args = parser.parse_args()

    print("Loading Haptic group data...")
    haptic_df, h_n, h_sess_info, h_raw_df = get_group_average_and_spread(args.haptic_dir)
    print("Loading Non-Haptic group data...")
    non_haptic_df, nh_n, nh_sess_info, nh_raw_df = get_group_average_and_spread(args.non_haptic_dir)

    if haptic_df is None or non_haptic_df is None:
        print("Failed to process one or both groups. Exiting.")
        return

    print(f"\nSuccessfully loaded! Haptic (n={h_n}) vs Non-Haptic (n={nh_n})")

    parent_dir = os.path.dirname(os.path.normpath(args.haptic_dir))
    output_dir = os.path.join(parent_dir, "Group_Comparison_Output")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory ready at: {output_dir}\n")

    haptic_color = '#E6C200'
    non_haptic_color = '#1f77b4'

    session_boundaries = []
    session_centers = []
    sess_info = h_sess_info if h_sess_info is not None else nh_sess_info
    if sess_info is not None:
        sess_grouped = sess_info.groupby('Session')['Absolute_Trial'].agg(['min', 'max']).reset_index()
        for idx, row in sess_grouped.iterrows():
            session_centers.append({'label': row['Session'], 'center': (row['min'] + row['max']) / 2})
            if idx < len(sess_grouped) - 1:
                session_boundaries.append(row['max'] + 0.5)

    # Combined Metrics (2x2 Excluded, 4-Quad AOI and 10x10 Included)
    metrics_to_plot = {
        'Blink_Count': ('Average Blinks Across Groups', 'Mean Blinks ± 1 SD'),
        'Blink_Rate_BPM': ('Average Blink Rate Across Groups', 'Mean BPM ± 1 SD'),
        'Saccade_Count': ('Average Saccades Across Groups', 'Mean Saccades ± 1 SD'),
        'Avg_Saccade_Peak_Velocity': ('Average Saccade Peak Velocity Across Groups', 'Mean Velocity ± 1 SD'),
        'Duration_sec': ('Average Trial Durations Across Groups', 'Mean Duration (Seconds) ± 1 SD'),
        'Trimmed_Pupil_Mean_mm': ('Average Trimmed Pupil Diameter', 'Mean Diameter (mm) ± 1 SD'),
        'Pupil_Change_From_Baseline_%': ('Average Pupil Diameter % Change', 'Mean % Change ± 1 SD'),
        'Gaze_Entropy_4Quad': ('Average 4-Quadrant (AOI) Gaze Entropy Across Groups', 'Mean Entropy (Bits) ± 1 SD'),
        'Gaze_Entropy_10x10': ('Average 10x10 Grid Gaze Entropy Across Groups', 'Mean Entropy (Bits) ± 1 SD')
    }

    for base_col, (title, ylabel) in metrics_to_plot.items():
        col_mean = f"{base_col}_mean"
        col_std = f"{base_col}_std"
        
        if col_mean not in haptic_df.columns or col_mean not in non_haptic_df.columns:
            continue

        plt.figure(figsize=(14, 6))
        
        if base_col == 'Pupil_Change_From_Baseline_%':
            plt.axhline(0, color='black', linestyle='-', alpha=0.8, linewidth=1.5, label='Baseline (0%)')
        
        plt.plot(haptic_df['Absolute_Trial'], haptic_df[col_mean], marker='o', color=haptic_color, linewidth=2.5, label=f'Haptic (n={h_n})')
        plt.fill_between(haptic_df['Absolute_Trial'], 
                         haptic_df[col_mean] - haptic_df[col_std], 
                         haptic_df[col_mean] + haptic_df[col_std], 
                         color=haptic_color, alpha=0.25)
        
        plt.plot(non_haptic_df['Absolute_Trial'], non_haptic_df[col_mean], marker='X', color=non_haptic_color, linestyle='--', linewidth=2, label=f'Non-Haptic (n={nh_n})')
        plt.fill_between(non_haptic_df['Absolute_Trial'], 
                         non_haptic_df[col_mean] - non_haptic_df[col_std], 
                         non_haptic_df[col_mean] + non_haptic_df[col_std], 
                         color=non_haptic_color, alpha=0.15)
        
        for b in session_boundaries:
            plt.axvline(x=b, color='red', linestyle='--', alpha=0.6)

        y_max = max((haptic_df[col_mean] + haptic_df[col_std]).max(), (non_haptic_df[col_mean] + non_haptic_df[col_std]).max())
        y_min = min((haptic_df[col_mean] - haptic_df[col_std]).min(), (non_haptic_df[col_mean] - non_haptic_df[col_std]).min())
        
        if pd.isna(y_max) or pd.isna(y_min):
            continue
            
        if base_col in ['Blink_Count', 'Blink_Rate_BPM', 'Saccade_Count', 'Avg_Saccade_Peak_Velocity', 'Duration_sec']:
            y_min = max(0, y_min)
            
        text_y = y_max + (y_max - y_min) * 0.1

        for sc in session_centers:
            plt.text(sc['center'], text_y, sc['label'], horizontalalignment='center', fontweight='bold', color='black')

        plt.title(title, fontweight='bold')
        plt.xlabel('Absolute Trials')
        plt.ylabel(ylabel)
        plt.ylim(y_min - (y_max - y_min) * 0.1, text_y + (y_max - y_min) * 0.05)
        plt.grid(alpha=0.3)
        plt.legend(loc='lower right' if base_col == 'Pupil_Change_From_Baseline_%' else 'best')
        plt.tight_layout()
        
        safe_col_name = base_col.lower().replace('%', 'pct')
        plot_path = os.path.join(output_dir, f"GroupComparison_{safe_col_name}.png")
        plt.savefig(plot_path, dpi=150)
        plt.close()


    # Direct Comparison Graphs for Top-Left, Top-Right, Bottom-Left
    quadrants = ['Top-Left', 'Top-Right', 'Bottom-Left']
    
    for quad in quadrants:
        col_mean = f"{quad}_%_mean"
        col_std = f"{quad}_%_std"

        if col_mean not in haptic_df.columns or col_mean not in non_haptic_df.columns:
            continue

        plt.figure(figsize=(14, 6))
        
        # Haptic Plot
        plt.plot(haptic_df['Absolute_Trial'], haptic_df[col_mean], marker='o', color=haptic_color, linewidth=2.5, label=f'Haptic (n={h_n})')
        if col_std in haptic_df.columns:
            plt.fill_between(haptic_df['Absolute_Trial'], 
                             haptic_df[col_mean] - haptic_df[col_std], 
                             haptic_df[col_mean] + haptic_df[col_std], 
                             color=haptic_color, alpha=0.25)
        
        # Non-Haptic Plot
        plt.plot(non_haptic_df['Absolute_Trial'], non_haptic_df[col_mean], marker='X', color=non_haptic_color, linestyle='--', linewidth=2, label=f'Non-Haptic (n={nh_n})')
        if col_std in non_haptic_df.columns:
            plt.fill_between(non_haptic_df['Absolute_Trial'], 
                             non_haptic_df[col_mean] - non_haptic_df[col_std], 
                             non_haptic_df[col_mean] + non_haptic_df[col_std], 
                             color=non_haptic_color, alpha=0.15)
        
        for b in session_boundaries:
            plt.axvline(x=b, color='red', linestyle='--', alpha=0.6)

        y_max = max((haptic_df[col_mean] + haptic_df.get(col_std, 0)).max(), (non_haptic_df[col_mean] + non_haptic_df.get(col_std, 0)).max())
        y_min = min((haptic_df[col_mean] - haptic_df.get(col_std, 0)).min(), (non_haptic_df[col_mean] - non_haptic_df.get(col_std, 0)).min())
        
        if pd.isna(y_max): y_max = 100
        if pd.isna(y_min): y_min = 0
            
        text_y = y_max + (y_max - y_min) * 0.1

        for sc in session_centers:
            plt.text(sc['center'], text_y, sc['label'], horizontalalignment='center', fontweight='bold', color='black')

        plt.title(f"Direct Group Comparison: {quad} Quadrant", fontweight='bold')
        plt.xlabel('Absolute Trials')
        plt.ylabel(f'{quad} Valid Fixation Time (%) ± 1 SD')
        plt.ylim(max(-5, y_min - (y_max - y_min) * 0.1), text_y + (y_max - y_min) * 0.05)
        plt.grid(alpha=0.3)
        plt.legend(loc='best')
        plt.tight_layout()
        
        direct_comp_path = os.path.join(output_dir, f"GroupComparison_{quad}_Direct.png")
        plt.savefig(direct_comp_path, dpi=150)
        plt.close()


    # Side-by-Side Quadrant Graphs (3 lines per graph)
    quad_colors = {
        'Top-Left': '#1f77b4',     # Blue
        'Top-Right': '#ff7f0e',    # Orange
        'Bottom-Left': '#2ca02c'   # Green
    }
    quad_markers = {
        'Top-Left': 'o',
        'Top-Right': 's',
        'Bottom-Left': '^'
    }

    fig, axes = plt.subplots(1, 2, figsize=(18, 6), sharey=True)

    groups = [
        (axes[0], haptic_df, f'Haptic Group (n={h_n})'),
        (axes[1], non_haptic_df, f'Non-Haptic Group (n={nh_n})')
    ]

    for ax, df, group_title in groups:
        for quad in quadrants:
            col_mean = f"{quad}_%_mean"
            col_std = f"{quad}_%_std"

            if col_mean in df.columns:
                ax.plot(
                    df['Absolute_Trial'], 
                    df[col_mean], 
                    marker=quad_markers[quad], 
                    color=quad_colors[quad], 
                    linewidth=2, 
                    label=quad
                )
                if col_std in df.columns:
                    ax.fill_between(
                        df['Absolute_Trial'], 
                        df[col_mean] - df[col_std], 
                        df[col_mean] + df[col_std], 
                        color=quad_colors[quad], 
                        alpha=0.15
                    )

        for b in session_boundaries:
            ax.axvline(x=b, color='red', linestyle='--', alpha=0.6)
            
        for sc in session_centers:
            ax.text(sc['center'], 110, sc['label'], horizontalalignment='center', fontweight='bold', color='black')

        ax.set_title(group_title, fontsize=13, fontweight='bold')
        ax.set_xlabel('Absolute Trials')
        ax.set_ylim(-10, 120)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(title='Quadrant', fontsize=10, loc='upper right')

    axes[0].set_ylabel('Percentage of Valid Fixation Time (%) ± 1 SD')
    fig.suptitle('Group Attention Distribution Across Quadrants', fontsize=15, fontweight='bold')
    plt.tight_layout()

    quad_plot_path = os.path.join(output_dir, "GroupComparison_Quadrant_Distribution_SideBySide.png")
    plt.savefig(quad_plot_path, dpi=150, bbox_inches='tight')
    plt.close()


    # Box and Whisker Plots for Entropies (Haptic vs Non-Haptic)
    h_raw_df['Group'] = 'Haptic'
    nh_raw_df['Group'] = 'Non-Haptic'
    combined_raw_df = pd.concat([h_raw_df, nh_raw_df], ignore_index=True)

    entropy_cols = ['Gaze_Entropy_4Quad', 'Gaze_Entropy_10x10']
    entropy_titles = ['4-Quadrant AOI Gaze Entropy', '10x10 Grid Gaze Entropy']

    for ent_col, title in zip(entropy_cols, entropy_titles):
        if ent_col in combined_raw_df.columns and not combined_raw_df[ent_col].isna().all():
            plt.figure(figsize=(10, 6))
            
            # If sessions are populated, group by Session and Group (like reference picture)
            if 'Session_Short' in combined_raw_df.columns and not combined_raw_df['Session_Short'].isna().all():
                sns.boxplot(
                    data=combined_raw_df, 
                    x='Session_Short', 
                    y=ent_col, 
                    hue='Group', 
                    palette={'Haptic': haptic_color, 'Non-Haptic': non_haptic_color}, 
                    width=0.6, 
                    showfliers=False
                )
                plt.xlabel('Session', fontweight='bold')
                plt.legend(title='Group', loc='best')
            else:
                # Fallback to direct Group vs Group
                sns.boxplot(
                    data=combined_raw_df, 
                    x='Group', 
                    y=ent_col, 
                    hue='Group',
                    palette={'Haptic': haptic_color, 'Non-Haptic': non_haptic_color}, 
                    width=0.5, 
                    showfliers=False,
                    legend=False
                )
                plt.xlabel('Group', fontweight='bold')
                
                # Create hardcoded legend correctly binding the color variables
                import matplotlib.patches as mpatches
                haptic_patch = mpatches.Patch(color=haptic_color, label='Haptic')
                non_haptic_patch = mpatches.Patch(color=non_haptic_color, label='Non-Haptic')
                plt.legend(handles=[haptic_patch, non_haptic_patch], title='Group', loc='best')
                
            plt.title(f'{title} Distribution: Haptic vs Non-Haptic', fontweight='bold')
            plt.ylabel('Entropy (Bits)', fontweight='bold')
            plt.grid(axis='y', alpha=0.3)
            plt.tight_layout()
            
            box_plot_path = os.path.join(output_dir, f"GroupComparison_boxplot_{ent_col.lower()}.png")
            plt.savefig(box_plot_path, dpi=150)
            plt.close()



    # Final Image - Average Across Quadrants Summary (Bottom-Right blank)
    four_quads = ['Top-Left', 'Top-Right', 'Bottom-Left', 'Bottom-Right']
    
    quad_means = {}
    for q in four_quads:
        col = f"{q}_%_mean"
        h_val = haptic_df[col].mean() if col in haptic_df.columns else np.nan
        nh_val = non_haptic_df[col].mean() if col in non_haptic_df.columns else np.nan
        quad_means[q] = {'Haptic': h_val, 'Non-Haptic': nh_val}

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.axis('off')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)

    ax.axhline(0.5, color='black', linewidth=4)
    ax.axvline(0.5, color='black', linewidth=4)
    
    positions = {
        'Top-Left': (0.25, 0.75),
        'Top-Right': (0.75, 0.75),
        'Bottom-Left': (0.25, 0.25),
        'Bottom-Right': (0.75, 0.25)
    }

    for q in four_quads:
        if q == 'Bottom-Right':
            continue

        x, y = positions[q]
        h_val = quad_means[q]['Haptic']
        nh_val = quad_means[q]['Non-Haptic']
        
        ax.text(x, y + 0.12, q, fontsize=18, fontweight='bold', ha='center', va='center')
        
        h_text = f"Haptic: {h_val:.1f}%" if pd.notna(h_val) else "Haptic: N/A"
        nh_text = f"Non-Haptic: {nh_val:.1f}%" if pd.notna(nh_val) else "Non-Haptic: N/A"
        
        ax.text(x, y + 0.02, h_text, fontsize=14, fontweight='bold', color='black', 
                ha='center', va='center', 
                bbox=dict(facecolor=haptic_color, edgecolor='black', boxstyle='round,pad=0.4', alpha=0.9))
        
        ax.text(x, y - 0.08, nh_text, fontsize=14, fontweight='bold', color='white', 
                ha='center', va='center', 
                bbox=dict(facecolor=non_haptic_color, edgecolor='black', boxstyle='round,pad=0.4', alpha=0.9))

    fig.suptitle('Average Across Quadrants (Across All Sessions)', fontsize=20, fontweight='bold', y=0.97)
    plt.tight_layout()
    
    final_summary_path = os.path.join(output_dir, "GroupComparison_Average_Across_Quadrants.png")
    plt.savefig(final_summary_path, dpi=150)
    plt.close()

    # Export Data
    merged_csv = pd.merge(haptic_df, non_haptic_df, on='Absolute_Trial', suffixes=('_Haptic', '_NonHaptic'))
    combined_csv_path = os.path.join(output_dir, "Group_Comparison_Averages.csv")
    merged_csv.to_csv(combined_csv_path, index=False)
    
    print(f"Saved side-by-side quadrant plots, direct comparison plots, box plots, final summary, and tabular data to: {output_dir}")
    print("Process Complete!")

if __name__ == "__main__":
    main()