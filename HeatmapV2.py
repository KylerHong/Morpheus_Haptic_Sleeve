import os
import glob
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import re

HAPTIC_DIRS = [
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SHF01',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SHF03',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SHF05',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SH11',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SH13'
]

NON_HAPTIC_DIRS = [
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SNFH06',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SNFH07',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SNHF08',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SNH09',
    '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/SNH10'
]

EXCLUDED_TRIALS = {
    1: [0, 1],
    2: [30, 31, 37, 38],
    3: [30, 31, 32, 33, 34]
}

MAX_POINTS_PER_PLOT = 30000

# Helper Functions
def get_trial_number(trial_name):
    """Extracts the first number found in the trial string."""
    match = re.search(r'\d+', str(trial_name))
    return int(match.group()) if match else 99999

def get_group_session_data(subject_dirs, session_num):
    """Aggregates all valid fixation coordinates for a specific group and session, ignoring excluded trials."""
    all_x = []
    all_y = []
    exclusions = EXCLUDED_TRIALS.get(session_num, [])

    for subj_dir in subject_dirs:
        # Search for Session folder using wildcards like run_all.py
        search_pattern = os.path.join(subj_dir, f"*_Session{session_num}")
        matching_folders = glob.glob(search_pattern)
        
        if not matching_folders:
            continue
            
        session_folder = matching_folders[0]
        fix_csv_path = os.path.join(session_folder, "Merged_Data", "merged_normalized_fixation_slice.csv")
        
        if os.path.exists(fix_csv_path):
            try:
                df = pd.read_csv(fix_csv_path)
                if 'Source_Trial' not in df.columns or 'fixation x [normalized]' not in df.columns:
                    continue
                
                # Drop baseline trials
                df = df[~df['Source_Trial'].astype(str).str.lower().str.contains("baseline")]
                
                # Drop explicitly excluded trials
                df['Trial_Num'] = df['Source_Trial'].apply(get_trial_number)
                df = df[~df['Trial_Num'].isin(exclusions)]
                
                # Ensure coordinates are within 0-1 bounds
                df = df[(df['fixation x [normalized]'] >= 0) & (df['fixation x [normalized]'] <= 1) & 
                        (df['fixation y [normalized]'] >= 0) & (df['fixation y [normalized]'] <= 1)]
                
                all_x.extend(df['fixation x [normalized]'].tolist())
                all_y.extend(df['fixation y [normalized]'].tolist())
            except Exception as e:
                print(f"Failed reading {fix_csv_path}: {e}")

    df_combined = pd.DataFrame({'x': all_x, 'y': all_y})
    
    # Downsample if too large
    if len(df_combined) > MAX_POINTS_PER_PLOT:
        stride = len(df_combined) // MAX_POINTS_PER_PLOT
        df_combined = df_combined.iloc[::stride]
        
    return df_combined

# MAIN EXECUTION
def main():
    print("Aggregating data and generating Group Heatmaps...")
    
    # Setup Figure (2 rows, 3 columns)
    fig, axes = plt.subplots(2, 3, figsize=(16, 10), gridspec_kw={'wspace': 0.05, 'hspace': 0.1})
    
    groups = [
        ("Haptic\ngroup", HAPTIC_DIRS),
        ("Non-haptic\ngroup", NON_HAPTIC_DIRS)
    ]
    
    # Define weather colormap matching Metrics_7.py
    colors = ['#4b0082', '#0000ff', '#00ffff', '#00ff00', '#ffff00', '#ff8c00', '#ff0000']
    weather_cmap = mcolors.LinearSegmentedColormap.from_list("weather_map", colors, N=256)

    for row_idx, (group_name, directories) in enumerate(groups):
        for col_idx, sess_num in enumerate([1, 2, 3]):
            ax = axes[row_idx, col_idx]
            print(f"Processing {group_name.replace(chr(10), ' ')} - Session {sess_num}...")
            
            df = get_group_session_data(directories, sess_num)
            
            if len(df) >= 10:
                # Flip Y for screen orientation
                sns.kdeplot(x=df['x'], y=1 - df['y'], fill=True, cmap=weather_cmap, alpha=0.8, levels=60, thresh=0.05, ax=ax)
            
            # Format Axis
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.axis('off')
            
            # Add Crosshairs and Border
            ax.axvline(x=0.5, color='black', linestyle='-', linewidth=2) 
            ax.axhline(y=0.5, color='black', linestyle='-', linewidth=2)
            ax.add_patch(plt.Rectangle((0, 0), 1, 1, fill=False, edgecolor='black', lw=3))
            
            # Add Viewport Labels (Front, Side, Top)
            ax.text(0.02, 0.98, 'Front', color='white', backgroundcolor='black', fontsize=12, fontweight='bold', va='top', ha='left')
            ax.text(0.98, 0.98, 'Side', color='white', backgroundcolor='black', fontsize=12, fontweight='bold', va='top', ha='right')
            ax.text(0.02, 0.02, 'Top', color='white', backgroundcolor='black', fontsize=12, fontweight='bold', va='bottom', ha='left')

    # Add Shared Colorbar to the right side of the figure
    sm = plt.cm.ScalarMappable(cmap=weather_cmap, norm=plt.Normalize(vmin=0, vmax=1))
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7]) 
    fig.colorbar(sm, cax=cbar_ax, ticks=[0, 1])

    # Save Output
    output_dir = '/Users/tonysaldana/Library/CloudStorage/Box-Box/Haptic Sleeve/1 - Subject Data/HvsNH'
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "Group_Heatmap_Comparison.png")
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"\nHeatmap grid successfully saved to: {output_path}")

if __name__ == "__main__":
    main()