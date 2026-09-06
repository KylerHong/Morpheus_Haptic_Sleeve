import os
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import re
from matplotlib.ticker import MaxNLocator

def get_quadrant(x, y):
    """
    Maps coordinates to quadrants using strict 0-1 boundaries.
    """
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

def main():
    parser = argparse.ArgumentParser(description="Generate an overall trend graph of active quadrant attention from merged data.")
    parser.add_argument("--merged-dir", required=True, help="Path to the Merged_Data folder")
    args = parser.parse_args()

    merged_dir = args.merged_dir

    # Output directory logic
    parent_dir = os.path.dirname(os.path.normpath(merged_dir))
    output_dir = os.path.join(parent_dir, "Overall_Quadrant_Trends")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory ready at: {output_dir}\n")

    # Determine the fixations filename
    fixations_path = os.path.join(merged_dir, "merged_normalized_fixation_slice.csv")
        
    if not os.path.exists(fixations_path):
        print("Fixations data not found in Merged_Data folder.")
        return

    fix_df = pd.read_csv(fixations_path)

    if 'fixation x [normalized]' not in fix_df.columns or 'fixation y [normalized]' not in fix_df.columns or 'Source_Trial' not in fix_df.columns:
        print("Required columns (normalized coordinates or Source_Trial) not found in the merged fixations file.")
        return

    # Filter and sort trials
    all_trials = fix_df['Source_Trial'].unique()
    trials = [t for t in all_trials if "baseline" not in str(t).lower()]
    trials = sorted(trials, key=get_trial_number)

    print(f"Found {len(trials)} trial(s). Compiling trend data...\n")

    overall_trend_data = []

    for trial in trials:
        trial_num = get_trial_number(trial)
        trial_data = fix_df[fix_df['Source_Trial'] == trial].copy()

        # Categorize every fixation point
        trial_data['Calculated_Quadrant'] = trial_data.apply(
            lambda row: get_quadrant(row['fixation x [normalized]'], row['fixation y [normalized]']), axis=1
        )

        # Calculate time in seconds
        trial_data['duration_sec'] = trial_data['duration [ms]'] / 1000.0

        quadrant_totals = trial_data.groupby('Calculated_Quadrant')['duration_sec'].sum().reset_index()
        
        # Isolate only the three valid quadrants 
        valid_quadrants = ["Top-Left", "Top-Right", "Bottom-Left"]
        valid_data = quadrant_totals[quadrant_totals['Calculated_Quadrant'].isin(valid_quadrants)]
        
        # Calculate total time spent only in the active quadrants
        total_valid_fixation_time = valid_data['duration_sec'].sum()

        trial_dict = {
            'Trial_ID': trial,
            'Trial_Number': trial_num,
            'Total_Valid_Fixation_Sec': round(total_valid_fixation_time, 2)
        }

        # Calculate percentages based on the new isolated total
        for _, row in valid_data.iterrows():
            quad = row['Calculated_Quadrant']
            percentage = (row['duration_sec'] / total_valid_fixation_time) * 100 if total_valid_fixation_time > 0 else 0
            trial_dict[quad] = round(percentage, 2)

        overall_trend_data.append(trial_dict)

    if not overall_trend_data:
        print("No valid data could be processed.")
        return

    # Create the Tabulated Master DataFrame
    trend_df = pd.DataFrame(overall_trend_data)

    # Ensure the three target columns exist
    expected_columns = ["Top-Left", "Top-Right", "Bottom-Left"]
    for col in expected_columns:
        if col not in trend_df.columns:
            trend_df[col] = 0.0

    trend_df = trend_df.fillna(0.0)

    # Generate the 3-Line Overall Trend Graph
    plt.figure(figsize=(12, 7))

    plt.plot(trend_df['Trial_Number'], trend_df['Top-Left'], marker='o', linewidth=2, color='#ff7f0e', label='Top-Left')
    plt.plot(trend_df['Trial_Number'], trend_df['Top-Right'], marker='s', linewidth=2, color='#1f77b4', label='Top-Right')
    plt.plot(trend_df['Trial_Number'], trend_df['Bottom-Left'], marker='^', linewidth=2, color='#d62728', label='Bottom-Left')

    plt.title('Attention Distribution Trends Across Trials (Active Quadrants Only)', fontsize=16, fontweight='bold')
    plt.xlabel('Trial Number', fontsize=12, fontweight='bold')
    plt.ylabel('Percentage of Valid Fixation Time (%)', fontsize=12, fontweight='bold')
    
    plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
    
    plt.ylim(-5, 105) 
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend(title='Quadrants', fontsize=10, title_fontsize=12, loc='upper right')

    # Save Outputs
    output_plot = os.path.join(output_dir, "Overall_Active_Quadrant_Trends.png")
    output_csv = os.path.join(output_dir, "Overall_Active_Quadrant_Tabulated_Data.csv")

    plt.tight_layout()
    plt.savefig(output_plot, dpi=200)
    plt.close()

    # Export purely the relevant data
    cols_to_export = ['Trial_Number', 'Trial_ID', 'Total_Valid_Fixation_Sec', 'Top-Left', 'Top-Right', 'Bottom-Left']
    trend_df = trend_df[cols_to_export]
    trend_df.to_csv(output_csv, index=False)

    print(f"\nSuccess! Overall analysis complete.")
    print(f"  - Graph saved to: {output_plot}")
    print(f"  - Tabulated Data saved to: {output_csv}")

if __name__ == "__main__":
    main()