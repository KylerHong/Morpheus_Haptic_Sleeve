import os
import pandas as pd

def calculate_trial_times():
    # Prompt the user for the file path
    file_path = input("Enter the path to the Merged_full_timeseries.csv file: ").strip()
    
    # Remove enclosing quotes if the user dragged and dropped the file into the terminal
    if file_path.startswith(('"', "'")) and file_path.endswith(('"', "'")):
        file_path = file_path[1:-1]
        
    if not os.path.exists(file_path):
        print(f"Error: The file '{file_path}' does not exist.")
        return
        
    try:
        print("Reading merged timeseries data...")
        df = pd.read_csv(file_path)
        
        # Verify columns exist
        if 'trial' not in df.columns:
            print("Error: The CSV does not contain a 'trial' column.")
            return
        if 'time_sec' not in df.columns:
            print("Error: The CSV does not contain a 'time_sec' column.")
            return
            
        # THE FIX: Force the 'trial' column to be strings. 
        # This prevents pandas from treating integer 1 and string '1' as separate groups.
        df['trial'] = df['trial'].astype(str)
        
        # Calculate duration for each trial
        trial_times = []
        
        # Using unique() preserves the original sorted order (b, 0, 1, 2, ...)
        trials = df['trial'].unique()
        
        for t in trials:
            trial_data = df[df['trial'] == t]
            
            # Duration is max time minus min time in the 'time_sec' column
            min_time = trial_data['time_sec'].min()
            max_time = trial_data['time_sec'].max()
            duration = max_time - min_time
                
            trial_times.append({
                'trial': t, 
                'Trial_Time': duration
            })
            
        # Convert our results into a new DataFrame
        summary_df = pd.DataFrame(trial_times)
        
        # Calculate the average Trial_Time
        avg_time = summary_df['Trial_Time'].mean()
        
        # Create a new row for the average and append it to the end
        avg_row = pd.DataFrame([{'trial': 'average', 'Trial_Time': avg_time}])
        summary_df = pd.concat([summary_df, avg_row], ignore_index=True)
        
        # Define output path (saves in the same directory as the input file)
        out_dir = os.path.dirname(file_path)
        out_file = os.path.join(out_dir, "Trial_Durations.csv")
        
        # Save to CSV
        summary_df.to_csv(out_file, index=False)
        
        print(f"\nSuccess! Saved trial durations to: {out_file}")
        print("\nPreview of the final data:")
        print(summary_df.head())
        print("...")
        print(summary_df.tail())
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    calculate_trial_times()