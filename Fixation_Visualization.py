import os
import matplotlib.pyplot as plt
import pandas as pd


def plot_fixations():
    # Prompt the user for the file path
    file_path = input("Enter the path to the  fixation CSV file: ").strip()

    # Remove enclosing quotes if the user dragged and dropped the file into the terminal
    if file_path.startswith(('"', "'")) and file_path.endswith(('"', "'")):
        file_path = file_path[1:-1]

    # Verify if the file exists
    if not os.path.exists(file_path):
        print(f"Error: The file '{file_path}' does not exist.")
        return

    try:
        # Load the CSV file
        print("Reading file...")
        df = pd.read_csv(file_path)

        # Check for required columns
        required_cols = [
            "start timestamp [ns]",
            "fixation x [px]",
            "fixation y [px]",
        ]
        for col in required_cols:
            if col not in df.columns:
                print(f"Error: Missing required column '{col}' in the CSV.")
                return

        # Convert timestamps from nanoseconds to relative seconds from the start
        # This makes the X-axis much easier to read
        time_seconds = (
            df["start timestamp [ns]"] - df["start timestamp [ns]"].iloc[0]
        ) / 1e9

        print("Generating graphs...")

        # 1. Graph and save Fixation X over time
        plt.figure(figsize=(10, 5))
        plt.plot(
            time_seconds,
            df["fixation x [px]"],
            marker="o",
            linestyle="-",
            color="b",
            markersize=4,
        )
        plt.title("Fixation X over Time")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Fixation X [px]")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.tight_layout()
        x_output_name = "fixation_x_time.png"
        plt.savefig(x_output_name, dpi=300)
        plt.close()
        print(f"Saved: {x_output_name}")

        # 2. Graph and save Fixation Y over time
        plt.figure(figsize=(10, 5))
        plt.plot(
            time_seconds,
            df["fixation y [px]"],
            marker="o",
            linestyle="-",
            color="r",
            markersize=4,
        )
        plt.title("Fixation Y over Time")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Fixation Y [px]")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.tight_layout()
        y_output_name = "fixation_y_time.png"
        plt.savefig(y_output_name, dpi=300)
        plt.close()
        print(f"Saved: {y_output_name}")

        print("Both images have been saved to directory.")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    plot_fixations()