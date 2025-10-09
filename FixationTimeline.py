import pandas as pd
import sys
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import numpy as np


def get_quadrant(x, y):
    if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return None

    if y < 0.5:
        if x < 0.5:
            return "Top-Left"
        else:
            return "Top-Right"
    else:
        if x < 0.5:
            return "Bottom-Left"
        else:
            return "Bottom-Right"


def generate_summary_stats(merged_df, output_dir):
    summary_cols = {
        'total_duration_s': ('duration_sec', 'sum'),
        'fixation_count': ('duration_sec', 'count'),
        'mean_fixation_duration_s': ('duration_sec', 'mean')
    }
    if 'mean_head_velocity' in merged_df.columns:
        summary_cols['mean_head_velocity_deg_s'] = ('mean_head_velocity', 'mean')
    if 'mean_head_yaw' in merged_df.columns:
        summary_cols['mean_head_yaw_deg'] = ('mean_head_yaw', 'mean')

    summary = merged_df.groupby('quadrant').agg(**summary_cols).reset_index()

    category_order = ["Top-Left", "Top-Right", "Bottom-Left", "Bottom-Right", "Off-Surface"]
    summary['quadrant'] = pd.Categorical(summary['quadrant'], categories=category_order, ordered=True)
    summary = summary.sort_values('quadrant')

    summary_filepath = os.path.join(output_dir, "summary_statistics.csv")
    summary.to_csv(summary_filepath, index=False)

    fig, ax = plt.subplots(figsize=(10, 6))
    summary.plot(
        kind='bar', x='quadrant', y='mean_fixation_duration_s',
        ax=ax, legend=None, rot=0
    )
    ax.set_title('Average Fixation Duration per Area of Attention')
    ax.set_xlabel('Area of Attention')
    ax.set_ylabel('Average Duration (s)')
    ax.grid(axis='y', linestyle='--', alpha=0.7)

    max_height = summary['mean_fixation_duration_s'].max()
    ax.set_ylim(top=max_height * 1.2)

    for p in ax.patches:
        ax.annotate(f'{p.get_height():.2f}s',
                    (p.get_x() + p.get_width() / 2., p.get_height()),
                    ha='center', va='center', xytext=(0, 9), textcoords='offset points')

    summary_plot_path = os.path.join(output_dir, "summary_plot.png")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(summary_plot_path)
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Create a comprehensive timeline visualization for cognitive load analysis."
    )
    parser.add_argument("--main-dir", required=True, help="Path to the main recording directory.")
    parser.add_argument("--surface-dir", required=True, help="Path to the surface data directory.")
    args = parser.parse_args()

    fixations_file = os.path.join(args.main_dir, "fixations.csv")
    surface_fixations_file = os.path.join(args.surface_dir, "fixations.csv")
    eye_states_file = os.path.join(args.main_dir, "3d_eye_states.csv")
    blinks_file = os.path.join(args.main_dir, "blinks.csv")
    saccades_file = os.path.join(args.main_dir, "saccades.csv")
    imu_file = os.path.join(args.main_dir, "imu.csv")

    try:
        main_df = pd.read_csv(fixations_file)
        surface_df = pd.read_csv(surface_fixations_file)
        eye_df = pd.read_csv(eye_states_file)
        blinks_df = pd.read_csv(blinks_file) if os.path.exists(blinks_file) else None
        saccades_df = pd.read_csv(saccades_file) if os.path.exists(saccades_file) else None
        imu_df = pd.read_csv(imu_file) if os.path.exists(imu_file) else None
    except FileNotFoundError as e:
        sys.exit(f"Error: A required file was not found. Missing file: {e.filename}")
    except Exception as e:
        sys.exit(f"Error reading one of the files: {e}")

    surface_df['quadrant'] = surface_df.apply(
        lambda r: get_quadrant(r['fixation x [normalized]'], r['fixation y [normalized]']),
        axis=1
    )
    surface_quadrants = surface_df[['fixation id', 'quadrant']]
    merged_df = pd.merge(main_df, surface_quadrants, on='fixation id', how='left')
    merged_df['quadrant'] = merged_df['quadrant'].fillna('Off-Surface')
    merged_df.dropna(subset=['quadrant'], inplace=True)

    start_time_ns = merged_df['start timestamp [ns]'].iloc[0]
    merged_df['start_time_sec'] = (merged_df['start timestamp [ns]'] - start_time_ns) * 1e-9
    merged_df['end_time_sec'] = (merged_df['end timestamp [ns]'] - start_time_ns) * 1e-9
    merged_df['duration_sec'] = merged_df['duration [ms]'] * 1e-3

    eye_df['pupil_diameter_avg'] = eye_df[['pupil diameter left [mm]', 'pupil diameter right [mm]']].mean(axis=1)
    eye_df['eyelid_aperture_avg'] = eye_df[['eyelid aperture left [mm]', 'eyelid aperture right [mm]']].mean(axis=1)
    eye_df['time_sec'] = (eye_df['timestamp [ns]'] - start_time_ns) * 1e-9

    if blinks_df is not None and not blinks_df.empty:
        blinks_df['start_time_sec'] = (blinks_df['start timestamp [ns]'] - start_time_ns) * 1e-9
        blinks_df['end_time_sec'] = (blinks_df['end timestamp [ns]'] - start_time_ns) * 1e-9

    velocity_col = 'peak velocity [px/s]'
    saccades_available = False
    if saccades_df is not None and not saccades_df.empty and velocity_col in saccades_df.columns:
        saccades_df['start_time_sec'] = (saccades_df['start timestamp [ns]'] - start_time_ns) * 1e-9
        saccades_available = True

    head_velocity_available = False
    head_yaw_available = False
    if imu_df is not None and not imu_df.empty:
        imu_df['time_sec'] = (imu_df['timestamp [ns]'] - start_time_ns) * 1e-9

        gyro_cols = ['gyro x [deg/s]', 'gyro y [deg/s]', 'gyro z [deg/s]']
        if all(col in imu_df.columns for col in gyro_cols):
            imu_df['head_angular_velocity'] = np.linalg.norm(imu_df[gyro_cols].values, axis=1)
            head_velocity_available = True

        if 'yaw [deg]' in imu_df.columns:
            initial_yaw = imu_df['yaw [deg]'].iloc[0]
            imu_df['relative_yaw_deg'] = imu_df['yaw [deg]'] - initial_yaw
            imu_df['relative_yaw_deg'] = imu_df['relative_yaw_deg'].apply(
                lambda x: x + 360 if x < -180 else (x - 360 if x > 180 else x)
            )
            head_yaw_available = True

        if head_velocity_available or head_yaw_available:
            imu_df_indexed = imu_df.set_index('time_sec')
            if head_velocity_available:
                avg_velocities = []
                for _, row in merged_df.iterrows():
                    vel_slice = imu_df_indexed.loc[row['start_time_sec']:row['end_time_sec']]
                    avg_velocities.append(vel_slice['head_angular_velocity'].mean())
                merged_df['mean_head_velocity'] = avg_velocities
            if head_yaw_available:
                avg_yaws = []
                for _, row in merged_df.iterrows():
                    yaw_slice = imu_df_indexed.loc[row['start_time_sec']:row['end_time_sec']]
                    avg_yaws.append(yaw_slice['relative_yaw_deg'].mean())
                merged_df['mean_head_yaw'] = avg_yaws

    plots_to_create = {
        'aperture': True, 'pupil': True, 'fix_duration': True,
        'saccades': saccades_available,
        'blinks': blinks_df is not None and not blinks_df.empty,
        'head_velocity': head_velocity_available,
        'head_yaw': head_yaw_available
    }
    num_plots = 1 + sum(plots_to_create.values())
    output_dir = os.path.dirname(args.main_dir)

    fig, axs = plt.subplots(num_plots, 1, figsize=(15, 1.675 * num_plots), sharex=True)
    fig.suptitle("Comprehensive Cognitive Load Analysis", fontsize=16)

    categories = {
        "Top-Right": {'y': 4, 'color': '#1f77b4'}, "Top-Left": {'y': 3, 'color': '#ff7f0e'},
        "Bottom-Right": {'y': 2, 'color': '#2ca02c'}, "Bottom-Left": {'y': 1, 'color': '#d62728'},
        "Off-Surface": {'y': 0, 'color': '#7f7f7f'}
    }
    merged_df['y_pos'] = merged_df['quadrant'].apply(lambda q: categories.get(q, {}).get('y'))
    merged_df['color'] = merged_df['quadrant'].apply(lambda q: categories.get(q, {}).get('color'))

    axs[0].barh(
        y=merged_df['y_pos'], width=merged_df['duration_sec'],
        left=merged_df['start_time_sec'], color=merged_df['color'], height=0.7
    )
    axs[0].set_yticks(ticks=[cat['y'] for cat in categories.values()])
    axs[0].set_yticklabels(labels=categories.keys())
    axs[0].set_ylabel("Area of Attention")
    axs[0].set_title("Timeline of Visual Attention")
    axs[0].grid(axis='x', linestyle='--', alpha=0.6)
    legend_patches = [mpatches.Patch(color=cat['color'], label=name) for name, cat in categories.items()]
    axs[0].legend(handles=legend_patches, bbox_to_anchor=(1.02, 1.02), loc='upper left')

    plot_idx = 1
    for _, row in merged_df.iterrows():
        if pd.notna(row['color']):
            for i in range(1, num_plots):
                axs[i].axvspan(
                    xmin=row['start_time_sec'],
                    xmax=row['start_time_sec'] + row['duration_sec'],
                    color=row['color'], alpha=0.15, zorder=0
                )

    axs[plot_idx].plot(eye_df['time_sec'], eye_df['eyelid_aperture_avg'], color='purple', linewidth=1.5, zorder=1)
    axs[plot_idx].set_ylabel("Aperture (mm)")
    axs[plot_idx].set_title("Average Eyelid Aperture")
    axs[plot_idx].grid(linestyle='--', alpha=0.6)
    plot_idx += 1

    axs[plot_idx].plot(eye_df['time_sec'], eye_df['pupil_diameter_avg'], color='teal', linewidth=1.5, zorder=1)
    axs[plot_idx].set_ylabel("Diameter (mm)")
    axs[plot_idx].set_title("Average Pupil Diameter")
    axs[plot_idx].grid(linestyle='--', alpha=0.6)
    plot_idx += 1

    axs[plot_idx].stem(merged_df['start_time_sec'], merged_df['duration_sec'], linefmt='grey', markerfmt='o',
                       basefmt=" ")
    axs[plot_idx].set_ylabel("Duration (s)")
    axs[plot_idx].set_title("Fixation Duration")
    axs[plot_idx].grid(linestyle='--', alpha=0.6)
    plot_idx += 1

    if plots_to_create['saccades']:
        axs[plot_idx].plot(saccades_df['start_time_sec'], saccades_df[velocity_col], color='darkorange', marker='.',
                           linestyle='', zorder=1)
        axs[plot_idx].set_ylabel("Velocity (px/s)")
        axs[plot_idx].set_title("Peak Saccadic Velocity")
        axs[plot_idx].grid(linestyle='--', alpha=0.6)
        plot_idx += 1

    if plots_to_create['head_velocity']:
        axs[plot_idx].plot(imu_df['time_sec'], imu_df['head_angular_velocity'], color='black', linewidth=1.5, zorder=1)
        axs[plot_idx].set_ylabel("Velocity (deg/s)")
        axs[plot_idx].set_title("Head Angular Velocity")
        axs[plot_idx].grid(linestyle='--', alpha=0.6)
        plot_idx += 1

    if plots_to_create['head_yaw']:
        axs[plot_idx].plot(imu_df['time_sec'], imu_df['relative_yaw_deg'], color='green', linewidth=1.5, zorder=1)
        axs[plot_idx].axhline(0, color='black', linestyle='--', linewidth=1)
        axs[plot_idx].set_ylabel("Degrees\nfrom Start")
        axs[plot_idx].set_title("Head Yaw (Relative)")
        axs[plot_idx].grid(linestyle='--', alpha=0.6)
        plot_idx += 1

    if plots_to_create['blinks']:
        for _, blink_row in blinks_df.iterrows():
            axs[plot_idx].axvspan(
                xmin=blink_row['start_time_sec'],
                xmax=blink_row['end_time_sec'],
                color='crimson', alpha=0.5, label='Blink Event'
            )
        axs[plot_idx].set_ylim(0, 1)
        axs[plot_idx].set_yticks([])
        axs[plot_idx].set_ylabel("Blink Events")
        axs[plot_idx].set_title("Timeline of Blink Events")
        axs[plot_idx].grid(linestyle='--', alpha=0.6)

        handles, labels = axs[plot_idx].get_legend_handles_labels()
        if handles:
            axs[plot_idx].legend([handles[0]], [labels[0]], loc='upper right')

    axs[-1].set_xlabel("Time (seconds)")
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    timeline_plot_path = os.path.join(output_dir, "timeline_plot.png")
    fig.savefig(timeline_plot_path)

    plt.show()

    generate_summary_stats(merged_df, output_dir)


if __name__ == "__main__":
    main()