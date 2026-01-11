import os
import glob
import shutil
import pandas as pd
import imageio.v2 as imageio
import matplotlib.pyplot as plt

from tqdm import tqdm

from .utils import parse_frame_file, draw_frame


def analyze_experiment_results(path_to_input_data: str, path_to_output: str, 
                               scenarios: list[str], versions_to_analyze: dict[str, list[str]], 
                               experiments_to_analyze: list[str]) -> None:
    """
    Analyze experiment results for given scenarios, versions, and experiments.

    Args:
        path_to_input_data (str): Path to the input experiment results.
        path_to_output (str): Path to save the analysis results.
        scenarios (list[str]): List of scenarios to analyze.
        versions_to_analyze (dict[str, list[str]]): Versions to analyze for each scenario.
        experiments_to_analyze (list[str]): Experiments to analyze (subdirectories).

    Returns:
        None
    """
    # Run the analysis for each scenario
    for scenario in scenarios:
            for experiment in experiments_to_analyze:

                # Resolve timed execution results
                if experiment == "timed_execution_results":
                    # Get the output path
                    current_output_path = os.path.join(path_to_output, scenario, experiment)
                    current_input_path = os.path.join(path_to_input_data, scenario, experiment)

                    # For timed execution results, analyze all versions
                    versions = versions_to_analyze[scenario]

                    # Run the analysis
                    analyze_timed_execution_results(versions, current_input_path, current_output_path)

                if experiment == "correctness_results":
                    # Get the output path
                    current_output_path = os.path.join(path_to_output, scenario, experiment)
                    current_input_path = os.path.join(path_to_input_data, scenario, "boid_data")

                    # For timed execution results, analyze all versions
                    versions = versions_to_analyze[scenario]

                    # Run the analysis
                    analyze_correctness_results(versions, current_input_path, current_output_path)

                for version in versions_to_analyze[scenario]:
                    # Define current paths
                    current_input_path = os.path.join(path_to_input_data, scenario, experiment, version)
                    current_output_path = os.path.join(path_to_output, scenario, experiment, version)

                    if experiment == "boid_data":
                        # Run the analysis for boid data
                        analyze_boid_data(current_input_path, current_output_path)
                    elif experiment == "timed_execution_results":
                        # Timed execution results are handled above
                        continue
                    elif experiment == "correctness_results":
                        # Correctness results are handled above
                        continue
                    else:
                        print(f"Unknown experiment type: {experiment}")

def analyze_boid_data(input_path: str, output_path: str) -> None:
    """
    Analyze boid data from the given input path and save results to the output path.

    Args:
        input_path (str): Path to the boid data (directory of frame_XXXXX.txt files).
        output_path (str): Path to save the analysis results.

    Returns:
        None
    """
    if not os.path.exists(input_path):
        print(f"Input path {input_path} does not exist. Skipping boid data analysis.")
        return
    
    # List all directories in the input path
    subdirectories = [d for d in os.listdir(input_path) if os.path.isdir(os.path.join(input_path, d))]

    for subdir in tqdm(subdirectories, desc="Subdirectories"):
        # Adjust current input and output paths
        current_input_path = os.path.join(input_path, subdir)
        current_output_path = os.path.join(output_path, subdir)
        
        # Define video path
        video_path = os.path.join(current_output_path, "boids.mp4")

        # If it already exists, skip
        if os.path.exists(video_path):
            print(f"Video {video_path} already exists. Skipping rendering.")
            continue

        os.makedirs(current_output_path, exist_ok=True)

        frames_dir = os.path.join(current_output_path, "frames")
        os.makedirs(frames_dir, exist_ok=True)

        # Load and sort all frame files
        frame_files = sorted(
            glob.glob(os.path.join(current_input_path, "*.txt"))
        )

        if not frame_files:
            print("No boid frame files found. Nothing to analyze.")
            return

        rendered_images: list[str] = []

        # Parse + render each frame
        for idx, path in enumerate(tqdm(frame_files, desc="Frames")):
            frame_data = parse_frame_file(path)

            img_path = os.path.join(frames_dir, f"frame_{idx:05d}.png")

            if os.path.exists(img_path):
                rendered_images.append(img_path)
                continue

            draw_frame(frame_data, img_path)

            rendered_images.append(img_path)

        # Create video
        with imageio.get_writer(
            video_path,
            fps=30,
            codec="libx264",
            format="FFMPEG"
        ) as writer:
            for img_path in rendered_images:
                img = None
                try:
                    if os.path.getsize(img_path) > 0:
                        img = imageio.imread(img_path)
                except Exception:
                    img = None

                if img is None:
                    print(f"Skipping corrupted frame: {img_path}")
                    continue
                writer.append_data(img)
        shutil.rmtree(frames_dir)


    print("Analysis complete.")

def analyze_timed_execution_results(versions: list[str], input_path: str, output_path: str) -> None:
    """
    Analyze timed execution results for the given versions.

    Args:
        versions (list[str]): List of versions to analyze.
        input_path (str): Path to the timed execution results.
        output_path (str): Path to save the analysis results.

    Returns:
        None
    """
    # If the output path does not exist, create it
    os.makedirs(output_path, exist_ok=True)

    # Load all files from the input path
    version_data = {}
    min_num_boids = float('inf')
    max_num_boids = 0
    min_max_boids = float('inf')
    for version in versions:
        # Get the path
        version_file_path = os.path.join(input_path, f"{version}_timed_execution_results.csv")

        # Check whether the file exists
        if not os.path.exists(version_file_path):
            print(f"File {version_file_path} does not exist. Skipping.")
            continue
        
        # Load the data
        df = pd.read_csv(version_file_path)
        version_data[version] = df

        # There should be two columns 'BoidCount' 'AvgStepTimeMs'
        if 'BoidCount' not in df.columns or 'AvgStepTimeMs' not in df.columns:
            print(f"File {version_file_path} does not contain required columns. Skipping.")
            continue

        # Update min and max number of boids
        min_num_boids = min(min_num_boids, df['BoidCount'].min())
        max_num_boids = max(max_num_boids, df['BoidCount'].max())
        min_max_boids = min(min_max_boids, max_num_boids)

    # Plot the results (BoidCount on x-axis, AvgStepTimeMs on y-axis)        
    plt.figure()
    for version, df in version_data.items():
        # Limit to the minimum max number of boids across all versions
        df = df[df['BoidCount'] <= min_max_boids]
        plt.plot(df['BoidCount'], df['AvgStepTimeMs'], label=version)
    plt.xlabel('Number of Boids [-]')
    plt.ylabel('Execution time [ms]')
    plt.title('Execution Time vs Number of Boids for Different Versions')
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(output_path, f"timed_execution_results_{min_max_boids}_{max_num_boids}_{'_'.join(versions)}.png"))
    plt.close()
    print(f"Timed execution analysis complete. Results saved to {output_path}")

    # Save combined CSV
    combined_df = pd.DataFrame()
    for version, df in version_data.items():
        df = df[df['BoidCount'] <= min_max_boids]
        df = df.rename(columns={'AvgStepTimeMs': f'AvgStepTimeMs_{version}'})
        if combined_df.empty:
            combined_df = df[['BoidCount', f'AvgStepTimeMs_{version}']]
        else:
            combined_df = pd.merge(combined_df, df[['BoidCount', f'AvgStepTimeMs_{version}']], on='BoidCount', how='outer')
    combined_df = combined_df.sort_values(by='BoidCount')
    combined_csv_path = os.path.join(output_path, f"timed_execution_results_combined_{min_max_boids}_{max_num_boids}_{'_'.join(versions)}.csv")
    combined_df.to_csv(combined_csv_path, index=False)
    print(f"Combined CSV saved to {combined_csv_path}")
