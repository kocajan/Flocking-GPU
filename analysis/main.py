import os
from src.analysis import analyze_experiment_results


def main() -> None:
    # Get path to this script's directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Define paths to input experiment results and output
    path_to_input_data = os.path.join(script_dir, "../data/results/")
    path_to_output = os.path.join(script_dir, "../data/analysis/")

    # Define the scenarios to be analyzed
    scenarios = [
        "plain",
        "plainWithObstacles",
        "plainWithObstaclesAndPredators",
        "default",
        "default3D",
    ]

    # Define versions to analyze for each scenario
    versions_to_analyze = {
        "plain": ["parallelNaive", "parallel"],
        "plainWithObstacles": ["parallelNaive", "parallel"],
        "plainWithObstaclesAndPredators": ["parallelNaive", "parallel"],
        "default": ["parallelNaive", "parallel"],
        "default3D": ["parallelNaive", "parallel"],
    }

    # Define experiments to analyze (their subdirectories)
    experiments_to_analyze = [
        "boid_data",
        "timed_execution_results",
    ]

    # Perform the analysis
    analyze_experiment_results(
        path_to_input_data=path_to_input_data,
        path_to_output=path_to_output,
        scenarios=scenarios,
        versions_to_analyze=versions_to_analyze,
        experiments_to_analyze=experiments_to_analyze,
    )


if __name__ == "__main__":
    main()
