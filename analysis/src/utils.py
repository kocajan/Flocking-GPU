import os
import imageio.v2 as imageio
import matplotlib.pyplot as plt

from typing import Any, Dict


def parse_frame_file(path: str) -> Dict[str, Any]:
    """
    Parse a boid frame data file and return its contents as a structured dictionary.

    Args:
        path (str): Path to the frame data file.

    Returns:
        Dict[str, Any]: Parsed frame data.
    """
    data = {
        "world": (0.0, 0.0, 0.0),
        "tick": 0,
        "boids": {}  # type -> list[(x,y,z,r)]
    }

    with open(path, "r") as f:
        for line in f:
            parts = line.strip().split()

            if not parts:
                continue

            if parts[0] == "world":
                # world X Y Z
                data["world"] = (
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                )

            elif parts[0] == "tick":
                data["tick"] = int(parts[1])

            elif parts[0] == "boid":
                # Example:
                # boid basic x 1078.32 y 901.866 z 925.556 vx ... r 4
                btype = parts[1]

                # Extract required numeric fields
                x = float(parts[3])
                y = float(parts[5])
                z = float(parts[7])

                # "r" is always last numeric field
                r = float(parts[-1])

                data["boids"].setdefault(btype, []).append((x, y, z, r))

    return data


def calculate_z_scale(world_z: float, z: float) -> float:
    """
    Calculate a scaling factor for the Z coordinate.

    Args:
        world_z (float): The maximum Z value of the world.
        z (float): The Z coordinate to scale.

    Returns:
        float: Scaled Z coordinate.
    """
    if (z < 0 or z > world_z):
        return 0.0
    return 1 - (z / world_z)


def draw_frame(frame: Dict[str, Any], out_path: str) -> None:
    """
    Draw a boid frame and save it as an image.

    Args:
        frame (Dict[str, Any]): Frame data to draw.
        out_path (str): Path to save the output image.

    Returns:
        None
    """
    world_x, world_y, world_z = frame["world"]

    plt.figure(figsize=(7, 4))
    ax = plt.gca()

    # Draw world bounds
    ax.set_xlim(0, world_x)
    ax.set_ylim(0, world_y)
    ax.set_aspect("equal", adjustable="box")

    ax.set_title(f"Tick {frame['tick']}")
    ax.axis("off")

    # Plot boids by type (XY projection)
    for btype, boids in frame["boids"].items():
        xs = [b[0] for b in boids]
        ys = [b[1] for b in boids]

        # Radius influences marker size a bit
        sizes = [b[3] * (0.3 + 0.7 * calculate_z_scale(world_z, b[2])) for b in boids]

        ax.scatter(xs, ys, s=sizes, label=btype)

    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close()
