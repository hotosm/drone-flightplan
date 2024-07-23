import logging
import argparse

# Instantiate logger
log = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(
        description="Generate waypoints for drone missions."
    )
    parser.add_argument(
        "--project_geojson",
        required=True,
        type=str,
        help="The GeoJSON polygon representing the area of interest.",
    )
    parser.add_argument(
        "--altitude_above_ground_level",
        required=True,
        type=float,
        help="The flight altitude in meters.",
    )
    parser.add_argument(
        "--forward_overlap",
        type=float,
        default=70.0,
        help="The forward overlap in percentage.",
    )
    parser.add_argument(
        "--side_overlap",
        type=float,
        default=70.0,
        help="The side overlap in percentage.",
    )
    parser.add_argument(
        "--generate_each_points",
        action="store_true",
        help="Do you want waypoints or waylines.",
    )
    parser.add_argument(
        "--generate_3d", action="store_true", help="Do you want to generate 3D Imagery"
    )

    parser.add_argument(
        "--output_file_path",
        type=str,
        required=True,
        help="The output file path for the KMZ file.",
    )

    args = parser.parse_args()

    # with open(args.project_geojson, "r") as f:
    #     boundary = geojson.load(f)

    # features = boundary["features"]

    # output_file = create_waypoint(
    #     features,
    #     args.altitude_above_ground_level,
    #     args.forward_overlap,
    #     args.side_overlap,
    #     args.generate_each_points,
    #     args.generate_3d,
    #     args.output_file_path,
    # )

    log.info(f"Waypoints created and saved to: {args.output_file_path}")


if __name__ == "__main__":
    main()
