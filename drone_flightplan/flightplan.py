import logging
import argparse
import geojson

from drone_flightplan.waypoints import create_waypoint, calculate_parameters
from drone_flightplan.create_wpml import create_xml


# Instantiate logger
log = logging.getLogger(__name__)


def generate_flightplan(
    project_area,
    agl,
    forward_overlap=70.0,
    side_overlap=70.0,
    generate_each_points=False,
    generate_3d=False,
    output_file_path="/tmp",
):
    """
    Generate flight plan for drone missions.

    Parameters
        :project_area: Project area as geojson polygon.
        :agl: Altitude above ground level in meters.
        :forward_overlap: Forward overlap in percentage.
        :side_overlap : Side overlap in percentage.
        :generate_each_points: Generate waypoints or waylines.
        :generate_3d: Generate 3D imagery.
        :output_file_path: Output file path for the KMZ file.

    Returns
        Generates a flightplan kmz file in the output_file_path

    """
    parameters = calculate_parameters(agl, forward_overlap, side_overlap)

    waypoints = create_waypoint(
        project_area,
        agl,
        forward_overlap,
        side_overlap,
        generate_each_points,
        generate_3d,
    )

    def add_speed_and_agl(placemark):
        new_placemark = list(placemark.values())
        new_placemark[0] = f"{new_placemark[0][0]},{new_placemark[0][1]}"
        new_placemark.insert(1, str(agl))
        new_placemark.insert(2, str(parameters["ground_speed"]))
        return new_placemark

    updated_waypoints = list(map(add_speed_and_agl, waypoints))

    # generate wpml file in output_file_path
    output_file = create_xml(updated_waypoints, "goHome", agl, output_file_path)

    return output_file


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

    with open(args.project_geojson, "r") as f:
        boundary = geojson.load(f)

    features = boundary["features"]

    generate_flightplan(
        features,
        args.altitude_above_ground_level,
        args.forward_overlap,
        args.side_overlap,
        args.generate_each_points,
        args.generate_3d,
        args.output_file_path,
    )
    log.info(f"Waypoints created and saved to: {args.output_file_path}")


if __name__ == "__main__":
    main()
