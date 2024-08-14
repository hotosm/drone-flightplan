import json
import argparse
import geojson
from typing import Union
from geojson import FeatureCollection


def create_placemarks(
    waypoints_geojson: Union[str, FeatureCollection, dict], parameters: dict
):
    """
    Arguments:
        waypoints_geojson: The waypoint coordinates to be included in the flightplan mission
        parameters: The drone flight parameters in a json
    Returns:
        Geojson object.
    """

    ground_speed = parameters["ground_speed"]
    agl = parameters["altitude_above_ground_level"]

    try:
        first_point = waypoints_geojson["features"][0]
        base_elevation = first_point["geometry"]["coordinates"][2]
    except IndexError:
        base_elevation = 0

    for feature in waypoints_geojson["features"]:
        coords = feature["geometry"]["coordinates"]
        try:
            elevation = coords[2]
            difference_in_elevation = base_elevation - elevation
            altitude = agl + difference_in_elevation
            coords[2] = altitude
        except IndexError:
            altitude = agl
            coords.append(altitude)

        feature["properties"]["speed"] = ground_speed

    return waypoints_geojson


def main(args_list: list[str] | None = None):
    def json_to_dict(value):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            raise argparse.ArgumentTypeError(f"Invalid JSON string: {value}")

    parser = argparse.ArgumentParser(
        description="Generate placemark data for drone missions."
    )
    parser.add_argument(
        "--waypoints_geojson",
        required=True,
        type=str,
        help="The waypoint coordinates to be included in the flightplan mission",
    )
    parser.add_argument(
        "--parameters",
        type=json_to_dict,
        help="The drone flight parameters in a json",
    )

    parser.add_argument("--outfile", required=True, help="output GeoJSON file")

    args = parser.parse_args(args_list)

    inpointsfile = open(args.waypoints_geojson, "r")
    points = inpointsfile.read()

    placemark_data = create_placemarks(geojson.loads(points), args.parameters)

    with open(args.outfile, "w") as f:
        f.write(geojson.dumps(placemark_data, indent=2))


if __name__ == "__main__":
    main()
