import argparse
import geojson
import logging
from typing import Union
from geojson import FeatureCollection
from drone_flightplan.calculate_parameters import calculate_parameters
from drone_flightplan.waypoints import create_waypoint
from drone_flightplan.add_elevation_from_dem import add_elevation_from_dem
from drone_flightplan.create_placemarks import create_placemarks
from drone_flightplan.wpml import create_wpml


# Instantiate logger
log = logging.getLogger(__name__)


def create_flightplan(
    aoi: Union[str, FeatureCollection, dict],
    forward_overlap: float,
    side_overlap: float,
    agl: float,
    gsd: float = None,
    image_interval: int = 2,
    dem: str = None,
    outfile: str = None,
    generate_each_points: bool = False,
):
    """
    Arguments:
        aoi: The area of interest in GeoJSON format.
        forward_overlap: The forward overlap in percentage.
        side_overlap: The side overlap in percentage.
        agl: The altitude above ground level in meters.
        gsd: The ground sampling distance in meters.
        image_interval: The time interval between two consecutive images in seconds.
    Returns:
        Drone Flightplan in kmz format
    """

    parameters = calculate_parameters(
        forward_overlap, side_overlap, agl, gsd, image_interval
    )

    waypoints = create_waypoint(
        aoi, agl, gsd, forward_overlap, side_overlap, generate_each_points
    )

    # Add elevation data to the waypoints
    if dem:
        # TODO: Do this with inmemory data
        outfile_with_elevation = "/tmp/output_file_with_elevation.geojson"
        add_elevation_from_dem(dem, waypoints, outfile_with_elevation)

        inpointsfile = open(outfile_with_elevation, "r")
        waypoints = inpointsfile.read()

    # calculate the placemark data
    placemarks = create_placemarks(geojson.loads(waypoints), parameters)

    # create wpml file
    outpath = create_wpml(placemarks, outfile)
    log.info(f"Flight plan generated in the path {outpath}")
    return outpath


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

    group = parser.add_mutually_exclusive_group(required=True)

    group.add_argument(
        "--altitude_above_ground_level",
        type=float,
        help="The flight altitude in meters.",
    )
    group.add_argument(
        "--gsd",
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
        "--image_interval",
        type=int,
        default=2,
        help="The time interval between two consecutive images in seconds.",
    )

    parser.add_argument("--inraster", help="input DEM GeoTIFF raster file")
    parser.add_argument("--outfile", required=True, help="output GeoJSON file")
    parser.add_argument(
        "--generate_each_points",
        action="store_true",
        help="Do you want waypoints or waylines.",
    )

    args = parser.parse_args()

    with open(args.project_geojson, "r") as f:
        aoi = geojson.load(f)

    create_flightplan(
        aoi,
        args.forward_overlap,
        args.side_overlap,
        args.altitude_above_ground_level,
        args.gsd,
        args.image_interval,
        args.inraster,
        args.outfile,
        args.generate_each_points,
    )


if __name__ == "__main__":
    main()


# python3 create_flightplan.py --project_geojson '/home/niraj/NAXA/HOT/adarsha_polygons_for_terrain_testing.geojson'  --altitude_above_ground_level 118 --forward_overlap 75 --side_overlap 70 --image_interval 2 --inraster '/home/niraj/Downloads/Bhanu.tif'  --outfile /home/niraj/NAXA/HOT/drone-flightplan/drone_flightplan --generate_each_points
