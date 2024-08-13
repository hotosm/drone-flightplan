import logging
import argparse
import geojson
import pyproj
from drone_flightplan.waypoints import create_waypoint, calculate_parameters
from drone_flightplan.create_wpml import create_xml
from drone_flightplan import sampleRasterAtPoints as sr

# Instantiate logger
log = logging.getLogger(__name__)

# Constant to convert gsd to Altitude above ground level
GSD_to_AGL_CONST = 29.7  # For DJI Mini 4 Pro


def process_waypoints_with_terrain_follow(waypoints, input_raster):
    # Define the coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")
    web_mercator = pyproj.CRS("EPSG:3857")

    # Define a transformer to convert from WGS 84 to Web Mercator
    transformer_to_3857 = pyproj.Transformer.from_crs(
        wgs84, web_mercator, always_xy=True
    )

    # Transform coordinates and include index
    points = [
        (index,)
        + transformer_to_3857.transform(
            waypoint["coordinates"][0], waypoint["coordinates"][1]
        )
        for index, waypoint in enumerate(waypoints)
    ]

    # Sample elevation data at transformed points
    grid_with_elevation = sr.sampleRasterFromPointsList(input_raster, points)

    # Update drone altitude from ground level with according to elevation
    base_elevation = grid_with_elevation[0][3]
    for i in grid_with_elevation:
        i.append(base_elevation - i[3])

    return grid_with_elevation


def generate_flightplan(
    project_area,
    agl=None,
    gsd=None,
    forward_overlap=70.0,
    side_overlap=70.0,
    generate_each_points=False,
    generate_3d=False,
    terrain_follow=False,
    input_raster=None,
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
    if gsd:
        agl = gsd * GSD_to_AGL_CONST

    parameters = calculate_parameters(agl, forward_overlap, side_overlap)

    waypoints = create_waypoint(
        project_area,
        agl,
        forward_overlap,
        side_overlap,
        generate_each_points,
        generate_3d,
    )

    if terrain_follow:
        grid_with_elevation = process_waypoints_with_terrain_follow(
            waypoints, input_raster
        )

    def add_speed_and_agl(placemark):
        index = placemark["index"]

        # 4th index is the elevation difference with reference to first point
        agl_diff = grid_with_elevation[index][4] if terrain_follow else 0

        new_placemark = []
        new_placemark.append(
            f"{placemark['coordinates'][0]},{placemark['coordinates'][1]}"
        )
        new_placemark.append(str(agl + agl_diff))  # altitude from ground level
        new_placemark.append(str(parameters["ground_speed"]))
        new_placemark.append(str(placemark["angle"]))  # direction angle
        new_placemark.append(str(placemark["take_photo"]))
        new_placemark.append(str(placemark["gimbal_angle"]))
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
        "--generate_each_points",
        action="store_true",
        help="Do you want waypoints or waylines.",
    )
    parser.add_argument(
        "--generate_3d", action="store_true", help="Do you want to generate 3D Imagery"
    )

    parser.add_argument(
        "--terrain_follow",
        action="store_true",
        help="Do you want to generate flight plan with terrain following",
    )
    parser.add_argument(
        "--input_raster", type=str, help="Digital Elevation Model GeoTIFF file"
    )

    parser.add_argument(
        "--output_file_path",
        type=str,
        required=True,
        help="The output file path for the KMZ file.",
    )

    args = parser.parse_args()

    # Custom validation logic for terrain_follow
    if args.terrain_follow and not args.input_raster:
        parser.error("--input_raster is required when --terrain_follow is set")

    with open(args.project_geojson, "r") as f:
        boundary = geojson.load(f)

    features = boundary["features"][0]

    generate_flightplan(
        features,
        args.altitude_above_ground_level,
        args.gsd,
        args.forward_overlap,
        args.side_overlap,
        args.generate_each_points,
        args.generate_3d,
        args.terrain_follow,
        args.input_raster,
        args.output_file_path,
    )
    log.info(f"Waypoints created and saved to: {args.output_file_path}")


if __name__ == "__main__":
    main()


# python3 flightplan.py --project_geojson '/home/niraj/NAXA/HOT/small_square.geojson'   --altitude_above_ground_level 118 --output_file_path output_path --forward_overlap 75 --side_overlap 70 --generate_3d --terrain_follow --input_raster '/home/niraj/NAXA/HOT/Kathmandu DEM/ALPSMLC30_N027E085_DSM.tif'
