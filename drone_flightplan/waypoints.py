import logging
import argparse
import pyproj
import geojson
from shapely.geometry import Point, shape
from shapely.affinity import rotate
from shapely.ops import transform
from drone_flightplan.calculate_parameters import calculate_parameters as cp

log = logging.getLogger(__name__)


def generate_grid_in_aoi(aoi_polygon, x_spacing, y_spacing):
    minx, miny, maxx, maxy = aoi_polygon.bounds
    xpoints = int((maxx - minx) / x_spacing) + 1
    ypoints = int((maxy - miny) / y_spacing) + 1

    points = []
    for yi in range(ypoints):
        for xi in range(xpoints):
            x = minx + xi * x_spacing
            y = miny + yi * y_spacing
            point = Point(x, y)
            if aoi_polygon.contains(point):
                points.append(point)

    return points


def create_path(points, generate_3d=False):
    rows = {}
    for point in points:
        row_key = round(point.y, 8)
        if row_key not in rows:
            rows[row_key] = []
        rows[row_key].append(point)

    continuous_path = []
    for idx, row in enumerate(sorted(rows.keys())):
        row_points = sorted(rows[row], key=lambda p: p.x)
        if idx % 2 == 1:
            row_points.reverse()

        # Add each point with its associated properties
        for point in row_points:
            continuous_path.append(
                {
                    "coordinates": point,
                    "angle": -90 if idx % 2 == 0 else 90,
                    "take_photo": True,
                    "gimbal_angle": "-90",
                }
            )

        if generate_3d:
            continuous_path.extend(
                generate_3d_waypoints(
                    row_points, idx, angle=(-90 if idx % 2 == 0 else 90)
                )
            )

    return continuous_path


def generate_3d_waypoints(row_points, angle, row_index):
    # Return path with -45 degree angle
    return_path = [
        {
            "coordinates": wp,
            "angle": str(-angle),
            "take_photo": True,
            "gimbal_angle": "-45"
            if row_index % 2 == 0
            else "45",  # Alternate angles based on row index
        }
        for wp in reversed(row_points)
    ]
    return_path[0]["take_photo"] = False
    return_path[-1]["take_photo"] = False

    # Forward path with 45 degree angle
    forward_path = [
        {
            "coordinates": wp,
            "angle": str(angle),
            "take_photo": True,
            "gimbal_angle": "45"
            if row_index % 2 == 0
            else "-45",  # Alternate angles based on row index
        }
        for wp in row_points
    ]
    forward_path[0]["take_photo"] = False
    forward_path[-1]["take_photo"] = False

    return return_path + forward_path


def exclude_no_fly_zones(points, no_fly_zones):
    return [
        point
        for point in points
        if not any(nfz.contains(point["coordinates"]) for nfz in no_fly_zones)
    ]


def create_waypoint(
    project_area,
    agl,
    gsd,
    forward_overlap,
    side_overlap,
    rotation_angle=0.0,
    generate_each_points=False,
    generate_3d=False,
    no_fly_zones=None,
):
    """
    Create waypoints for a given project area based on specified parameters.

    Parameters:
        project_area (dict): GeoJSON dictionary representing the project area.
        agl (float): Altitude above ground level.
        forward_overlap (float): Forward overlap percentage for the waypoints.
        side_overlap (float): Side overlap percentage for the waypoints.
        rotation_angle (float): The rotation angle for the flight grid in degrees.
        generate_each_points (bool): Flag to determine if each point should be generated.
        generate_3d (bool): Flag to determine if 3D waypoints should be generated.

    Returns:
        geojson: waypoints generated within the project area in the geojson format

    Example Response:
    {
        "type": "FeatureCollection",
            "features": [
                {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [
                    85.328347,
                    27.729837
                    ]
                },
                "properties": {
                    "index": 0,
                    "angle": "0",
                    "take_photo": false,
                    "gimbal_angle": "-90"
                }
            }
        }
    }

    """
    parameters = cp(forward_overlap, side_overlap, agl, gsd)
    side_spacing = parameters["side_spacing"]
    forward_spacing = parameters["forward_spacing"]

    polygon = shape(project_area["features"][0]["geometry"])

    wgs84 = pyproj.CRS("EPSG:4326")
    web_mercator = pyproj.CRS("EPSG:3857")

    transformer_to_3857 = pyproj.Transformer.from_crs(
        wgs84, web_mercator, always_xy=True
    ).transform
    transformer_to_4326 = pyproj.Transformer.from_crs(
        web_mercator, wgs84, always_xy=True
    ).transform

    polygon_3857 = transform(transformer_to_3857, polygon)

    # Rotate the polygon to the specified angle
    rotated_polygon = rotate(
        polygon_3857, rotation_angle, origin="centroid", use_radians=False
    )

    # Generate waypoints in the rotated AOI
    grid = generate_grid_in_aoi(rotated_polygon, forward_spacing, side_spacing)

    # Create path and rotate back to the original angle
    waypoints = create_path(grid, generate_3d=generate_3d)
    waypoints = [
        {
            "coordinates": rotate(
                wp["coordinates"],
                -rotation_angle,
                origin=rotated_polygon.centroid,
                use_radians=False,
            ),
            "angle": wp["angle"],
            "take_photo": wp["take_photo"],
            "gimbal_angle": wp["gimbal_angle"],
        }
        for wp in waypoints
    ]

    if no_fly_zones:
        no_fly_polygons = [
            transform(transformer_to_3857, shape(zone["geometry"]))
            for zone in no_fly_zones["features"]
        ]
        waypoints = exclude_no_fly_zones(waypoints, no_fly_polygons)

    features = []
    for index, wp in enumerate(waypoints):
        coordinates_4326 = transformer_to_4326(wp["coordinates"].x, wp["coordinates"].y)
        feature = geojson.Feature(
            geometry=geojson.Point(coordinates_4326),
            properties={
                "index": index,
                "heading": wp["angle"],
                "take_photo": wp["take_photo"],
                "gimbal_angle": wp["gimbal_angle"],
            },
        )
        features.append(feature)

    feature_collection = geojson.FeatureCollection(features)
    return geojson.dumps(feature_collection, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Generate waypoints for drone missions."
    )
    parser.add_argument(
        "--project_geojson_polygon",
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
        "--rotation_angle",
        type=float,
        default=0.0,
        help="The rotation angle for the flight grid in degrees.",
    )
    parser.add_argument(
        "--generate_each_points", action="store_true", help="Generate dense waypoints."
    )
    parser.add_argument(
        "--generate_3d", action="store_true", help="Generate 3D Imagery."
    )
    parser.add_argument(
        "--no_fly_zones", type=str, help="GeoJSON file containing no-fly zones."
    )
    parser.add_argument(
        "--output_file_path",
        type=str,
        required=True,
        help="The output file geojson file path for the waypoints file.",
    )

    args = parser.parse_args()

    with open(args.project_geojson_polygon, "r") as f:
        boundary = geojson.load(f)

    no_fly_zones = None
    if args.no_fly_zones:
        with open(args.no_fly_zones, "r") as f:
            no_fly_zones = geojson.load(f)

    coordinates = create_waypoint(
        boundary,
        args.altitude_above_ground_level,
        None,  # GSD can be None if you calculate based on altitude
        args.forward_overlap,
        args.side_overlap,
        args.rotation_angle,
        args.generate_each_points,
        args.generate_3d,
        no_fly_zones,
    )

    with open(args.output_file_path, "w") as f:
        f.write(coordinates)

    return coordinates


if __name__ == "__main__":
    main()
