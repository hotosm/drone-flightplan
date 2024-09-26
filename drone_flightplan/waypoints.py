import logging
import argparse
from math import sqrt
import pyproj
import geojson
from shapely.geometry import Point, shape, Polygon
from shapely.affinity import rotate
from shapely.ops import transform
from drone_flightplan.calculate_parameters import calculate_parameters as cp


log = logging.getLogger(__name__)


def generate_grid_in_aoi(
    aoi_polygon: shape, x_spacing: float, y_spacing: float
) -> list[Point]:
    """
    Generate a grid of points within a given Area of Interest (AOI) polygon.

    Parameters:
        aoi_polygon (shape): The Shapely shape representing the area of interest.
        x_spacing (float): The spacing between points along the x-axis (in meters).
        y_spacing (float): The spacing between points along the y-axis (in meters).

    Returns:
        list[Point]: A list of Points representing the generated grid within the AOI.
    """
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


def calculate_distance(point1, point2):
    """
    Calculate Euclidean distance between two points.
    """
    return sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)


def create_path(
    points: list[Point],
    forward_spacing: float,
    generate_3d: bool = False,
    take_off_point: list[float] = None,
) -> list[dict]:
    """
    Create a continuous path of waypoints from a grid of points.

    Parameters:
        points (list[Point]): A list of Points representing the grid.
        forward_spacing (float): The spacing between rows of points (in meters).
        generate_3d (bool): Whether to generate additional 3D waypoints for the path.

    Returns:
        list[dict]: A list of dictionaries representing the waypoints along the path.
    """
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

        # initialize points at the start and end of each row
        first_point = row_points[0]
        last_point = row_points[-1]

        # define coordinates for extra points
        start_extra_point = Point(
            first_point.x - (forward_spacing if idx % 2 == 0 else -forward_spacing),
            first_point.y,
        )
        end_extra_point = Point(
            last_point.x + (forward_spacing if idx % 2 == 0 else -forward_spacing),
            last_point.y,
        )

        # Add the extra points with no photo taken
        continuous_path.append(
            {
                "coordinates": start_extra_point,
                "angle": -90 if idx % 2 == 0 else 90,
                "take_photo": False,
                "gimbal_angle": "-90",
            }
        )

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

        # Add the extra point at the end with no photo taken
        continuous_path.append(
            {
                "coordinates": end_extra_point,
                "angle": -90 if idx % 2 == 0 else 90,
                "take_photo": False,
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


def generate_3d_waypoints(
    row_points: list[Point], row_index: int, angle: int
) -> list[dict]:
    """
    Generate additional 3D waypoints by alternating the gimbal angle for each row.

    Parameters:
        row_points (list[Point]): A list of Points in the current row.
        row_index (int): The index of the current row.
        angle (int): The angle at which the gimbal should be tilted.

    Returns:
        list[dict]: A list of dictionaries representing the additional 3D waypoints.
    """
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


def exclude_no_fly_zones(points: list[dict], no_fly_zones: list[Polygon]) -> list[dict]:
    """
    Exclude waypoints that fall within defined no-fly zones.

    Parameters:
        points (list[dict]): A list of waypoints.
        no_fly_zones (list[Polygon]): A list of Polygons representing no-fly zones.

    Returns:
        list[dict]: A list of waypoints excluding those within no-fly zones.
    """
    return [
        point
        for point in points
        if not any(nfz.contains(point["coordinates"]) for nfz in no_fly_zones)
    ]


def remove_middle_points(data):
    processed_data = []
    i = 0

    while i < len(data):
        current_angle = data[i]["angle"]
        segment_start = i

        # Find the end of the segment with the same angle
        while i < len(data) and data[i]["angle"] == current_angle:
            i += 1

        segment_end = i

        # If the segment has more than 4 points, keep only the first 2 and the last 2
        if segment_end - segment_start > 4:
            processed_data.extend(data[segment_start : segment_start + 2])
            processed_data.extend(data[segment_end - 2 : segment_end])
        else:
            processed_data.extend(data[segment_start:segment_end])

    # Make take_photo = False for all the points
    for point in processed_data:
        point["take_photo"] = False

    return processed_data


def create_waypoint(
    project_area: dict,
    agl: float,
    gsd: float,
    forward_overlap: float,
    side_overlap: float,
    rotation_angle: float = 0.0,
    generate_each_points: bool = False,
    generate_3d: bool = False,
    no_fly_zones: dict = None,
    take_off_point: list[float] = None,
) -> str:
    """
    Create waypoints for a given project area based on specified parameters.

    Parameters:
        project_area (dict): GeoJSON dictionary representing the project area.
        agl (float): Altitude above ground level.
        gsd (float): Ground Sampling Distance.
        forward_overlap (float): Forward overlap percentage for the waypoints.
        side_overlap (float): Side overlap percentage for the waypoints.
        rotation_angle (float): The rotation angle for the flight grid in degrees.
        generate_each_points (bool): Flag True to generate individual waypoints, False for waylines.
        generate_3d (bool): Flag to determine if 3D waypoints should be generated.
        no_fly_zones (dict, optional): GeoJSON dictionary representing no-fly zones.
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

    # Calculate the centroid for centering the grid
    centroid = polygon_3857.centroid

    # Rotate the polygon to the specified angle around the centroid
    rotated_polygon = rotate(
        polygon_3857, rotation_angle, origin=centroid, use_radians=False
    )

    # Generate grid within the rotated AOI
    grid = generate_grid_in_aoi(rotated_polygon, forward_spacing, side_spacing)

    # Create path (either waypoints or waylines) and rotate back to original angle
    initial_path = create_path(grid, forward_spacing, generate_3d=generate_3d)

    # Path initialization
    path = []

    # Conditionally add takeoff point if available
    if take_off_point:

        # Get the first and last point of the initial path
        first_path_point = initial_path[0]["coordinates"]
        last_path_point = initial_path[-1]["coordinates"]

        # Calculate distances from the takeoff point
        distance_to_first = calculate_distance(Point(transformer_to_3857(*take_off_point)), first_path_point)
        distance_to_last = calculate_distance(Point(transformer_to_3857(*take_off_point)), last_path_point)
        if distance_to_last < distance_to_first:
            initial_path.reverse()

        initial_point = {
            "coordinates": Point(transformer_to_3857(*take_off_point)),
            "take_photo": False,
            "angle": 0,
            "gimbal_angle": "-90",
        }
        path.append(initial_point)

    # Add the rest of the points with rotation
    path.extend(
        [
            {
                "coordinates": rotate(
                    point["coordinates"],
                    -rotation_angle,
                    origin=centroid,
                    use_radians=False,
                ),
                "angle": point["angle"],
                "take_photo": point["take_photo"],
                "gimbal_angle": point["gimbal_angle"],
            }
            for point in initial_path
        ]
    )

    # If generating waylines, just add two points at each end
    waypoints = remove_middle_points(path) if not generate_each_points else path

    # If no-fly zones are provided, exclude points that fall inside no-fly zones
    if no_fly_zones:
        no_fly_polygons = [
            transform(transformer_to_3857, shape(zone["geometry"]))
            for zone in no_fly_zones["features"]
        ]
        waypoints = exclude_no_fly_zones(waypoints, no_fly_polygons)

    # Generate GeoJSON features
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


def validate_coordinates(value):
    try:
        lon, lat = map(float, value.split(","))
        if not (-180 <= lon <= 180 and -90 <= lat <= 90):
            raise argparse.ArgumentTypeError(
                "Coordinates must be in the format 'longitude,latitude' and within valid ranges."
            )
        return [lon, lat]
    except ValueError:
        raise argparse.ArgumentTypeError(
            "Invalid format. Coordinates must be in 'longitude,latitude' format."
        )


def main():
    """
    The main entry point of the script. Parses command-line arguments and
    generates waypoints for a drone mission based on the provided parameters.
    """
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
        "--generate_each_points",
        action="store_true",
        help="Do you want waypoints or waylines.",
    )
    parser.add_argument(
        "--generate_3d", action="store_true", help="Generate 3D imagery."
    )
    parser.add_argument(
        "--no_fly_zones", type=str, help="GeoJSON file containing no-fly zones."
    )
    parser.add_argument(
        "--output_file_path",
        type=str,
        required=True,
        help="The output GeoJSON file path for the waypoints.",
    )
    parser.add_argument(
        "--take_off_point",
        required=True,
        type=validate_coordinates,
        help="Take off Point Coordinates in 'longitude,latitude' format (e.g., 82.52,28.29).",
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
        args.take_off_point,
    )

    with open(args.output_file_path, "w") as f:
        f.write(coordinates)

    return coordinates


if __name__ == "__main__":
    main()
