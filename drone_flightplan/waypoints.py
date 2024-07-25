import logging
import argparse
import pyproj
import geojson
from shapely.geometry import Polygon


# Instantiate logger
log = logging.getLogger(__name__)


def calculate_parameters(
    agl: float,
    forward_overlap: float,
    side_overlap: float,
):
    """
    Parameters
    ---------------------------------
    AGL(Altitude above ground level in meter ) = 115
    Forward overlap = 75
    Side overlap = 75

    ## Fixed Parameters
    Image interval = 2 sec
    Vertical FOV = 0.71
    Horizontal FOV = 1.26

    Forward Photo height = AGL * Vertical_FOV = 115*0.71 = 81.65
    Side Photo width = AGL * Horizontal_FOV = 115*1.26 = 144
    forward overlap distance =  forward photo height * forward overlap = 75 / 100 * 81.65 = 61.5
    side overlap distance = side photo width * side overlap = 75 / 100 * 144 = 108
    forward spacing =  forward photo height - forward overlap distance = 81.65 - 61.5 = 20.15
    side spacing = side photo width - side overlap distance = 144 - 108 = 36
    ground speed = forward spacing / image interval = 10

    """

    # Constants
    image_interval = 2
    vertical_fov = 0.71
    horizontal_fov = 1.26

    # Calculations
    forward_photo_height = agl * vertical_fov
    side_photo_width = agl * horizontal_fov
    forward_overlap_distance = forward_photo_height * forward_overlap / 100
    side_overlap_distance = side_photo_width * side_overlap / 100
    forward_spacing = forward_photo_height - forward_overlap_distance
    side_spacing = side_photo_width - side_overlap_distance
    ground_speed = forward_spacing / image_interval

    return {
        "forward_photo_height": round(forward_photo_height, 0),
        "side_photo_width": round(side_photo_width, 0),
        "forward_overlap_distance": round(forward_overlap_distance, 2),
        "side_overlap_distance": round(side_overlap_distance, 2),
        "forward_spacing": round(forward_spacing, 2),
        "side_spacing": round(side_spacing, 2),
        "ground_speed": round(ground_speed, 2),
    }


def generate_waypoints_within_polygon(
    aoi: Polygon,
    distance_between_lines_x: float,
    distance_between_lines_y: float,
    generate_each_points: bool,
    generate_3d: bool,
):
    minx, miny, maxx, maxy = aoi.bounds

    centroid = aoi.centroid

    waypoints = [
        {
            "coordinates": (centroid.x, centroid.y),
            "angle": "0",
            "take_photo": False,
            "gimbal_angle": "-90",
        }
    ]

    # Generate waypoints within the polygon
    y = miny
    row_count = 0
    angle = -90

    while y <= maxy:
        x = minx - distance_between_lines_x
        x_row_waypoints = []

        while x <= maxx + 2 * distance_between_lines_x:
            x_row_waypoints.append(
                {
                    "coordinates": (x, y),
                    "angle": str(angle),
                    "take_photo": True,
                    "gimbal_angle": "-90",
                }
            )
            x += distance_between_lines_x
        y += distance_between_lines_y

        if generate_each_points:
            if row_count % 2 == 0:
                # Forward path
                first_point = x_row_waypoints[0].copy()
                first_point["take_photo"] = False
                waypoints.append(first_point)
                waypoints.extend(x_row_waypoints[1:-1])
                last_point = x_row_waypoints[-1].copy()
                last_point["take_photo"] = False
                waypoints.append(last_point)

                if generate_3d:
                    # Return path with -45 degree angle
                    return_path = [
                        {
                            "coordinates": wp["coordinates"],
                            "angle": str(angle * -1),
                            "take_photo": True,
                            "gimbal_angle": "-45",
                        }
                        for wp in reversed(x_row_waypoints)
                    ]

                    # do not take picture in first and last point
                    return_path[0]["take_photo"] = False
                    return_path[-1]["take_photo"] = False

                    waypoints.extend(return_path)

                    # return path with 45 degree angle
                    forward_path = [
                        {
                            "coordinates": wp["coordinates"],
                            "angle": str(angle),
                            "take_photo": True,
                            "gimbal_angle": "-45",
                        }
                        for wp in x_row_waypoints
                    ]

                    # do not take picture in first and last point
                    forward_path[0]["take_photo"] = False
                    forward_path[-1]["take_photo"] = False
                    waypoints.extend(forward_path)

            else:
                last_point = x_row_waypoints[-1].copy()
                last_point["take_photo"] = False
                waypoints.append(last_point)
                waypoints.extend(reversed(x_row_waypoints[1:-1]))
                first_point = x_row_waypoints[0].copy()
                first_point["take_photo"] = False
                waypoints.append(first_point)

                if generate_3d:
                    # Return path with -45 degree angle
                    return_path = [
                        {
                            "coordinates": wp["coordinates"],
                            "angle": str(angle * -1),
                            "take_photo": True,
                            "gimbal_angle": "45",
                        }
                        for wp in x_row_waypoints
                    ]
                    # do not take picture in first and last point
                    return_path[0]["take_photo"] = False
                    return_path[-1]["take_photo"] = False
                    waypoints.extend(return_path)

                    # Forward path with +45 degree angle
                    forward_path = [
                        {
                            "coordinates": wp["coordinates"],
                            "angle": str(angle * -1),
                            "take_photo": True,
                            "gimbal_angle": "45",
                        }
                        for wp in reversed(x_row_waypoints)
                    ]

                    # do not take picture in first and last point
                    forward_path[0]["take_photo"] = False
                    forward_path[-1]["take_photo"] = False
                    waypoints.extend(forward_path)

        else:
            for x_row_waypoint in x_row_waypoints:
                x_row_waypoint["take_photo"] = False

            if x_row_waypoints:
                if row_count % 2 == 0:
                    # add first point
                    waypoints.append(x_row_waypoints[0])

                    # add second point too, if there are more than 1 points
                    # 2 points in each end are needed to straighten the flights
                    if len(x_row_waypoints) > 1:
                        waypoints.append(x_row_waypoints[1])

                    # If there are more than 2 points, add last 2 points
                    if len(x_row_waypoints) > 2:
                        waypoints.append(x_row_waypoints[-2])
                        waypoints.append(x_row_waypoints[-1])

                    if generate_3d:
                        ## Return path with -45 degree angle for the relevant points
                        # add last 2 points of the row in reverse order
                        if len(x_row_waypoints) > 2:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-1]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-2]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                        # add the first point for -45 angle in reverse order
                        if len(x_row_waypoints) > 1:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[1]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                        # add the first point for -45 angle
                        waypoints.append(
                            {
                                "coordinates": x_row_waypoints[0]["coordinates"],
                                "angle": str(angle * -1),
                                "take_photo": False,
                                "gimbal_angle": "-45",
                            }
                        )

                        # TODO: fix the angle, it might not just be 45 degree.
                        # add the 45 lateral
                        waypoints.append(
                            {
                                "coordinates": x_row_waypoints[0]["coordinates"],
                                "angle": str(angle),
                                "take_photo": False,
                                "gimbal_angle": "45",
                            }
                        )

                        # Forward path with +45 degree angle for the relevant points
                        if len(x_row_waypoints) > 1:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[1]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                        # If there are more than 2 points, add last 2 points
                        if len(x_row_waypoints) > 2:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-2]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-1]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                else:
                    if len(x_row_waypoints) > 2:
                        waypoints.append(x_row_waypoints[-1])
                        waypoints.append(x_row_waypoints[-2])
                    if len(x_row_waypoints) > 1:
                        waypoints.append(x_row_waypoints[1])
                    waypoints.append(x_row_waypoints[0])

                    if generate_3d:
                        ## RETURN PATH
                        # Point Index 0
                        waypoints.append(
                            {
                                "coordinates": x_row_waypoints[0]["coordinates"],
                                "angle": str(angle * -1),
                                "take_photo": False,
                                "gimbal_angle": "-45",
                            }
                        )

                        # Return path with -45 degree angle for the relevant points
                        if len(x_row_waypoints) > 1:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[1]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                        # Forward path with +45 degree angle for the relevant points
                        if len(x_row_waypoints) > 2:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-2]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-1]["coordinates"],
                                    "angle": str(angle * -1),
                                    "take_photo": False,
                                    "gimbal_angle": "-45",
                                }
                            )

                        ## FORWARD PATH
                        # Last Point Index
                        waypoints.append(
                            {
                                "coordinates": x_row_waypoints[-1]["coordinates"],
                                "angle": str(angle),
                                "take_photo": False,
                                "gimbal_angle": "45",
                            }
                        )
                        # Forward path with +45 degree angle for the relevant points
                        if len(x_row_waypoints) > 1:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[-2]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "45",
                                }
                            )
                        if len(x_row_waypoints) > 2:
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[1]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "45",
                                }
                            )
                            waypoints.append(
                                {
                                    "coordinates": x_row_waypoints[0]["coordinates"],
                                    "angle": str(angle),
                                    "take_photo": False,
                                    "gimbal_angle": "45",
                                }
                            )

        row_count += 1
        angle = angle * -1

    # We have gimbal angle 45. for the lateral 45 ( We change them to Pitch -90 and lateral -45) in the waypoints file
    waypoints.append(
        {
            "coordinates": (centroid.x, centroid.y),
            "angle": "0",
            "take_photo": False,
            "gimbal_angle": "-90",
        }
    )

    return waypoints


def create_waypoint(
    project_area, agl, forward_overlap, side_overlap, generate_each_points, generate_3d
):
    """
    Create waypoints for a given project area based on specified parameters.

    Parameters:
        project_area (dict): GeoJSON dictionary representing the project area.
        agl (float): Altitude above ground level.
        forward_overlap (float): Forward overlap percentage for the waypoints.
        side_overlap (float): Side overlap percentage for the waypoints.
        generate_each_points (bool): Flag to determine if each point should be generated.
        generate_3d (bool): Flag to determine if 3D waypoints should be generated.

    Returns:
        list: List of waypoints generated within the project area.

    Example Response:
        [
            {
            "coordinates": [
                85.3199287413415,
                27.71246834707544
            ],
            "angle": "-90",
            "take_photo": false,
            "gimbal_angle": "-90"
            },
            ...
        ]

    """

    parameters = calculate_parameters(agl, forward_overlap, side_overlap)

    side_spacing = parameters["side_spacing"]
    forward_spacing = parameters["forward_spacing"]

    # transform to 3857
    polygon = Polygon(project_area["geometry"]["coordinates"][0])

    # Define the coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")
    web_mercator = pyproj.CRS("EPSG:3857")

    # Define a transformer to convert from WGS 84 to Web Mercator
    transformer_to_3857 = pyproj.Transformer.from_crs(
        wgs84, web_mercator, always_xy=True
    )
    transformer_to_4326 = pyproj.Transformer.from_crs(
        web_mercator, wgs84, always_xy=True
    )

    polygon_3857 = Polygon(
        [
            transformer_to_3857.transform(lon, lat)
            for lon, lat in polygon.exterior.coords
        ]
    )

    distance_between_lines_x = forward_spacing
    distance_between_lines_y = side_spacing

    waypoints = generate_waypoints_within_polygon(
        polygon_3857,
        distance_between_lines_x,
        distance_between_lines_y,
        generate_each_points,
        generate_3d,
    )

    print("count of waypoints = ", len(waypoints))

    waypoints_4326 = [
        {
            "coordinates": transformer_to_4326.transform(
                wp["coordinates"][0], wp["coordinates"][1]
            ),
            "angle": wp["angle"],
            "take_photo": wp["take_photo"],
            "gimbal_angle": wp["gimbal_angle"],
        }
        for wp in waypoints
    ]
    return waypoints_4326


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
        "--generate_each_points",
        action="store_true",
        help="Do you want waypoints or waylines.",
    )
    parser.add_argument(
        "--generate_3d", action="store_true", help="Do you want to generate 3D Imagery"
    )

    args = parser.parse_args()

    with open(args.project_geojson_polygon, "r") as f:
        boundary = geojson.load(f)

    features = boundary["features"][0]

    coordinates = create_waypoint(
        features,
        args.altitude_above_ground_level,
        args.forward_overlap,
        args.side_overlap,
        args.generate_each_points,
        args.generate_3d,
    )

    return coordinates


if __name__ == "__main__":
    main()

# python3 waypoints.py  --forward_overlap 80 --side_overlap 75 --project_geojson_polygon '/home/niraj/0_1_sq_km_above_bpbd_office.geojson' --altitude_above_ground_level 100
