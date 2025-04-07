# Drone Flightplan

<p align="center">
  <img src="https://raw.githubusercontent.com/hotosm/fmtm/main/docs/images/hot_logo.png" style="width: 200px;" alt="HOT">
</p>

> [!WARNING]
>
> Archived.
>
> This repo was moved to be a package in a monorepo.
>
> https://github.com/hotosm/drone-tm/tree/development/src/backend/packages/drone-flightplan
>
> See: https://github.com/hotosm/drone-tm/pull/521

<p align="center">
  <em>Generates waypoints for drone and creates a flightplan</em>
</p>

---

📖 **Documentation**: [https://hotosm.github.io/drone-flightplan/](https://hotosm.github.io/drone-flightplan/)

🖥️ **Source Code**: [https://github.com/hotosm/drone-flightplan](https://github.com/hotosm/drone-flightplan)

---

## Overview

The Drone Flightplan Generator is a Python package designed to automate the creation of flight plans for drones. This tool is essential for users needing efficient and precise aerial surveys, mapping, and imagery collection.

### Waypoint File

DJI drones require waypoint files. WPML route files all end with a ".kmz" suffix and are essentially archive files packed in ZIP format. After decompression of a standard WPML route file, its file structure is as follows:

![image](https://github.com/user-attachments/assets/bb7a6f95-29f8-40e0-972c-92a974aa0bf0)

For more details, check the [DJI Cloud API documentation](https://github.com/dji-sdk/Cloud-API-Doc/blob/master/docs/en/60.api-reference/00.dji-wpml/10.overview.md).

## Installation

To install the package, use pip:

```bash
pip install drone-flightplan
```

## Modules

### 1. `calculate_parameters`

This module helps in calculating various parameters required for the flight plan, such as:

```
calculate_parameters(
    forward_overlap: float,
    side_overlap: float,
    agl: float,
    gsd: float = None,
    image_interval: int = 2,
)
```

**Parameters:**

- `AGL` (Altitude above ground level in meters) = 115
- `Forward overlap` = 75
- `Side overlap` = 75

**Fixed Parameters:**

- `Image interval` = 2 sec
- `Vertical FOV` = 0.71
- `Horizontal FOV` = 1.26

**Calculations:**

- Forward Photo height = AGL *Vertical_FOV = 115* 0.71 = 81.65
- Side Photo width = AGL *Horizontal_FOV = 115* 1.26 = 144
- Forward overlap distance = Forward photo height *Forward overlap = 75 / 100* 81.65 = 61.5
- Side overlap distance = Side photo width *Side overlap = 75 / 100* 144 = 108
- Forward spacing = Forward photo height - Forward overlap distance = 81.65 - 61.5 = 20.15
- Side spacing = Side photo width - Side overlap distance = 144 - 108 = 36
- Ground speed = Forward spacing / Image interval = 10

**Parameters:**

- `waypoints_geojson`: The waypoint coordinates to be included in the flight plan mission.
- `parameters`: The drone flight parameters in JSON format.

### 2. `create_waypoint`

This module generates waypoints for a given project area, using parameters such as altitude, GSD, overlap ratios, and the option to avoid no-fly zones. It can also create 3D waypoints:

```
from drone_flightplan import create_waypoint

create_waypoint(
    project_area,
    agl,
    gsd,
    forward_overlap,
    side_overlap,
    rotation_angle=0.0,
    generate_each_points=False,
    generate_3d=False,
    no_fly_zones=None,
    take_off_point=None,
)
```

**Parameters:**

- `project_area` (dict): A GeoJSON dictionary representing the project area (Polygon AOI).
- `agl` (float): Altitude above ground level (the height at which the drone will fly).
- `gsd` (float): Ground Sampling Distance (resolution of the images captured).
- `forward_overlap` (float): Desired forward overlap percentage for the images in the flight plan.
- `side_overlap` (float): Desired side overlap percentage for the images in the flight plan.
- `rotation_angle` (float, optional): The rotation angle for the flight grid in degrees (default is 0.0).
- `generate_each_points` (bool, optional): True to generate individual waypoints, False to generate waylines connecting waypoints for a continuous flight path.
- `generate_3d` (bool, optional): True to capture 3D images at −90°, −45°, and 45° lateral angles, False to capture only 2D images.
- `no_fly_zones` (dict, optional): A GeoJSON dictionary representing no-fly zones (areas to avoid).
- `take_off_point` (list[float], optional): The GPS coordinates of the take-off point [longitude, latitude] for the flight.

### 3. `add_elevation_from_dem`

This module integrates elevation data from Digital Elevation Models (DEMs) into the flight plan to account for changes in terrain. This ensures more accurate waypoint positioning for varying altitudes:

```
from drone_flightplan import add_elevation_from_dem

add_elevation_from_dem(raster_file, points, outfile)
```

**Parameters:**

- `raster_file`: Path to the DEM GeoTIFF file.
- `points`: GeoJSON string with point coordinates.
- `outfile`: Path for saving the output with added elevation.

### 4. `create_placemarks`

This module creates placemarks for the flight plan, useful for marking key locations:

```
from drone_flightplan import create_placemarks

create_placemarks(
    waypoints_geojson: Union[str, FeatureCollection, dict], 
    parameters: dict
)
```

### 5. `create_wpml`

This module is responsible for creating WPML files (Waypoint Markup Language), which are often used for visualizing waypoints and flight paths in different tools or simulators:

```
from drone_flightplan import create_wpml

create_wpml(
    placemark_geojson: Union[str, FeatureCollection, dict],
    output_file_path: str = "/tmp/",
)
```

**Parameters:**

- `placemark_geojson`: The placemark coordinates to be included in the flight plan mission.
- `output_file_path`: The output file path for the WPML file.

### 6. `create_flightplan`

This is the core function responsible for generating a complete flight plan for a specified area of interest (AOI):

```
from drone_flightplan import create_flightplan

create_flightplan(
    aoi,
    forward_overlap,
    side_overlap,
    agl,
    gsd=None,
    image_interval=2,
    dem=None,
    outfile=None,
    generate_each_points=False,
    rotation_angle=0.0,
    take_off_point=None,
)
```

**Parameters:**

- `aoi`: The area of interest in GeoJSON format.
- `forward_overlap` (float): Desired forward overlap percentage for the images.
- `side_overlap` (float): Desired side overlap percentage for the images.
- `agl` (float): Altitude above ground level (in meters) for drone flight.
- `gsd` (float, optional): Ground sampling distance in cm/px. If not provided, it will be calculated based on the altitude.
- `image_interval` (int, optional): Time interval (in seconds) between two consecutive image captures.
- `dem` (str, optional): Path to the Digital Elevation Model (DEM) file to add terrain elevation data to the flight plan.
- `outfile` (str, optional): The output file path where the resulting flight plan will be saved. If not provided, the flight plan is returned as a string.
- `generate_each_points` (bool, optional): True to generate individual waypoints for flight, False to generate waylines.
- `rotation_angle` (float, optional): The rotation angle (in degrees) for the flight grid. Default is 0.0.
- `take_off_point` (list[float], optional): A list of GPS coordinates [longitude, latitude] for the takeoff point.
