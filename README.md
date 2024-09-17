# Drone Flightplan
<!-- markdownlint-disable -->
<p align="center">
  <img src="https://github.com/hotosm/fmtm/blob/main/images/hot_logo.png?raw=true" style="width: 200px;" alt="HOT"></a>
</p>
<p align="center">
  <em>Generates waypoints for drone and creates a flightplan </em>
</p>

---

📖 **Documentation**: <a href="https://hotosm.github.io/drone-flightplan/" target="_blank">https://hotosm.github.io/drone-flightplan/</a>

🖥️ **Source Code**: <a href="https://github.com/hotosm/drone-flightplan" target="_blank">https://github.com/hotosm/drone-flightplan</a>

---


## Overview
The Drone Flightplan Generator is a Python package designed to automate the creation of flight plans for drones. This tool is essential for users needing efficient and precise aerial surveys, mapping, and imagery collection.

## Installation
To install the package, use pip

```pip install drone-flightplan```

### Usage

#### `create_waypoint`

To get the drone waypoints for a given Geojson Polygon AOI, we can do this:
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

Parameters:
  - project_area (dict): A GeoJSON dictionary representing the project area (Polygon AOI).
  - agl (float): Altitude above ground level (the height at which the drone will fly).
  - gsd (float): Ground Sampling Distance (resolution of the images captured).
  - forward_overlap (float): Desired forward overlap percentage for the images in the flight plan.
  - side_overlap (float): Desired side overlap percentage for the images in the flight plan.
  - rotation_angle (float, optional): The rotation angle for the flight grid in degrees (default is 0.0).
  - generate_each_points (bool, optional): True to generate individual waypoints, False to generate waylines 
    connecting waypoints for a continuous flight path.
  - generate_3d (bool, optional): True to capture 3D images at −90°, −45°, and 45° lateral angles, False to capture only
    2D images (default is False).
  - no_fly_zones (dict, optional): A GeoJSON dictionary representing no-fly zones (areas to avoid).
  - take_off_point (list[float], optional): The GPS coordinates of the take-off point [longitude, latitude] for the flight.

```

#### `create_flightplan`

This is the core function responsible for generating a complete flight plan for a specified area of interest (AOI).

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

Parameters:
  - aoi: The area of interest in GeoJSON format.
  - forward_overlap (float): Desired forward overlap percentage for the images.
  - side_overlap (float): Desired side overlap percentage for the images.
  - agl (float): Altitude above ground level (in meters) for drone flight.
  - gsd (float, optional): Ground sampling distance in cm/px. If not provided, it will be calculated based on the altitude.
  - image_interval (int, optional): Time interval (in seconds) between two consecutive image captures. 
  - dem (str, optional): Path to the Digital Elevation Model (DEM) file to add terrain elevation data to the flight plan.
  - outfile (str, optional): The output file path where the resulting flight plan will be saved. If not provided, 
    the flight plan is returned as a string.
  - generate_each_points (bool, optional): True to generate individual waypoints for flight, False to generate waylines.
  - rotation_angle (float, optional): The rotation angle (in degrees) for the flight grid. Default is 0.0.
  - take_off_point (list[float], optional): A list of GPS coordinates [longitude, latitude] for the takeoff point. 
```

### Waypoint File
DJI drones requires waypoint file. 
WPML route files all end with a ".kmz" suffix and are essentially archive files packed in ZIP format. After decompression of a standard WPML route file, its file structure is as follows

![image](https://github.com/user-attachments/assets/bb7a6f95-29f8-40e0-972c-92a974aa0bf0)

You can find more on this from the documentation [here](https://github.com/dji-sdk/Cloud-API-Doc/blob/master/docs/en/60.api-reference/00.dji-wpml/10.overview.md).
