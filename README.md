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
To get the drone waypoints for a given Geojson Polygon AOI, we can do this:
```
from drone_flightplan.waypoints import create_waypoint

create_waypoint(
        polygon_geojson,
        altitude_above_ground_level,
        forward_overlap,
        side_overlap,
        generate_each_points,
        generate_3d,
    )

Parameters:
  - polygon_geojson = Geojson Polygon AOI
  - altitude_above_ground_level = The height at which you want to fly the drone from ground level
  - forward_overlap = Forward Overlap you want in the imageries
  - side_overlap = Desired Side Overlap you want in the imageries
  - generate_each_points (bool) : True if you want waypoints and False if you want waylines
  - generate_3d : True if you want to generate 3d imageries. False if you just want 2d imageries. 3d imageries will take photos ad 3 different angles (-90, -45 and lateral 45 degree angle)

```

### Waypoint File
DJI drones requires waypoint file. 
WPML route files all end with a ".kmz" suffix and are essentially archive files packed in ZIP format. After decompression of a standard WPML route file, its file structure is as follows

![image](https://github.com/user-attachments/assets/bb7a6f95-29f8-40e0-972c-92a974aa0bf0)

You can find more on this from the documentation [here](https://github.com/dji-sdk/Cloud-API-Doc/blob/master/docs/en/60.api-reference/00.dji-wpml/10.overview.md).

You can get the waypoint file following this steps:
```
from drone_flightplan import flightplan

flightplan.generate_flightplan(
  project_area,
  altitude_above_ground_level,
  forward_overlap,
  side_overlap,
  generate_each_points,
  generate_3d,
  output_file_path
)

Parameters:
  - polygon_geojson = Geojson Polygon AOI
  - altitude_above_ground_level = The height at which you want to fly the drone from ground level
  - forward_overlap = Forward Overlap you want in the imageries
  - side_overlap = Desired Side Overlap you want in the imageries
  - generate_each_points (bool) : True if you want waypoints and False if you want waylines
  - generate_3d : True if you want to generate 3d imageries. False if you just want 2d imageries. 3d imageries will take photos ad 3 different angles (-90, -45 and lateral 45 degree angle)
  - output_file_path: The path where you want your output flightplan

```
