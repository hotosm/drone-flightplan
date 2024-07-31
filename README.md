# Drone Flightplan Generator

## Overview
The Drone Flightplan Generator is a Python package designed to automate the creation of flight plans for drones. This tool is essential for users needing efficient and precise aerial surveys, mapping, and imagery collection.

## Installation
To install the package, use pip

```pip install drone-flightplan-generator```

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
