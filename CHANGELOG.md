# CHANGELOG

## 0.3.4rc0 (2024-12-19)

### Feat

- add waypoints support to create_waypoint function

### Fix

- Handle out-of-bounds access in RasterIO and improve logging

## 0.3.3 (2024-12-09)

## 0.3.3rc2 (2024-12-09)

### Fix

- waypoint generation

## 0.3.3rc1 (2024-12-09)

## 0.3.3rc0 (2024-12-09)

## 0.3.2 (2024-11-27)

### Fix

- add ground speed cap with safety buffer to prevent controller rejection

## 0.3.1 (2024-10-14)

### Feat

- add logic to reverse initial path based on proximity to takeoff point
- update waypoints file
- update read me docs with all modules
- update read me docs

### Fix

- update distance calculation of first and last point based on tramsformer 3857 projection
- remove command to run waypoint
- remove python keywords from the docs copy command
- take photo action for waylines flight

## 0.3.1rc4 (2024-09-12)

### Feat

- Add support for generating waylines when generate_each_points is False

## 0.3.1rc3 (2024-09-10)

### Feat

- polygon edge coverage by adding extra waypoints
- add extra start and end waypoints for each row in aoi

### Refactor

- add type hinting, docstrings, and correct unit measurements

## 0.3.1rc2 (2024-08-29)

## 0.3.1rc1 (2024-08-20)

### Fix

- return path for flightplan

## 0.3.1rc0 (2024-08-20)

## 0.3.0 (2024-08-20)

### Feat

- create_flightplan module

## 0.2.3 (2024-08-06)

### Fix

- relax pin for shapely '==' to '>='

## 0.2.2 (2024-08-06)

## 0.2.1 (2024-08-05)

## v0.2.0 (2024-08-05)

## v0.1.2 (2024-08-01)

### Fix

- gimbal angle issue for flightplans

## v0.1.1 (2024-08-01)

## v0.1.0 (2024-07-25)

### Feat

- program to generate waypoints within a polygon created
- generate waypoints in a polygon function
