#!/bin/python3
"""
Creates a set of waypoints for drone mapping flight plans.

Arguments:
    Required:


Returns:


Typical usage example:


"""

__version__ = "2024-07-31"

import argparse
from osgeo import gdal, ogr, osr
import math

def bbox_from_gis_file(infile):
    """
    Accepts theoretically any OGR/GDAL-compatible GIS vector file,
    in any CRS, as long as the file knows its correct CRS.
    Returns a tuple of coordinates representing the bounding box,
    as (xmin, ymin, xmax, ymax) in EPSG:3857
    """
    f = ogr.Open(infile)
    lyr = f.GetLayer()
#    if len(lyr) != 1:
#        print('Sorry, that file has more than one feature')
#        return None
#    feature = list(lyr)[0] # sigh. Gets the single feature from the layer
#    geom = feature.GetGeometryRef()
#    
    crs = lyr.GetSpatialRef()
    extent = lyr.GetExtent()
    print(f'\nInput layer Coordinate Reference System: {crs.GetName()}')
    targetcrs = osr.SpatialReference()
    targetcrs.ImportFromEPSG(3857)
    transform = osr.CoordinateTransformation(crs, targetcrs)
    lowerleft = transform.TransformPoint(extent[0], extent[2])
    upperright = transform.TransformPoint(extent[1], extent[3])
    return (round(lowerleft[0],2), round(lowerleft[1], 2),
            round(upperright[0], 2), round(upperright[1], 2))


def grid(minx, miny, maxx, maxy, xspac: float, yspac: float):
    """
    Create a set of waypoints
    """
    
    xlen = maxx - minx
    ylen = maxy - miny
    xpoints = int(xlen / xspac + 1)
    ypoints = int(ylen / yspac + 1)
    points = []
    # loop through lines
    idx = 1
    for yi in range(0, ypoints, 2):
        # points heading out
        for xi in range(xpoints):
            x = minx + (xi * xspac)
            y = miny + (yi * yspac)
            points.append([idx, x, y])
            idx += 1
        # points returning
        for xi in range(xpoints, 0, -1):
            x = minx + ((xi - 1) * xspac)
            y = miny + ((yi + 1) * yspac)
            points.append([idx, x, y])
            idx += 1
    return points


if __name__ == "__main__":
    """
    Call the module from the console. See module docstring for usage details.
    """
    p = argparse.ArgumentParser()

    p.add_argument("-aoi", "--aoi", type=str,
                   help="GeoJSON AOI boundary polygon file")
    p.add_argument("-xspac", "--x-spacing", type=float,
                   help="Forward spacing")
    p.add_argument("-yspac", "--y-spacing", type=float,
                   help="Lateral spacing")
    a = p.parse_args()

    bbox = bbox_from_gis_file(a.aoi)
    print(f'The bbox extent is {bbox}')
    waypoints = grid(bbox[0],bbox[1],bbox[2],bbox[3], a.x_spacing, a.y_spacing)
    print(waypoints)
    
