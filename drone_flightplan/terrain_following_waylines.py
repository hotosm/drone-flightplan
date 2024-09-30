#!/usr/bin/python3
"""
Convert a terrain following drone waypoint mission to a terrain following wayline mission by removing as many waypoints as can be done without deviating beyond a certain threshold from the desired Altitude Above Ground Level.
"""

import argparse

import json
from shapely.geometry import shape
from shapely import distance
from shapely.ops import transform
from pyproj import Transformer



def extractLines(plan):
    """
    Accepts a flight plan as a list of dicts
    Returns each of the individual lines of a flight plan as list of dicts
    """
    # Empty list to contain all of the lines in the flight plan, each line
    # defined as a series of points proceeding along the same heading
    lines = []
    # Empty list to hold the points of each individual line
    currentline = []
    lastheading = None

    for point in plan:
        currentheading = point['properties']['heading']
        # If it's not the first line, and the direction has changed
        if lastheading and currentheading != lastheading:
         # Yep, changed direction. Add the current line to the list of lines
            # and empty the current line before adding the current point.
            lines.append(currentline)
            currentline = []
        currentline.append(point)
        lastheading = currentheading
    return lines

def trim(line, threshold):
    """
    Accepts a waypoint flight line as a list of dicts from GeoJSON
    Returns a wayline flight line, with the first and last point intact
    but as many as possible of the intermediate points removed
    consistent with not exceeding the threshold of deviation from AGL
    as a list of dicts
    """
    transformer = Transformer.from_crs(4326, 3857, always_xy=True).transform
    
    # All points, indexed, in EPSG:3857 (tp for transformed_points)
    tp = []
    for point in line:
        indexedpoint = {}
        geom = transform(transformer, shape(point['geometry']))
        indexedpoint['index'] = point['properties']['index']
        indexedpoint['geometry'] = geom
        tp.append(indexedpoint)

    # Keeper points we already know about (first and last for now)
    kp = [tp[0], tp[-1]]

    # new keeper points after injection
    nkp = inject(kp, tp, threshold)

    return nkp

def inject(kp, tp, threshold):
    """
    Add the point furthest from consistent AGL (if over threshold)
    kp is the keeper_points (the ones we already know we need), indexes only
    tp is the transformed points (in EPSG:3857)
    threshold is how far the point should be allowed to deviate from AGL

    Returns a new list of keeperpoints (indexes only)
    """
    #print(f"\nHere are the current points we need to keep:")
    currentpoint = kp[0]
    segments = []
    for endpoint in kp[1:]:
        segment = (currentpoint, endpoint)
        segments.append(segment)
        currentpoint = endpoint
        
    for segment in segments:
        print(segment)
        run = distance(segment[0]['geometry'], segment[1]['geometry'])
        rise = segment[1]['geometry'].z - segment[0]['geometry'].z
        slope = rise / run
        print(f"Run: {run}, Rise: {rise}, Slope: {slope}")
        print(f"Index of first point to check: {segment[1]['index']}")

        
        
        
    
    

if __name__ == "__main__":
    p = argparse.ArgumentParser()

    p.add_argument("infile", help="input flight plan as GeoJSON")
    p.add_argument("-th", "--threshold",
                   help='Allowable altitude deviation in meters', default=2)
    #p.add_argument("outfile", help="output file")

    a = p.parse_args()

    print(f"\nLet's fuckin' goooo!")

    inplan = json.load(open(a.infile))['features']

    lines = extractLines(inplan)

    print(f"\nThis flight plan consists of {len(lines)} lines, like so:")
    for line in lines:
        print(f'A line {len(line)} points long')
    
    waylines = []
    for line in lines:
        wayline = trim(line, a.threshold)
        waylines.append(wayline)
    print(f"\nOk, let's examine the trimmed flight lines:")
    for wayline in waylines:
        print(wayline)


