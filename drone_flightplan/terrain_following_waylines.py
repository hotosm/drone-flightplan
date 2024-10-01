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
        print(point)
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

    print("\n************\nSegments:")
    for segment in segments:
        print(segment)
    
    for segment in segments:
        fp = segment[0]['geometry']
        run = distance(fp, segment[1]['geometry'])
        rise = segment[1]['geometry'].z - segment[0]['geometry'].z
        slope = rise / run
        print(f"Run: {run}, Rise: {rise}, Slope: {slope}")
        max_agl_difference = 0
        max_agl_difference_point = 0
        injection_point = None
        for i in range(1, segment[-1]['index'] - segment[0]['index']):
            pt = tp[i]['geometry']
            z = pt.z
            ptrun = distance(fp, pt)
            expected_z = fp.z + (ptrun * slope)
            agl_difference = z - expected_z
            if (abs(agl_difference) > max_agl_difference
                and agl_difference > threshold):
                max_agl_difference = agl_difference
                max_agl_difference_point = tp[i]['index']
                injection_point = i
            #print(f"{tp[i]['index']}: {expected_z}, {z}, {agl_difference}")
        print(f"Max AGL difference in this segment: {max_agl_difference}"
              f" at point {max_agl_difference_point}")
        if injection_point:
            new_segment = [segment[0], tp[injection_point], segment[1]]
            print(f"\nNew segment: {new_segment}")
        
        
        
    
    

if __name__ == "__main__":
    p = argparse.ArgumentParser()

    p.add_argument("infile", help="input flight plan as GeoJSON")
    p.add_argument("-th", "--threshold",
                   help='Allowable altitude deviation in meters', default=5)
    #p.add_argument("outfile", help="output file")

    a = p.parse_args()

    print(f"\nLet's fuckin' goooo!")

    injson = json.load(open(a.infile))
    print(injson)
    print(f"Type: {type(injson)}")

    inplan = injson['features']

    print(len(inplan))
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


