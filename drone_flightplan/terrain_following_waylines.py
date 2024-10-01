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




def extract_lines(plan):
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
        #print(point)
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
    print(f"\n\n************************************\nHere's another line:")
    transformer = Transformer.from_crs(4326, 3857, always_xy=True).transform
    
    # All points, indexed, in EPSG:3857 (tp for transformed_points)
    tp = []
    for point in line:
        indexedpoint = {}
        geom = transform(transformer, shape(point['geometry']))
        indexedpoint['index'] = point['properties']['index']
        indexedpoint['geometry'] = geom
        tp.append(indexedpoint)
    
    # Keeper points (indexes only)we know about (initially first and last)
    kp = [tp[0]['index'], tp[-1]['index']]

    # new keeper points after injection
    nkp = inject(kp, tp, threshold)

    print(f"\n\nNew keeper points from first round of injection:")
    print(nkp)

def inject(kp, tp, threshold):
    """
    Add the point furthest from consistent AGL (if over threshold)
    All points must be in a single line (as from function extract_lines
    kp is the keeper_points (the ones we already know we need)
    tp is the transformed points (in EPSG:3857)
    threshold is how far the point should be allowed to deviate from AGL

    Returns a new list of keeperpoints
    """
    print(f"\nRunning injection.\nkeeper points in this line: {kp}\n")
    print(f"Transformed points we're working on:")
    for p in tp:
        print(f"{p['index']}, {p['geometry']}")
    currentpoint = kp[0]
    segments = []
    for endpoint in kp[1:]:
        print(f"Going for currentpoint {currentpoint} and endpoint {endpoint}")
        segment = (tp[currentpoint], tp[endpoint])
        segments.append(segment)
        currentpoint = endpoint

    new_keeperpoints = []

    print("\n************\nSegments:")
    for segment in segments:
        print(segment)

    new_segments = []
    for segment in segments:
        fp = segment[0]['geometry']
        run = distance(fp, segment[1]['geometry'])
        rise = segment[1]['geometry'].z - segment[0]['geometry'].z
        slope = rise / run
        print(f"\nRun: {run}, Rise: {rise}, Slope: {slope}\n")
        max_agl_difference = 0
        max_agl_difference_point = 0
        injection_point = None
        print("index, expected z, z, AGL Difference")
        for i in range(segment[0]['index'] + 1, segment[1]['index']):
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
            print(f"{tp[i]['index']}: {expected_z}, {z}, {agl_difference}")
        print(f"Max AGL difference in this segment: {max_agl_difference}"
              f" at point {max_agl_difference_point}")
        if injection_point:
            new_segment = [segment[0], tp[injection_point], segment[1]]
            new_segments.append(new_segment)
        else:
            new_segments.append(segment)

    new_kp_set_indexes = set()
    for segment in new_segments:
        for point_maybe_not_unique in segment:
            new_kp_set_indexes.add(point_maybe_not_unique['index'])

    print(new_kp_set_indexes)

    return new_segments
            
if __name__ == "__main__":
    p = argparse.ArgumentParser()

    p.add_argument("infile", help="input flight plan as GeoJSON")
    p.add_argument("-th", "--threshold",
                   help='Allowable altitude deviation in meters', default=5)
    #p.add_argument("outfile", help="output file")

    a = p.parse_args()

    print(f"\nLet's fuckin' goooo!")

    injson = json.load(open(a.infile))
    #print(injson)
    #print(f"Type: {type(injson)}")

    inplan = injson['features']

    lines = extract_lines(inplan)

    print(f"\nThis flight plan consists of {len(lines)} lines, like so:")
    for line in lines:
        print(f'A line {len(line)} points long')
    
    waylines = []
    for line in lines:
        wayline = trim(line, a.threshold)
        waylines.append(wayline)
#    print(f"\nOk, let's examine the trimmed flight lines:")
#    for wayline in waylines:
#        print(wayline)


