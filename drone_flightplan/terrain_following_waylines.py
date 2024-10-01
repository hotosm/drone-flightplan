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

    # new keeper segments after injection
    nkp = inject(kp, tp, threshold)

    while set(inject(nkp, tp, threshold)) != set(nkp):
        nkp = inject(nkp, tp, threshold)

    nkpset = set(nkp)
    new_line = [p for p in line if p['properties']['index'] in nkpset]
    return new_line

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
    currentpoint = kp[0]
    segments = []
    for endpoint in kp[1:]:
        print(f"Going for currentpoint {currentpoint} and endpoint {endpoint}")
        segment = (tp[currentpoint - kp[0]], tp[endpoint - kp[0]])
        segments.append(segment)
        currentpoint = endpoint

    new_keeperpoints = []

    for segment in segments:
        fp = segment[0]['geometry']
        run = distance(fp, segment[1]['geometry'])
        rise = segment[1]['geometry'].z - segment[0]['geometry'].z
        slope = rise / run
        print(f"\nRun: {run}, Rise: {rise}, Slope: {slope}\n")
        max_agl_difference = 0
        max_agl_difference_point = 0
        injection_point = None
        points_to_traverse = segment[1]['index'] - segment[0]['index']
        #print("index, expected z, z, AGL Difference")
        for i in range(1, points_to_traverse):
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
            for new_point in new_segment:
                new_keeperpoints.append(new_point['index'])
        else:
            for point in segment:
                new_keeperpoints.append(point['index'])

    return new_keeperpoints
            
if __name__ == "__main__":
    p = argparse.ArgumentParser()

    p.add_argument("infile", help="input flight plan as GeoJSON")
    p.add_argument("outfile", help="output flight plan as GeoJSON")
    p.add_argument("-th", "--threshold", type=float,
                   help='Allowable altitude deviation in meters', default=5)
    #p.add_argument("outfile", help="output file")

    a = p.parse_args()

    print(f"\nLet's fuckin' goooo!")

    injson = json.load(open(a.infile))

    inheader = injson['type']
    inplan = injson['features']

    # Skip the first point which is a dummy waypoint in mid flighplan
    lines = extract_lines(inplan[1:])

    print(f"\nThis flight plan consists of {len(lines)} lines, like so:")
    for line in lines:
        print(f'A line {len(line)} points long')
    
    features = []
    for line in lines:
        wayline = trim(line, a.threshold)
        for point in wayline:
            features.append(point)

    outgeojson = {}

    outgeojson['type'] = injson['type']
    outgeojson['features'] = features
        
    with open(a.outfile, 'w') as output_file:
        json.dump(outgeojson, output_file)
