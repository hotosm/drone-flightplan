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
    Return the waypoints of a flight plan as individual unidirectional lines.
    Works by inspecting the heading property of each point and creating
    a new line each time the direction changes.

    Parameter
    --------
    plan : list
        The 'features' portion of a GeoJSON flight plan from the Drone Tasking
        Manager. This consists of a list of waypoints (which are dicts)

    Returns
    --------
    waylines : list
        A list of lists containing waypoints (which are dicts)
    
    """
    # Empty list to contain all of the lines in the flight plan, each line
    # defined as a series of points proceeding along the same heading
    waylines = []
    
    # Empty list to hold the points of an individual unidirectional line
    currentline = []
    lastheading = None

    for point in plan:
        currentheading = point['properties']['heading']
        # If it's not the first line, and the direction has changed
        if lastheading and currentheading != lastheading:
            # Yep, changed direction. Add the current line to the list of lines
            # and flush the current line before adding the current point.
            waylines.append(currentline)
            currentline = []
        currentline.append(point)
        lastheading = currentheading
    return waylines

def trim(line, threshold):
    """
    Return a wayline flight line, with the first and last point intact
    but as many as possible of the intermediate points removed
    consistent with not exceeding the threshold of deviation from AGL.

    Parameters
    --------
    line : list
        A waypoint flight line as a list of waypoints (dicts)

    threshold : float
        The allowable deviation from a consistent AGL in m

    Returns
    --------
    new_line : list
        A wayline as list of waypoints (dicts), hopefully indistinguishable
        from the input line, but with fewer points while retaining the
        AGL within the threshold
    """
    # Work in meters. Assumes input is in EPSG:4326 (will break if not).
    transformer = Transformer.from_crs(4326, 3857, always_xy=True).transform
    
    # Create a set of all points, indexed, in EPSG:3857 (tp=transformed_points)
    tp = []
    for point in line:
        indexedpoint = {}
        geom = transform(transformer, shape(point['geometry']))
        indexedpoint['index'] = point['properties']['index']
        indexedpoint['geometry'] = geom
        tp.append(indexedpoint)
    
    # Keeper points (indexes only)we know about (initially first and last)
    kp = [tp[0]['index'], tp[-1]['index']]

    # New keeper points after injection of the intermediate points needed
    # to maintain consistent AGL
    # nkp = new keeper points
    nkp = inject(kp, tp, threshold)

    # Keep injecting needed points until there aren't any more needed
    # (presumably because the AGL differences are below the threshold)
    #
    # NOTE: Probably quite inefficient; it may be doing the injection twice,
    # once to check if it changes anything, and then again to actually do it.
    # However, it's working fast for now (and flight plans aren't likely to
    # become all that huge, so whatever, leave it pending profiling.
    while set(inject(nkp, tp, threshold)) != set(nkp):
        nkp = inject(nkp, tp, threshold)

    # The way we're handling segments in the inject function means that there
    # are duplicate points in the keeperpoints. Make it a set, which anyway
    # is more efficient and safer to use as a filter.
    # nkpset = new keeper point set
    nkpset = set(nkp)
    new_line = [p for p in line if p['properties']['index'] in nkpset]
    return new_line

def inject(kp, tp, threshold):
    """
    Add the point furthest from consistent AGL (if over threshold)

    Parameters:
    --------
    kp : list
        A list of integer waypoint indexes, the "keeper points' we already know
        must be retained (such as the start and end points of the waylines)
    tp : list
        A list of actual waypoints (dicts from GeoJSON)

    threshold : float
        The allowable deviation from a consistent AGL in m

    Returns:
    --------
    new_keeperpoints : list
        A list of integer waypoint indexes
    """
    # Create segements between existing keeper waypoints; each segement
    # will be examined to see if another one in the middle is needed
    currentpoint = kp[0]
    segments = []
    for endpoint in kp[1:]:
        segment = (tp[currentpoint - kp[0]], tp[endpoint - kp[0]])
        segments.append(segment)
        currentpoint = endpoint

    new_keeperpoints = []

    # Run through all the segments between keeperpoints and inject (from
    # the entire list of original waypoints) those needed to keep AGL
    # deviation below the threshold
    for segment in segments:
        fp = segment[0]['geometry']
        run = distance(fp, segment[1]['geometry'])
        rise = segment[1]['geometry'].z - segment[0]['geometry'].z
        #print(f"rise: {rise}, run: {run}")
        # Avoid divide by zero if for some reason two waypoints are on
        # top of one another
        slope = 0
        if run:
            slope = rise / run
        max_agl_difference = 0
        max_agl_difference_point = 0
        injection_point = None
        points_to_traverse = segment[1]['index'] - segment[0]['index']
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
        #print(f"Max AGL difference in this segment: {max_agl_difference}"
        #      f" at point {max_agl_difference_point}")
        if injection_point:
            new_segment = [segment[0], tp[injection_point], segment[1]]
            for new_point in new_segment:
                new_keeperpoints.append(new_point['index'])
        else:
            for point in segment:
                new_keeperpoints.append(point['index'])

    return new_keeperpoints
            
if __name__ == "__main__":
    """
    Remove as many as possible of the waypoints in a Drone Tasking Manager
    flight plan, leaving those waypoints needed to prevent the AGL from
    exceeding a specified threshold.

    Parameters:
    --------
    infile : path
        An input GeoJSON file from the Drone Tasking Manager
    outfile : path
        An output GeoJSON file, hopefully identical to the input file but with
        many fewer waypoints but still maintaining AGL within threshold
    threshold : float
        The allowable deviation from consistent AGL.

    Returns:
    --------
    Writes a GeoJSON file containing the waypoints remaining after removal
    of as many as we can without AGL deviation exceeding threshold
    """
    p = argparse.ArgumentParser()

    p.add_argument("infile", help="input flight plan as GeoJSON")
    p.add_argument("outfile", help="output flight plan as GeoJSON")
    p.add_argument("-th", "--threshold", type=float,
                   help='Allowable altitude deviation in meters', default=5)

    a = p.parse_args()

    print(f"\nLet's fuckin' goooo!")

    injson = json.load(open(a.infile))

    inheader = injson['type']
    inplan = injson['features']

    # Skip the first point which is a dummy waypoint in the middle of
    # the flightplan area (for safety ascending to working altitude)
    # TODO: this should probably be a parameter as it's not necessarily
    # the case that all flight plans will have a dummy first point
    lines = extract_lines(inplan[1:])

    print(f"\nThis flight plan consists of {len(lines)} lines "
          f"and {len(inplan)} waypoints.")
    #for line in lines:
    #    print(f'A line {len(line)} points long')
    
    features = []
    for line in lines:
        wayline = trim(line, a.threshold)
        for point in wayline:
            features.append(point)

    outgeojson = {}

    outgeojson['type'] = injson['type']
    outgeojson['features'] = features

    print(f"\nThe output flight plan consists of {len(features)} waypoints.")
        
    with open(a.outfile, 'w') as output_file:
        json.dump(outgeojson, output_file)
