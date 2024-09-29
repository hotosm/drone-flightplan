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
    firstpoint = shape(line[0]['geometry'])
    lastpoint = shape(line[-1]['geometry'])
    fp = transform(transformer, firstpoint)
    lp = transform(transformer, lastpoint)
    print(f"A line with first point {firstpoint} and last point {lastpoint}"
          f"for a total distance of {distance(fp, lp)}")
    for point in line:
        ps = shape(point['geometry'])
        pm = transform(transformer, ps)
        #print(f'{ps}, {pm}')


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

    #print(f'\nFor reference, the second line looks like this:')
    #for point in lines[1]:
    #    print(f"A point with Heading: {point['properties']['heading']}, "
    #          f"Altitude: {point['properties']['altitude']}, "
    #          f"take photo: {point['properties']['take_photo']}")

    print(f"\nOk, let's examine the altitude profiles of the flight lines:")
    waylines = []
    for line in lines:
        wayline = trim(line, a.threshold)
        waylines.append(wayline)
    print(waylines)


