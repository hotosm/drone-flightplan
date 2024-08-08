#!/usr/bin/python3
"""
Sample a Digital Elevation Model raster values at points. Primarily intended
as a library; other Python modules can use it to add an elevation attribute
to a file of autogenerated waypoints.

However, it can be used standalone to sample a given DEM (GeoTIFF) at the
points given by a given GeoJSON point layer.

Used from the command line, it expects the points to be a GeoJSON file on the
local filesystem. 
"""

import argparse
from osgeo import ogr, gdal, osr
import math
import struct
import csv

def addElevationFromDEM(raster_file, point_file, outfile):
    """
    Arguments:
        DEM raster file as GeoTIFF
        Points as GeoJSON file
    Returns:
        GeoJSON with added elevation attribute on each point
    """
    r = gdal.Open(raster_file)
    
    # Create an empty Spatial Reference object
    # and set it to the CRS of the input raster
    rasterSR = osr.SpatialReference()
    rasterSR.ImportFromProj4(r.GetProjection())
    print(f"\nRaster Coordinate Reference System: {rasterSR}")

    # Get the raster band (if it's a DEM, this should be the only band)
    # TODO this would be a good time to check that it's a single-band
    # raster with values that make sense for a DEM
    band = r.GetRasterBand(1)

    # Determine the data type. The AW3D30 global 30-meter DEM is 32-bit floats,
    # but other DEMs might contain other data types. At the moment we're not
    # using this information to correctly unpack the data read from the band
    # (we're assuming 16-bit little-endian signed int), but we should be.
    raster_data_type = gdal.GetDataTypeName(band.DataType)
    print(f'\nRaster band 1 data type: {raster_data_type}')

    # Create the tranforms between geographical and pixel coordinates
    # The forward transform takes pixel coords and returns geographical coords,
    # the reverse transform... well, you get the idea.
    forward = r.GetGeoTransform()
    reverse = gdal.InvGeoTransform(forward)

    p = ogr.Open(point_file)
    lyr = p.GetLayer()
    print("\nPoint layer Coordinate Reference System:")
    pointSR = lyr.GetSpatialRef()
    print(pointSR.GetName())
    pointLD = lyr.GetLayerDefn()
    

    transform = osr.CoordinateTransformation(pointSR, rasterSR)

    # Create the GDAL GeoJSON output file infrastructure
    outDriver = ogr.GetDriverByName('GeoJSON')
    outDataSource = outDriver.CreateDataSource(outfile)
    outLayer = outDataSource.CreateLayer('waypoints', pointSR, ogr.wkbPoint)

    # Add an elevation field for easy reference (the point is XYZ but
    # it's nice to have access to the elevation as a property)
    elevation_field = ogr.FieldDefn('elevation', ogr.OFTReal)
    outLayer.CreateField(elevation_field)
    # Add fields from input layer to output layer
    fields = []
    for i in range(pointLD.GetFieldCount()):
        fd = pointLD.GetFieldDefn(i)
        fields.append(fd)
    for fd in fields:
        print(f'\nAdding field {fd.name} of type {fd.GetTypeName()}.')
        outLayer.CreateField(fd)
    featureDefn = outLayer.GetLayerDefn()

    for feature in lyr:
        geom = feature.GetGeometryRef()
        pointXYRasterCRS = transform.TransformPoint(geom.GetX(), geom.GetY())
        mapX = pointXYRasterCRS[1]
        mapY = pointXYRasterCRS[0]
        pixcoords = gdal.ApplyGeoTransform(reverse, mapX, mapY)
        pixX = math.floor(pixcoords[0])
        pixY = math.floor(pixcoords[1])
        elevationstruct = band.ReadRaster(pixX, pixY, 1, 1)
        # GDAL returns a C struct object when reading  a raster. Hassle.
        # TODO: make sure the struct data type being unpacked is correct.
        # For now using "<h" (16-bit signed little-endian int) works for the
        # DEMS from AW3D30. All format strings are documented at:
        # https://docs.python.org/3/library/struct.html,
        # so we need to look at GDAL documentation of raster data type names
        # and use raster band DataType to select the right format string.  
        raster_data_type_format = '<h'
        elevation = struct.unpack(raster_data_type_format, elevationstruct)[0]
        new_point = ogr.Geometry(ogr.wkbPoint)
        new_point.AddPoint(geom.GetX(), geom.GetY(), elevation)
        outFeature = ogr.Feature(featureDefn)
        outFeature.SetGeometry(new_point)
        outFeature.SetField('elevation', elevation)
        for fd in fields:
            val = feature.GetField(fd.name)
            outFeature.SetField(fd.name, val)
        outLayer.CreateFeature(outFeature)
    return 0

if __name__ == "__main__":
    p = argparse.ArgumentParser()

    p.add_argument("inraster", help="input DEM GeoTIFF raster file")
    p.add_argument("inpoints", help="input points geojson file")
    p.add_argument("outfile", help="output GeoJSON file")

    a = p.parse_args()
    
    writeout = addElevationFromDEM(a.inraster, a.inpoints, a.outfile)
