"""
note: mesh coordinates are in kilometers
"""

from numpy.lib.function_base import meshgrid
import trimesh
from shapely.geometry.polygon import Polygon
from OSMPythonTools.overpass import Overpass
from trimesh.transformations import translation_matrix #Use overpy if we want JSON instead (might be faster)
import elevation
from osgeo import gdal
import os
import matplotlib.pyplot as plt
import numpy as np
import math

#FILE_TYPE = ".stl"
FILE_TYPE = ".ply" #using this now since it preserves colors/textures and scale

OUTPUT_DIR = "cache"
EARTH_CIRCUMFERENCE = 40075 #km

DEG_PER_KM = 0.0089982311916 #approximation assuming earth perfect sphere

#TARGET_COORD=(32.2319,-110.9501) # U of A
#TARGET_COORD=(32.4387,-110.7598) # Summerhaven
#TARGET_COORD=(44.0717157763241, 7.254062280821728) # saint-martin-vesubie, France
TARGET_COORD=(43.73930237228949, 7.420153277183919)#Monaco

TARGET_RADIUS=0.48 # km from center point, square. Should be multiple of 0.03 due to elevation dataset resolution

DEFAULT_BUILDING_HEIGHT = 0.006 #6 meters

ROAD_OFFSET = 0.0005 #roads will hover half a meter off the ground to avoid clipping in dramatic terrain

# def coord_to_xy(lat, lon): #in km
#     x = (TARGET_RADIUS*2)*(180+lon)/360
#     y = (TARGET_RADIUS*2)*(90-lat)/180
#     return(x,y)

def km_to_deg(km):
    return km*DEG_PER_KM

def deg_to_km(deg):
    return deg/DEG_PER_KM

#debug
my_scene = trimesh.scene.scene.Scene()

#******************************
#get buildings
#******************************
def get_buildings(south_bound, west_bound, north_bound, east_bound, elevations):
    #debug
    global my_scene

    overpass = Overpass()
    #query location #Note: overpass takes bounding box as (south,west,north,east)
    building_result = overpass.query(f"""
    way["building"]({south_bound},{west_bound},{north_bound},{east_bound});
    out geom;
    relation["building"]({south_bound},{west_bound},{north_bound},{east_bound});out;
    way(r)[!"building:part"];
    out geom;
    """)
    try:
        os.mkdir(OUTPUT_DIR)
    except FileExistsError:
        pass
    #building_scene = trimesh.scene.scene.Scene()
    for element in building_result.elements():
        if element.type() == "way":
            coordinates = None
            try:
                coordinates = element.geometry()["coordinates"][0]
                coordinates.append(coordinates[0])
                coordinates = [[deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)] for coord in coordinates]
            except TypeError:
                continue # throw out building is formatting problem #TODO: come up with more graceful solution
            poly = Polygon(coordinates)
            mesh = trimesh.creation.extrude_polygon(poly, height=DEFAULT_BUILDING_HEIGHT)
            
            #get lowest elevation (so that building not floating)
            min_elevation = 100000000000000
            for coord in coordinates:
                x_index = int(coord[0]/0.03)
                y_index = int(coord[1]/0.03)
                if int(coord[0]/0.03) > elevations.shape[0] - 1: #TODO: temporary solution while off-by-one bug fixed
                    x_index = elevations.shape[0] - 1
                if int(coord[1]/0.03) > elevations.shape[1] - 1:
                    y_index = elevations.shape[1] - 1
                if int(coord[0]/0.03) < 0:
                    x_index = 0
                if int(coord[1]/0.03) < 0:
                    y_index = 0
                elevation = elevations[x_index,y_index]
                if elevation < min_elevation:
                    min_elevation = elevation
            min_elevation /= 1000#meters to km
            
            #mesh.apply_translation((0,0,min_elevation))
            for i in range(len(mesh.vertices)):
                mesh.vertices[i][2] += min_elevation
            
            name = "unnamed"
            try:
                tags = element.tags()
                if tags != None:
                    name = tags['name']
            except KeyError:
                pass
            mesh.export(f"{OUTPUT_DIR}/{name}_{element.id()}.{FILE_TYPE}")
            #building_scene.add_geometry(mesh)
            my_scene.add_geometry(mesh)
            x,y = zip(*coordinates)
            plt.plot(x,y)
    #building_scene.show()
    #plt.show()

#******************************
#get elevation
#******************************
def get_elevation(south_bound, west_bound, north_bound, east_bound):
    #debug
    global my_scene
    #get file from internet (of from local cache)
    file_name = f"{south_bound}_{west_bound}_{north_bound}_{east_bound}.tif"
    elevation.clip(bounds=(west_bound, south_bound, east_bound, north_bound),output=file_name,cache_dir=OUTPUT_DIR)
    data = gdal.Open(OUTPUT_DIR + "/SRTM1/" + file_name)
    band = data.GetRasterBand(1)
    elevations = band.ReadAsArray(0,0, band.XSize, band.YSize)
    #generate mesh
    points = list()
    triangles = list()
    #generate vertices
    for i in range(elevations.shape[0]):
        for j in range(elevations.shape[1]):
            points.append((2*TARGET_RADIUS*i/elevations.shape[0], 2*TARGET_RADIUS*j/elevations.shape[1], elevations[i][j]/1000) )
    #generate faces
    for i in range(elevations.shape[0] - 1):
        for j in range(elevations.shape[1] -1):
            triangles.append( ( (i+1)*elevations.shape[1] + j + 1,i*elevations.shape[1] + j+1,i*elevations.shape[1] + j ) )
            triangles.append( ( (i+1)*elevations.shape[1] + j, (i+1)*elevations.shape[1] + j+1, i*elevations.shape[1] + j ) )

    terrain_mesh = trimesh.base.Trimesh(vertices=points,faces=triangles)
    terrain_mesh.export(f"{OUTPUT_DIR}/{south_bound}_{west_bound}_{north_bound}_{east_bound}.{FILE_TYPE}")
    my_scene.add_geometry(terrain_mesh)
    #terrain_mesh.show()
    return elevations


#******************************
#get roads
#******************************
def get_roads(south_bound, west_bound, north_bound, east_bound, elevations):
    #debug
    global my_scene
    overpass = Overpass()
    #query location #Note: overpass takes bounding box as (south,west,north,east)
    road_result = overpass.query(f"""
    (
    way[highway=motorway]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=motorway_link]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=trunk]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=trunk_link]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=primary]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=secondary]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=tertiary]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=unclassified]({south_bound},{west_bound},{north_bound},{east_bound});
    way[highway=residential]({south_bound},{west_bound},{north_bound},{east_bound});
    );
    out geom;

    """)
    try:
        os.mkdir(OUTPUT_DIR)
    except FileExistsError:
        pass
    road_scene = trimesh.scene.scene.Scene()
    for element in road_result.elements():
        if element.type() == "way":
            coordinates = np.asarray(element.geometry()["coordinates"]).squeeze().tolist()
            coordinates = [[deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)] for coord in coordinates]

            #cut roads that go out of bounds short
            temp_coords = list()
            for coordinate in coordinates:
                if coordinate[0] < 0:
                    coordinate[0] = 0
                if coordinate[0] > 2*(TARGET_RADIUS - 0.03): #TODO: the -0.03 is a temporary solution while the off-by-1 elevation bug is investigated
                    coordinate[0] = 2*(TARGET_RADIUS - 0.03)
                if coordinate[1] < 0:
                    coordinate[1] = 0
                if coordinate[1] > 2*(TARGET_RADIUS - 0.03):
                    coordinate[1] = 2*(TARGET_RADIUS - 0.03)

            name = "unnamed"
            try:
                tags = element.tags()
                if tags != None:
                    name = tags['name']
            except KeyError:
                pass
            x,y = zip(*coordinates)
            #make meshes from roads
            points=list()
            triangles=list()
            for i in range(len(x) - 1):
                #find perpendicular points
                u = (x[i] - x[i+1],y[i] - y[i+1])#road vector
                length = 0
                if u[0] == 0 and u[1] == 0:#TODO: temporary fix while the off-by-1 elevation bug investigated
                    length = 0
                else:
                    length = 0.005/math.sqrt(u[0]**2+u[1]**2) #make points 5 meters away from road center
                v1 = (-1*u[1]*length,u[0]*length)
                v2 = (u[1]*length,-1*u[0]*length)
                points.append( (x[i]+v1[0],y[i]+v1[1], elevations[int((x[i]+v1[0])/0.03),int((y[i]+v1[1])/0.03)]/1000 + ROAD_OFFSET) )
                points.append( (x[i]+v2[0],y[i]+v2[1], elevations[int((x[i]+v2[0])/0.03),int((y[i]+v2[1])/0.03)]/1000 + ROAD_OFFSET) )
                points.append( (x[i+1]+v1[0],y[i+1]+v1[1], elevations[int((x[i+1]+v1[0])/0.03),int((y[i+1]+v1[1])/0.03)]/1000 + ROAD_OFFSET) )
                points.append( (x[i+1]+v2[0],y[i+1]+v2[1], elevations[int((x[i+1]+v2[0])/0.03),int((y[i+1]+v2[1])/0.03)]/1000 + ROAD_OFFSET) )
            for i in range(len(x) - 1):
                triangles.append( (4*i + 0, 4*i + 1, 4*i + 3) )
                triangles.append( (4*i + 0, 4*i + 3, 4*i + 2) )
                #add second set of triangles with opposite surface normals, to avoid
                #accidental invisible roads
                triangles.append( (4*i + 0, 4*i + 3, 4*i + 1) )
                triangles.append( (4*i + 0, 4*i + 2, 4*i + 3) )
            mesh = trimesh.base.Trimesh(vertices=points, faces=triangles)
            #mesh.show()
            #road_scene.add_geometry(mesh)
            my_scene.add_geometry(mesh)
            mesh.export(f"{OUTPUT_DIR}/{name}_{element.id()}.{FILE_TYPE}")
            #plt.plot(x,y)
    #road_scene.show()
    #plt.show()

def generate_chunk(lat,lon):
    south_bound = round(TARGET_COORD[0] - km_to_deg(TARGET_RADIUS),4)
    north_bound = round(TARGET_COORD[0] + km_to_deg(TARGET_RADIUS),4)
    east_bound = round(TARGET_COORD[1] + km_to_deg(TARGET_RADIUS),4)
    west_bound = round(TARGET_COORD[1] - km_to_deg(TARGET_RADIUS),4)
    elevations = get_elevation(south_bound, west_bound, north_bound, east_bound)
    get_buildings(south_bound, west_bound, north_bound, east_bound, elevations)
    get_roads(south_bound, west_bound, north_bound, east_bound, elevations)

if __name__ == "__main__":
    generate_chunk(TARGET_COORD[0], TARGET_COORD[1])
    my_scene.show()

    """
    TODO: refactor: clean up magic numbers (to be more explicit with units), add function to get single elevation at point
    TODO: elevation locations off by 1 sample (30m) in both directions, currently a workaround in place
    TODO: some building formats not supported, messes up parsing of coordinate data
    TODO: wrap up in ROS node, add topics that take commands, send file paths for models

    maybe------------
    TODO: make entire chunk a single model, just union all the individual meshes together. Then only need to save one file, and send one file path to simulation node
    """