import trimesh
from shapely.geometry.polygon import Polygon
from OSMPythonTools.overpass import Overpass #Use overpy if we want JSON instead (might be faster)
import overpy
#for elevation: either open-elevation POST using urllib or use elevation library
import requests, json
import os
import matplotlib.pyplot as plt
import numpy as np

OUTPUT_DIR = "building_models"
EARTH_CIRCUMFERENCE = 40075 #km

DEG_PER_KM = 0.0089982311916 #approximation assuming earth perfect sphere

TARGET_COORD=(32.2319,-110.9501) # U of A
TARGET_RADIUS=0.5 # Square, spherical km from center

DEFAULT_BUILDING_HEIGHT = 0.006

ELEVATION_ENDPOINT = "https://www.api.open-elevation.com/api/v1/lookup"
ELEVATION_RESOLUTION = 1

overpass = Overpass()

# def coord_to_xy(lat, lon): #in km
#     x = (TARGET_RADIUS*2)*(180+lon)/360
#     y = (TARGET_RADIUS*2)*(90-lat)/180
#     return(x,y)

def km_to_deg(km):
    return km*DEG_PER_KM

def deg_to_km(deg):
    return deg/DEG_PER_KM

south_bound = round(TARGET_COORD[0] - km_to_deg(TARGET_RADIUS),4)
north_bound = round(TARGET_COORD[0] + km_to_deg(TARGET_RADIUS),4)
east_bound = round(TARGET_COORD[1] + km_to_deg(TARGET_RADIUS),4)
west_bound = round(TARGET_COORD[1] - km_to_deg(TARGET_RADIUS),4)

#******************************
#get buildings
#******************************
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
building_scene = trimesh.scene.scene.Scene()
for element in building_result.elements():
    if element.type() == "way":
        coordinates = element.geometry()["coordinates"][0]
        # for coordinate in coordinates:
        #     print(coordinate)
        coordinates.append(coordinates[0])
        coordinates = [[deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)] for coord in coordinates]
        poly = Polygon(coordinates)
        mesh = trimesh.creation.extrude_polygon(poly, height=DEFAULT_BUILDING_HEIGHT)
        name = "unnamed"
        try:
            tags = element.tags()
            if tags != None:
                name = tags['name']
        except KeyError:
            pass
        mesh.export(f"{OUTPUT_DIR}/{name}_{element.id()}.stl")
        building_scene.add_geometry(mesh)
        x,y = zip(*coordinates)
        plt.plot(x,y)
building_scene.show()
plt.show()

#******************************
#get elevation
#******************************
#make json object to post
query_dict = dict()
query_dict["locations"] = list()
for i in range(ELEVATION_RESOLUTION + 1):
    for j in range(ELEVATION_RESOLUTION + 1):
        new_coord = dict()
        new_coord["latitude"] = south_bound + (north_bound - south_bound)*(i/ELEVATION_RESOLUTION)
        new_coord["longitude"] = west_bound + (east_bound - west_bound)*(j/ELEVATION_RESOLUTION)
        query_dict["locations"].append(new_coord)
query_coords = json.dumps(query_dict).replace("'",'"')
print(query_coords)
elevations = requests.post(ELEVATION_ENDPOINT, json=query_coords, headers={"Accept":"application/json", "Content-Type":"application/json"})
# print(query_dict)
# elevations = requests.post(ELEVATION_ENDPOINT, data=query_dict)
print(elevations.content)
#******************************
#get roads
#******************************
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
#road_scene = trimesh.scene.scene.Scene()
for element in road_result.elements():
    if element.type() == "way":
        coordinates = np.asarray(element.geometry()["coordinates"]).squeeze().tolist()
        # for coordinate in coordinates:
        #     print(coordinate)
        coordinates = [[deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)] for coord in coordinates]
        #poly = Polygon(coordinates)
        #mesh = trimesh.creation.extrude_polygon(poly, height=DEFAULT_BUILDING_HEIGHT)
        name = "unnamed"
        try:
            tags = element.tags()
            if tags != None:
                name = tags['name']
        except KeyError:
            pass
        #mesh.export(f"{OUTPUT_DIR}/{name}_{element.id()}.stl")
        #road_scene.add_geometry(mesh)
        x,y = zip(*coordinates)
        plt.plot(x,y)
#road_scene.show()
plt.show()