import trimesh
from shapely.geometry.polygon import Polygon
from OSMPythonTools.overpass import Overpass #Use overpy if we want JSON instead (might be faster)
import overpy
#for elevation: either open-elevation POST using urllib or use elevation library
import requests
import os
import matplotlib.pyplot as plt

OUTPUT_DIR = "building_models"
EARTH_CIRCUMFERENCE = 40075 #km

DEG_PER_KM = 0.0089982311916 #approximation assuming earth perfect sphere

TARGET_COORD=(32.2319,-110.9501) # U of A
TARGET_RADIUS=0.5 # Square, spherical km from center

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

#query location #Note: overpass takes bounding box as (south,west,north,east)
result = overpass.query(f"""
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
scene = trimesh.scene.scene.Scene()
for element in result.elements():
    if element.type() == "way":
        coordinates = element.geometry()["coordinates"][0]
        # for coordinate in coordinates:
        #     print(coordinate)
        coordinates.append(coordinates[0])
        coordinates = [[deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)] for coord in coordinates]
        poly = Polygon(coordinates)
        mesh = trimesh.creation.extrude_polygon(poly, height=0.1)
        name = "unnamed"
        try:
            tags = element.tags()
            if tags != None:
                name = tags['name']
        except KeyError:
            pass
        mesh.export(f"{OUTPUT_DIR}/{name}_{element.id()}.stl")
        scene.add_geometry(mesh)
        x,y = zip(*coordinates)
        plt.plot(x,y)
scene.show()
plt.show()

#TODO: get elevation, building heights when available. Change default height to 1 story