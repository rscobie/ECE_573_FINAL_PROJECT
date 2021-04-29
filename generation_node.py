"""
note: mesh coordinates are in kilometers
"""

from numpy.lib.function_base import meshgrid
import trimesh
from shapely.geometry.polygon import Polygon
from OSMPythonTools.overpass import Overpass
from trimesh.base import Trimesh
from trimesh.transformations import translation_matrix #Use overpy if we want JSON instead (might be faster)
import elevation
from osgeo import gdal
import os
import matplotlib.pyplot as plt
import numpy as np
import math
from PIL import Image, ImageDraw
import networkx as nx

import rospy
from std_msgs.msg import Float32MultiArray, String

FILE_TYPE = "obj"

OUTPUT_DIR = os.path.dirname(os.path.realpath(__file__)) + "/models/chunks"
EARTH_CIRCUMFERENCE = 40075 #km

DEG_PER_KM = 0.0089982311916 #approximation assuming earth perfect sphere

TARGET_COORD=(32.2319,-110.9501) # U of A
#TARGET_COORD=(32.4387,-110.7598) # Summerhaven
#TARGET_COORD=(44.0717157763241, 7.254062280821728) # saint-martin-vesubie, France
#TARGET_COORD=(43.73930237228949, 7.420153277183919)#Monaco

TARGET_RADIUS=0.48 # km from center point, square. Should be multiple of 0.03 due to elevation dataset resolution

DEFAULT_BUILDING_HEIGHT = 0.006 #6 meters

ROAD_OFFSET = 0.0005 #roads will hover half a meter off the ground to avoid clipping in dramatic terrain

TEXTURE_RESOLUTION = 10000 #per kilometer

def km_to_deg(km):
    return km*DEG_PER_KM

def deg_to_km(deg):
    return deg/DEG_PER_KM

"""
#TODO: temporary solution while off-by-one bug fixed

Make this more general (by taking lat, lon instead of x,y) and use elsewhere in future, right now just used as bug fix
"""
def get_elevation(coord,elevations): 
    x_index = int(coord[0]/0.03)
    y_index = int(coord[1]/0.03)
    if int(coord[0]/0.03) > elevations.shape[0] - 1:
        x_index = elevations.shape[0] - 1
    if int(coord[1]/0.03) > elevations.shape[1] - 1:
        y_index = elevations.shape[1] - 1
    if int(coord[0]/0.03) < 0:
        x_index = 0
    if int(coord[1]/0.03) < 0:
        y_index = 0
    return elevations[x_index, y_index]

#******************************
#get buildings
#******************************
def gen_buildings(south_bound, west_bound, north_bound, east_bound, elevations, vertices, faces, uvs, texture):
    overpass = Overpass()
    #query location #Note: overpass takes bounding box as (south,west,north,east)
    building_result = overpass.query(f"""
    way["building"]({south_bound},{west_bound},{north_bound},{east_bound});
    out geom;
    relation["building"]({south_bound},{west_bound},{north_bound},{east_bound});out;
    way(r)[!"building:part"];
    out geom;
    """)
    tex_drawer = ImageDraw.Draw(texture)
    for element in building_result.elements():
        if element.type() == "way":
            coordinates = None
            try:
                coordinates = element.geometry()["coordinates"][0]
                coordinates.append(coordinates[0])
                coordinates = [(deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)) for coord in coordinates]
            except TypeError:
                continue # throw out building is formatting problem #TODO: come up with more graceful solution
            
            #update texture
            pixel_coords = list()
            for coord in coordinates:
                pixel_coords.append((coord[0]*TEXTURE_RESOLUTION,coord[1]*TEXTURE_RESOLUTION))
            tex_drawer.polygon(pixel_coords, fill=(255,0,0))
            
            poly = Polygon(coordinates)
            mesh = trimesh.creation.extrude_polygon(poly, height=DEFAULT_BUILDING_HEIGHT)
            
            #get lowest elevation (so that building not floating)
            min_elevation = 100000000000000
            for coord in coordinates:
                elevation = get_elevation(coord, elevations)
                if elevation < min_elevation:
                    min_elevation = elevation
            min_elevation /= 1000#meters to km
            
            #mesh.apply_translation((0,0,min_elevation))
            for i in range(len(mesh.vertices)):
                mesh.vertices[i][2] += min_elevation
            
            #concatenate to chunk
            old_index = len(vertices)
            vertices.extend(mesh.vertices)
            faces.extend([[y + old_index for y in x] for x in mesh.faces])
            
            #add uvs for buildings
            for vertex in mesh.vertices:
                uvs.append((vertex[0]/(texture.size[0]/TEXTURE_RESOLUTION),vertex[1]/(texture.size[0]/TEXTURE_RESOLUTION)))

    return vertices, faces, uvs, texture

#******************************
#get elevation
#******************************
def gen_elevation(south_bound, west_bound, north_bound, east_bound): #TODO: query an extra 30 meters on each side
    #get file from internet (of from local cache)
    file_name = f"{south_bound}_{west_bound}_{north_bound}_{east_bound}.tif"
    elevation.clip(bounds=(west_bound, south_bound, east_bound, north_bound),output=file_name,cache_dir=OUTPUT_DIR)
    data = gdal.Open(OUTPUT_DIR + "/SRTM1/" + file_name)
    band = data.GetRasterBand(1)
    elevations = band.ReadAsArray(0,0, band.XSize, band.YSize)
    #generate mesh
    points = list()
    triangles = list()
    uvs = list()
    #colors = list()#per vertex
    #generate vertices
    for i in range(elevations.shape[0]):
        for j in range(elevations.shape[1]):
            points.append((2*TARGET_RADIUS*i/elevations.shape[0], 2*TARGET_RADIUS*j/elevations.shape[1], elevations[i][j]/1000) )
            uvs.append((i/(elevations.shape[0]-1),j/(elevations.shape[1]-1)))#TODO: check for empty elevations list edge case?
            #colors.append((0,255,0))#green
    #generate faces
    for i in range(elevations.shape[0] - 1):
        for j in range(elevations.shape[1] -1):
            triangles.append( ( (i+1)*elevations.shape[1] + j + 1,i*elevations.shape[1] + j+1,i*elevations.shape[1] + j ) )
            triangles.append( ( (i+1)*elevations.shape[1] + j, (i+1)*elevations.shape[1] + j+1, i*elevations.shape[1] + j ) )
    tex_res = int(TEXTURE_RESOLUTION*0.03*elevations.shape[0])#actual resolution of this image given spatial size
    texture = Image.new('RGB', (tex_res,tex_res), (0,255,0))
    #terrain_mesh = trimesh.base.Trimesh(vertices=points,faces=triangles, vertex_colors=colors)

    return elevations, points, triangles, uvs, texture#terrain_mesh

#******************************
#get roads
#******************************
def gen_roads(south_bound, west_bound, north_bound, east_bound, elevations, vertices, faces, uvs, texture):
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
    
    road_graph = nx.Graph()
    tex_drawer = ImageDraw.Draw(texture)

    for element in road_result.elements():
        if element.type() == "way":
            coordinates = np.asarray(element.geometry()["coordinates"]).squeeze().tolist()
            coordinates = [(deg_to_km(coord[0] - west_bound), deg_to_km(coord[1] - south_bound)) for coord in coordinates]

            #TODO: cut roads that go out of bounds short
            temp_coords = list()

            name = "unnamed"
            try:
                tags = element.tags()
                if tags != None:
                    name = tags['name']
            except KeyError:
                pass
            x,y = zip(*coordinates)

            #paint roads onto texture
            pixel_coords = list()
            for coord in coordinates:
                pixel_coords.append((coord[0]*TEXTURE_RESOLUTION,coord[1]*TEXTURE_RESOLUTION))
            tex_drawer.line(pixel_coords, fill=(0,0,0), width=int(TEXTURE_RESOLUTION/100))#TODO: do I need to discretize this?
            #add roads to graph
            for i in range(len(x) - 1):
                elev1 = get_elevation([x[i],y[i]], elevations)/1000
                elev2 = get_elevation([x[i+1],y[i+1]], elevations)/1000
                road_graph.add_edge( (x[i],y[i],elev1),(x[i+1],y[i+1],elev2), weight=math.sqrt( (x[i] - x[i+1])**2 + (y[i] - y[i+1])**2 ) )#weighted edge to destination

    #TODO: process graph to combine nearby nodes, add nodes at intersections of lines, etc.
    return road_graph, vertices, faces, uvs, texture#chunk_model

def generate_chunk(lat,lon):
    try:
        os.mkdir(OUTPUT_DIR)
    except FileExistsError:
        pass

    south_bound = round(lat - km_to_deg(TARGET_RADIUS),4)
    north_bound = round(lat + km_to_deg(TARGET_RADIUS),4)
    east_bound = round(lon + km_to_deg(TARGET_RADIUS),4)
    west_bound = round(lon - km_to_deg(TARGET_RADIUS),4)
    elevations, vertices, faces, uvs, texture = gen_elevation(south_bound, west_bound, north_bound, east_bound)
    vertices, faces, uvs, texture = gen_buildings(south_bound, west_bound, north_bound, east_bound, elevations, vertices, faces, uvs, texture)
    road_graph, vertices, faces, uvs, texture = gen_roads(south_bound, west_bound, north_bound, east_bound, elevations, vertices, faces, uvs, texture)
    
    #flip image since pillow coordinate system different
    texture = texture.transpose(Image.FLIP_TOP_BOTTOM)
    tex_vis = trimesh.visual.texture.TextureVisuals(uv=uvs,image=texture)
    chunk_model = trimesh.base.Trimesh(vertices=vertices,faces=faces,visual=tex_vis)
    
    chunk_name = f"{south_bound}_{west_bound}_{north_bound}_{east_bound}"
    model_dir = f"{OUTPUT_DIR}/{chunk_name}"
    try:
        os.mkdir(model_dir)
    except FileExistsError:
        pass
    model_path = f"{model_dir}/{chunk_name}.{FILE_TYPE}"
    graph_path = f"{model_dir}/{chunk_name}.{'roads'}"
    
    fp = open(graph_path, "w")
    fp.close()
    nx.readwrite.gpickle.write_gpickle(road_graph, graph_path)
    chunk_model.export(model_path)
    return chunk_name

if __name__ == "__main__":
    publisher = rospy.Publisher("chunk_path", String, queue_size=10)

    def callback(array):
        lat = array.data[0]
        lon = array.data[1]
        pub_val = String()
        pub_val.data = generate_chunk(lat, lon)
        publisher.publish(pub_val)

    subscriber = rospy.Subscriber("chunk_coordinate", Float32MultiArray, callback, queue_size=10) #1d arrays of size 2
    rospy.init_node("generation_node")
    #os.chdir("./src/ECE_573_FINAL_PROJECT")#move us into package, ros starts us off in workspace root ece573_ws
    rospy.spin()

    """
    TODO: refactor: clean up magic numbers (to be more explicit with units), add function to get single elevation at point
    TODO: elevation locations off by 1 sample (30m) in both directions, currently a workaround in place
    TODO: some building formats not supported, messes up parsing of coordinate data
    TODO: find elegent way to cut off ways (buildings/roads) that are partially outside of terrain bounds (left in place for now, using last known elevation)
    """