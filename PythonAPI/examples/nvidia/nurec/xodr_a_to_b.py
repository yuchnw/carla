from pyproj import Transformer
import pyproj_eqdc
from pyproj import Proj
import math

proj_a = ("+proj=eqc +lat_ts=37.98634149 +lat_0=37.98634149 "
          "+lon_0=-121.99173067 +datum=WGS84 +units=m +no_defs")

proj_b = ("+proj=eqdc +lat_0=0 +lon_0=0 +lat_1=30 +lat_2=50 "
          "+x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs")

OFFSET_X = -7588916.727497556
OFFSET_Y =  10353498.911958147
OFFSET_Z =  15.63076114654541

EQDC_CONVERTER = pyproj_eqdc.EqdcConverter()
EQDC_CONVERTER.configure('NA', 'WGS84')

# Transformers (always_xy => argument order is (x, y) / (lon, lat))
a_to_wgs = Transformer.from_crs(proj_a, "EPSG:4326", always_xy=True)
wgs_to_a = Transformer.from_crs("EPSG:4326", proj_a, always_xy=True)
b_to_wgs = Transformer.from_crs(proj_b, "EPSG:4326", always_xy=True)
wgs_to_b = Transformer.from_crs("EPSG:4326", proj_b, always_xy=True)

proj_b_str = ("+proj=eqdc +lat_0=0 +lon_0=0 +lat_1=30 +lat_2=50 "
              "+x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs")
proj_b = Proj(proj_b_str)

def gamma_b_rad(lat, lon):
    # meridian_convergence is in degrees; +ve means grid-north is east of true-north
    gamma_deg = proj_b.get_factors(lon, lat).meridian_convergence
    return math.radians(gamma_deg)

def map_a_to_latlon(x_a, y_a):
    lon, lat = a_to_wgs.transform(x_a, y_a)
    heading_a_to_b(math.radians(43.4), lat, lon)
    return lat, lon

def latlon_to_map_b(lat, lon):
    # x_b_global, y_b_global = wgs_to_b.transform(lon, lat)
    eqdc_x, eqdc_y = EQDC_CONVERTER.from_latlon(lat, lon)
    x = eqdc_x - OFFSET_X
    y = eqdc_y - OFFSET_Y
    return x, y

def map_b_to_latlon(x_b, y_b):
    lon, lat = b_to_wgs.transform(x_b + OFFSET_X, y_b + OFFSET_Y)
    return lat, lon

def latlon_to_map_a(lat, lon):
    return wgs_to_a.transform(lon, lat)

def map_a_to_map_b(x_a, y_a, z_a=0.0):
    lat, lon = map_a_to_latlon(x_a, y_a)
    x_b, y_b = latlon_to_map_b(lat, lon)
    return x_b, y_b, z_a - OFFSET_Z, lat, lon

def heading_a_to_b(psi_a, lat, lon, h_b=0.0):
    gamma_b = gamma_b_rad(lat, lon)
    psi_b = psi_a - gamma_b - h_b
    # normalize to (-pi, pi]
    new_psi = (psi_b + math.pi) % (2 * math.pi) - math.pi
    print(new_psi)
    return new_psi

# Example: the origin of Map A
x_b, y_b, z_b, lat, lon = map_a_to_map_b(-34.89078903198242, 32.10968780517578, 0.0)
print(f"lat, lon    = {lat:.8f}, {lon:.8f}")
print(f"Map B (x,y) = {x_b:.3f}, {y_b:.3f}")