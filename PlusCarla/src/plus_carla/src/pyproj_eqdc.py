import warnings

from pyproj import Proj

REGIONS = {
    'NA': {'lat_1': 30, 'lat_2': 50},
    'EU': {'lat_1': 30, 'lat_2': 50},
    'AU': {'lat_1': -20, 'lat_2': -35},
    # Add more regions as necessary
}

def get_eqdc_proj_str(region, ellps='GRS80'):
    if region not in REGIONS:
        raise ValueError(f"Region '{region}' not supported.")
    params = REGIONS[region]
    return f'+proj=eqdc +lat_0=0 +lon_0=0 +lat_1={params["lat_1"]} +lat_2={params["lat_2"]} +x_0=0 +y_0=0 +ellps={ellps} +units=m +no_defs'

class EqdcConverter:
    def __init__(self, region=None, ellps='GRS80'):
        self.proj_str = None
        if region:
            self.configure(region, ellps)

    def configure(self, region, ellps='GRS80'):
        if region not in REGIONS:
            raise ValueError(f"Region '{region}' not supported.")
        self.proj_str = get_eqdc_proj_str(region, ellps)

    def from_latlon(self, lat, lon):
        if not self.proj_str:
            raise RuntimeError("EqdcConverter is not configured. Please call configure() method first.")
        proj = Proj(self.proj_str)
        return proj(lon, lat)

    def to_latlon(self, eqdc_x, eqdc_y):
        if not self.proj_str:
            raise RuntimeError("EqdcConverter is not configured. Please call configure() method first.")
        proj = Proj(self.proj_str)
        lon, lat = proj(eqdc_x, eqdc_y, inverse=True)
        return lat, lon


def from_latlon(lat, lon, region=None, ellps='GRS80'):
    if region is None:
        region = "NA"
        warnings.warn(f"Calling `from_latlon` without a `region` argument is deprecated. Will use default of '{region}'", DeprecationWarning)
    proj_str = get_eqdc_proj_str(region, ellps)
    proj = Proj(proj_str)
    eqdc_x, eqdc_y = proj(lon, lat)
    return eqdc_x, eqdc_y

def to_latlon(eqdc_x, eqdc_y, region=None, ellps='GRS80'):
    if region is None:
        region = "NA"
        warnings.warn(f"Calling `to_latlon` without a `region` argument is deprecated. Will use default of '{region}'", DeprecationWarning)
    proj_str = get_eqdc_proj_str(region, ellps)
    proj = Proj(proj_str)
    lon, lat = proj(eqdc_x, eqdc_y, inverse=True)
    return lat, lon
