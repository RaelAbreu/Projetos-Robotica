import math

WGS84_A = 6378137.0                   
WGS84_F = 1.0 / 298.257223563          
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)   


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)   
    x = (n + alt) * cl * math.cos(lon)
    y = (n + alt) * cl * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt) * sl
    return x, y, z


class ENUConverter:
    def __init__(self, lat0_deg: float, lon0_deg: float, alt0: float = 0.0):
        self.lat0, self.lon0, self.alt0 = lat0_deg, lon0_deg, alt0
        self._x0, self._y0, self._z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
        lat, lon = math.radians(lat0_deg), math.radians(lon0_deg)
        self._sl, self._cl = math.sin(lat), math.cos(lat)
        self._so, self._co = math.sin(lon), math.cos(lon)

    def to_enu(self, lat_deg: float, lon_deg: float, alt: float):
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
        dx, dy, dz = x - self._x0, y - self._y0, z - self._z0
        e = -self._so * dx + self._co * dy
        n = (-self._sl * self._co * dx - self._sl * self._so * dy + self._cl * dz)
        u = (self._cl * self._co * dx + self._cl * self._so * dy + self._sl * dz)
        return e, n, u


def enu_to_gazebo_local(e: float, n: float, heading_deg: float):
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    return -(c * e - s * n), -(s * e + c * n)
