"""
Degrees to metres and back, for lengths and buffers.

Around Provo one degree of latitude is ~111 km and one of longitude ~84 km;
this flat-earth conversion is off by well under 1% across a city, and needs
no projection library.
"""
import numpy as np

LAT0, LON0 = 40.25, -111.67
M_PER_DEG_LAT = 111_132.0
M_PER_DEG_LON = 111_320.0 * np.cos(np.radians(LAT0))
METERS_PER_MILE = 1609.344


def to_metres(x, y, z=None):
    return (np.asarray(x) - LON0) * M_PER_DEG_LON, (np.asarray(y) - LAT0) * M_PER_DEG_LAT


def to_degrees(x, y, z=None):
    return np.asarray(x) / M_PER_DEG_LON + LON0, np.asarray(y) / M_PER_DEG_LAT + LAT0
