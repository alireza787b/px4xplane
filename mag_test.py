import numpy as np
from scipy.spatial.transform import Rotation as R
import ppigrf
import pandas as pd

def get_magnetic_field_vector(lat, long, alt, roll, pitch, yaw):
    """
    This function simulates a magnetometer that measures the Earth's magnetic field vector at a given location and orientation in the body frame. The goal is to provide a simulated measurement of the Earth's magnetic field as it would be measured by a magnetometer on an aircraft or other vehicle.

    Parameters:
    lat (float): Latitude of the location in degrees.
    long (float): Longitude of the location in degrees.
    alt (float): Altitude of the location in meters.
    roll (float): Roll angle of the vehicle in degrees.
    pitch (float): Pitch angle of the vehicle in degrees.
    yaw (float): Heading (yaw angle) of the vehicle in degrees.

    Returns:
    B_body (numpy.ndarray): The Earth's magnetic field vector at the given location and orientation in the body frame, in Gauss.
    """
    
    # Convert angles from degrees to radians
    lat, long = np.radians([lat, long])
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    
    # Convert altitude from meters to kilometers
    alt = alt / 1000
    
    # Use ppigrf to get the Earth's magnetic field vector at the given location
    r = 6371 + alt  # Earth's radius plus altitude in km
    theta = 90 - np.degrees(lat)  # colatitude in degrees
    phi = np.degrees(long)  # longitude in degrees
    
    # Get current date and time
    date = pd.Timestamp.now()
    
    Br, Btheta, Bphi = ppigrf.igrf_gc(r, theta, phi, date=date)
    
    # Convert from spherical to Cartesian coordinates
    B_N = -Btheta.item()
    B_E = Bphi.item()
    B_D = -Br.item()
    
    print(f"Magnetic field vector in NED frame: {B_N},{B_E},{B_D}")

    # Create the rotation matrix using roll, pitch and yaw
    rotation = R.from_euler('ZYX', [yaw, pitch, roll])
    
    # Rotate the Earth's magnetic field vector from NED (North-East-Down) to body frame
    B_body = rotation.apply(np.array([B_N, B_E, B_D]))
    
    # Convert from nT to Gauss
    B_body = B_body / 100000

    return B_body


# Test case for Tehran Mehrabad Airport with level orientation
lat = 35.6891666667  # latitude of Tehran Mehrabad Airport in degrees
long = 51.3108333333  # longitude of Tehran Mehrabad Airport in degrees
alt = 1204  # altitude of Tehran Mehrabad Airport in meters
roll = 0  # roll angle in degrees
pitch = 0  # pitch angle in degrees
yaw = 0  # heading (yaw angle) in degrees

# Call the function with the test case
B_body = get_magnetic_field_vector(lat, long, alt, roll, pitch, yaw)

# Print the result
print(f"Magnetic field vector in body frame: {B_body}")
