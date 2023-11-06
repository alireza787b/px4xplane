#include <iostream>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <Eigen/Dense>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Eigen::Vector3d get_magnetic_field_vector(double lat, double lon, double alt, double roll, double pitch, double yaw) {
    // Convert angles from degrees to radians
    lat = lat * M_PI / 180;
    lon = lon * M_PI / 180;
    roll = roll * M_PI / 180;
    pitch = pitch * M_PI / 180;
    yaw = yaw * M_PI / 180;

    // Convert altitude from meters to kilometers
    alt = alt / 1000;

    // Use GeographicLib to get the Earth's magnetic field vector at the given location
    const GeographicLib::MagneticModel mag("wmm2020");
    double Bx, By, Bz;
    double year = 2023;  // The year for which the magnetic field is calculated
    mag(lat, lon, alt, year, Bx, By, Bz);

    // Convert from spherical to Cartesian coordinates
    double B_N = -Bx;
    double B_E = By;
    double B_D = -Bz;

    // Create the rotation matrix using roll, pitch and yaw
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d rotationMatrix = q.matrix();

    // Rotate the Earth's magnetic field vector from NED (North-East-Down) to body frame
    Eigen::Vector3d B_body = rotationMatrix * Eigen::Vector3d(B_N, B_E, B_D);

    // Convert from nT to Gauss
    B_body = B_body / 100000;

    return B_body;
}

int main() {
    // Test case for Tehran Mehrabad Airport with level orientation
    double lat = 35.6891666667;  // latitude of Tehran Mehrabad Airport in degrees
    double lon = 51.3108333333;  // longitude of Tehran Mehrabad Airport in degrees
    double alt = 1204;  // altitude of Tehran Mehrabad Airport in meters
    double roll = 0;  // roll angle in degrees
    double pitch = 0;  // pitch angle in degrees
    double yaw = 70;  // heading (yaw angle) in degrees

    // Call the function with the test case
    Eigen::Vector3d B_body = get_magnetic_field_vector(lat, lon, alt, roll, pitch, yaw);

    // Print the result
    std::cout << "Magnetic field vector in body frame: " << B_body.transpose() << std::endl;

    return 0;
}
