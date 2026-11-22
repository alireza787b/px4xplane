#pragma once

#include "ConfigManager.h"
#include "XPLMCamera.h"

#include <Eigen/Geometry>
#include <cmath>

namespace CameraPose {
namespace detail {

constexpr double pi = 3.14159265358979323846;
constexpr double radiansPerDegree = pi / 180.0;

inline Eigen::Quaterniond rotation(double headingDeg, double pitchDeg, double rollDeg)
{
    // XPLM: clockwise heading about +Y, pitch up about +X, roll right about -Z.
    // https://developer.x-plane.com/article/screencoordinates/
    return Eigen::AngleAxisd(-headingDeg * radiansPerDegree, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(pitchDeg * radiansPerDegree, Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(-rollDeg * radiansPerDegree, Eigen::Vector3d::UnitZ());
}

} // namespace detail

inline XPLMCameraPosition_t calculate(const Eigen::Vector3d& aircraftPosition,
                                     double headingDeg, double pitchDeg, double rollDeg,
                                     const CameraViewConfig& view)
{
    const Eigen::Quaterniond aircraftRotation = detail::rotation(headingDeg, pitchDeg, rollDeg);
    const Eigen::Vector3d bodyOffset(view.rightOffsetM, view.upOffsetM, -view.forwardOffsetM);
    const Eigen::Vector3d position = aircraftPosition + aircraftRotation * bodyOffset;

    const Eigen::Quaterniond mountRotation = detail::rotation(
        view.headingOffsetDeg, view.pitchOffsetDeg, view.rollOffsetDeg);
    const Eigen::Matrix3d cameraRotation = (aircraftRotation * mountRotation).toRotationMatrix();
    // Eigen's Y-X-Z decomposition uses right-handed axes; XPLM negates Y and Z.
    // https://developer.x-plane.com/sdk/XPLMCamera/
    const Eigen::Vector3d angles = cameraRotation.eulerAngles(1, 0, 2);
    double heading = -angles[0];
    double pitch = angles[1];
    double roll = -angles[2];

    // Choose the equivalent representation with pitch in [-90, 90] degrees.
    if (pitch > detail::pi / 2.0) {
        pitch = detail::pi - pitch;
        heading += detail::pi;
        roll += detail::pi;
    } else if (pitch < -detail::pi / 2.0) {
        pitch = -detail::pi - pitch;
        heading += detail::pi;
        roll += detail::pi;
    }

    XPLMCameraPosition_t result{};
    result.x = static_cast<float>(position.x());
    result.y = static_cast<float>(position.y());
    result.z = static_cast<float>(position.z());
    result.pitch = static_cast<float>(pitch / detail::radiansPerDegree);
    result.heading = static_cast<float>(std::fmod(heading / detail::radiansPerDegree + 360.0, 360.0));
    if (result.heading >= 360.0f) {
        result.heading = 0.0f;
    }
    result.roll = static_cast<float>(std::remainder(roll / detail::radiansPerDegree, 360.0));
    result.zoom = view.zoom;
    return result;
}

} // namespace CameraPose
