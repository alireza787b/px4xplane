#include "CameraPose.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;
const Eigen::Vector3d origin(12.25, 50.5, -42.75);

void expectNear(double actual, double expected, double tolerance = 2e-6)
{
    assert(std::isfinite(actual));
    assert(std::fabs(actual - expected) < tolerance);
}

void expectVectorNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
                      double tolerance = 2e-6)
{
    for (int axis = 0; axis < 3; ++axis) {
        expectNear(actual[axis], expected[axis], tolerance);
    }
}

Eigen::Vector3d positionOf(const XPLMCameraPosition_t& pose)
{
    return Eigen::Vector3d(pose.x, pose.y, pose.z);
}

// Independent scalar oracle: Laminar's aircraft-to-local equations, not the
// production quaternion/decomposition helpers.
// https://developer.x-plane.com/article/screencoordinates/
Eigen::Vector3d referenceRotate(const Eigen::Vector3d& point,
                                double headingDeg, double pitchDeg, double rollDeg)
{
    const double psi = headingDeg * radiansPerDegree;
    const double theta = pitchDeg * radiansPerDegree;
    const double phi = rollDeg * radiansPerDegree;
    const double xPhi = point.x() * std::cos(phi) + point.y() * std::sin(phi);
    const double yPhi = point.y() * std::cos(phi) - point.x() * std::sin(phi);
    const double yTheta = yPhi * std::cos(theta) - point.z() * std::sin(theta);
    const double zTheta = point.z() * std::cos(theta) + yPhi * std::sin(theta);
    return Eigen::Vector3d(xPhi * std::cos(psi) - zTheta * std::sin(psi),
                           yTheta,
                           zTheta * std::cos(psi) + xPhi * std::sin(psi));
}

void expectAxes(const XPLMCameraPosition_t& pose, const Eigen::Vector3d& right,
                const Eigen::Vector3d& up, const Eigen::Vector3d& forward)
{
    expectVectorNear(referenceRotate(Eigen::Vector3d::UnitX(), pose.heading, pose.pitch, pose.roll), right);
    expectVectorNear(referenceRotate(Eigen::Vector3d::UnitY(), pose.heading, pose.pitch, pose.roll), up);
    expectVectorNear(referenceRotate(-Eigen::Vector3d::UnitZ(), pose.heading, pose.pitch, pose.roll), forward);
}

void testLevelAndCardinalTranslations()
{
    const CameraViewConfig view{"Forward", 2.0f, 3.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.9f};
    const double headings[] = {0.0, 90.0, 180.0, 270.0, -90.0, 450.0};
    const Eigen::Vector3d offsets[] = {
        {3.0, 4.0, -2.0}, {2.0, 4.0, 3.0}, {-3.0, 4.0, 2.0},
        {-2.0, 4.0, -3.0}, {-2.0, 4.0, -3.0}, {2.0, 4.0, 3.0}
    };
    for (int i = 0; i < 6; ++i) {
        const auto pose = CameraPose::calculate(origin, headings[i], 0.0, 0.0, view);
        expectVectorNear(positionOf(pose), origin + offsets[i]);
        expectNear(pose.zoom, view.zoom);
        assert(pose.heading >= 0.0f && pose.heading < 360.0f);
    }
    const auto pose = CameraPose::calculate(origin, 0.0, 0.0, 0.0, CameraViewConfig{});
    expectVectorNear(positionOf(pose), origin);
    expectAxes(pose, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0});
    expectNear(pose.zoom, 1.0);
}

void testTiltedTranslations()
{
    const CameraViewConfig view{"Mount", 2.0f, 3.0f, 4.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    expectVectorNear(positionOf(CameraPose::calculate(origin, 0.0, 90.0, 0.0, view)),
                     origin + Eigen::Vector3d(3.0, 2.0, 4.0));
    expectVectorNear(positionOf(CameraPose::calculate(origin, 0.0, -90.0, 0.0, view)),
                     origin + Eigen::Vector3d(3.0, -2.0, -4.0));
    expectVectorNear(positionOf(CameraPose::calculate(origin, 0.0, 0.0, 90.0, view)),
                     origin + Eigen::Vector3d(4.0, -3.0, -2.0));
    expectVectorNear(positionOf(CameraPose::calculate(origin, 0.0, 0.0, -90.0, view)),
                     origin + Eigen::Vector3d(-4.0, 3.0, -2.0));
}

void testMinus90Plus90Mount()
{
    // Existing FPV preset: the level look must not redirect the forward mount.
    const CameraViewConfig view{"FPV", 0.85f, 0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.9f};
    for (double heading : {0.0, 37.0, 270.0}) {
        for (double roll : {-45.0, 0.0, 30.0, 90.0}) {
            const auto pose = CameraPose::calculate(origin, heading, -90.0, roll, view);
            expectNear(pose.pitch, 0.0);
            expectNear(pose.roll, 0.0);
            const double expectedHeading = std::fmod(heading + roll + 360.0, 360.0);
            expectNear(pose.heading, expectedHeading);
            expectVectorNear(positionOf(pose), origin + Eigen::Vector3d(0.0, -0.85f, 0.0));
            expectAxes(pose,
                referenceRotate(Eigen::Vector3d::UnitX(), expectedHeading, 0.0, 0.0),
                {0.0, 1.0, 0.0},
                referenceRotate(-Eigen::Vector3d::UnitZ(), expectedHeading, 0.0, 0.0));
        }
    }
}

void testRollComposition()
{
    const CameraViewConfig view{"Tilted", 0.0f, 0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 1.0f};
    const auto pose = CameraPose::calculate(origin, 0.0, 0.0, 90.0, view);
    expectAxes(pose, {0.0, -1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
}

void testBodyUpNoseMount()
{
    const CameraViewConfig view{"Body-up nose", 0.0f, 0.0f, 0.85f, 90.0f, 0.0f, 0.0f, 0.9f};
    const auto hover = CameraPose::calculate(origin, 0.0, 0.0, 0.0, view);
    expectVectorNear(positionOf(hover), origin + Eigen::Vector3d(0.0, 0.85f, 0.0));
    expectAxes(hover, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});

    const auto forwardFlight = CameraPose::calculate(origin, 0.0, -90.0, 0.0, view);
    expectVectorNear(positionOf(forwardFlight), origin + Eigen::Vector3d(0.0, 0.0, -0.85f));
    expectAxes(forwardFlight, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0});

    const auto banked = CameraPose::calculate(origin, 0.0, -90.0, 30.0, view);
    expectVectorNear(positionOf(banked),
                     origin + Eigen::Vector3d(0.85f / 2.0, 0.0, -0.85f * std::sqrt(3.0) / 2.0));
    expectNear(banked.heading, 30.0);
    expectNear(banked.pitch, 0.0);
    expectNear(banked.roll, 0.0);
}

void testTranslationIndependentOfLook()
{
    CameraViewConfig view{"Mount", -11.0f, 1.25f, 2.0f, 0.0f, 0.0f, 0.0f, 0.7f};
    const auto forward = CameraPose::calculate(origin, 43.0, -57.0, 38.0, view);
    view.pitchOffsetDeg = 90.0f;
    view.headingOffsetDeg = 125.0f;
    view.rollOffsetDeg = -73.0f;
    view.zoom = 2.0f;
    const auto tilted = CameraPose::calculate(origin, 43.0, -57.0, 38.0, view);
    assert(forward.x == tilted.x && forward.y == tilted.y && forward.z == tilted.z);
    expectNear(tilted.zoom, 2.0);

    view.forwardOffsetM = 0.0f;
    view.rightOffsetM = 0.0f;
    view.upOffsetM = 0.0f;
    const auto centered = CameraPose::calculate(origin, 43.0, -57.0, 38.0, view);
    assert(centered.heading == tilted.heading);
    assert(centered.pitch == tilted.pitch);
    assert(centered.roll == tilted.roll);
    expectVectorNear(positionOf(centered), origin);
}

int testAgainstSdkEquations()
{
    const CameraViewConfig mounts[] = {
        {"Aligned", 2.0f, 1.25f, 0.7f, 0.0f, 0.0f, 0.0f, 0.9f},
        {"FPV", 0.85f, 0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.9f},
        {"Body-up nose", 0.0f, 0.0f, 0.85f, 90.0f, 0.0f, 0.0f, 0.9f},
        {"Down", 0.4f, 0.0f, -0.55f, -90.0f, 0.0f, 0.0f, 0.85f},
        {"Right", 0.0f, 1.0f, 0.0f, 0.0f, 90.0f, 0.0f, 1.0f},
        {"Rolled", 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 90.0f, 1.0f},
        {"Compound", -11.0f, 1.25f, 2.0f, 14.0f, 27.0f, -39.0f, 0.7f},
        {"Beyond vertical", -2.0f, -0.5f, -0.25f, 110.0f, -245.0f, 145.0f, 2.0f}
    };
    int cases = 0;
    for (double heading : {-720.0, -90.0, 0.0, 37.0, 90.0, 179.0, 270.0, 450.0}) {
        for (double pitch : {-180.0, -110.0, -90.0, -89.999, -30.0, 0.0, 45.0, 89.999, 90.0, 100.0, 180.0}) {
            for (double roll : {-180.0, -90.0, -33.0, 0.0, 62.0, 90.0, 180.0}) {
                for (const auto& view : mounts) {
                    const auto pose = CameraPose::calculate(origin, heading, pitch, roll, view);
                    const Eigen::Vector3d bodyOffset(view.rightOffsetM, view.upOffsetM, -view.forwardOffsetM);
                    expectVectorNear(positionOf(pose), origin + referenceRotate(bodyOffset, heading, pitch, roll), 5e-6);

                    // Reconstruct what XPLM will render, including screen roll, from
                    // the returned Euler angles. At vertical look directions the
                    // numeric heading/roll pair is not unique, but the axes are.
                    for (const Eigen::Vector3d& axis : {Eigen::Vector3d(1.0, 0.0, 0.0),
                                                       Eigen::Vector3d(0.0, 1.0, 0.0),
                                                       Eigen::Vector3d(0.0, 0.0, -1.0)}) {
                        const auto mountAxis = referenceRotate(axis, view.headingOffsetDeg,
                                                               view.pitchOffsetDeg, view.rollOffsetDeg);
                        const auto expectedAxis = referenceRotate(mountAxis, heading, pitch, roll);
                        expectVectorNear(referenceRotate(axis, pose.heading, pose.pitch, pose.roll), expectedAxis);
                    }
                    expectNear(pose.zoom, view.zoom);
                    assert(pose.heading >= 0.0f && pose.heading < 360.0f);
                    assert(pose.pitch >= -90.0f && pose.pitch <= 90.0f);
                    assert(pose.roll >= -180.0f && pose.roll <= 180.0f);
                    ++cases;
                }
            }
        }
    }
    return cases;
}

} // namespace

int main()
{
    testLevelAndCardinalTranslations();
    testTiltedTranslations();
    testMinus90Plus90Mount();
    testRollComposition();
    testBodyUpNoseMount();
    testTranslationIndependentOfLook();
    const int cases = testAgainstSdkEquations();
    std::printf("camera_pose_test: known attitudes and %d SDK-equation cases passed\n", cases);
    return 0;
}
