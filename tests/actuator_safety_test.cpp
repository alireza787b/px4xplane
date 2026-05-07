#include "ActuatorSafety.h"

#include <cassert>
#include <cmath>

namespace {

void expectNear(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 1e-5f);
}

} // namespace

int main()
{
    expectNear(ActuatorSafety::scaleActuatorCommand(0.0f, -1.0f, 1.0f, -20.0f, 20.0f), 0.0f);
    expectNear(ActuatorSafety::scaleActuatorCommand(1.0f, -1.0f, 1.0f, -20.0f, 20.0f), 20.0f);
    expectNear(ActuatorSafety::scaleActuatorCommand(-1.0f, -1.0f, 1.0f, -20.0f, 20.0f), -20.0f);

    expectNear(ActuatorSafety::scaleActuatorCommand(2.0f, -1.0f, 1.0f, 0.0f, 1.0f), 1.0f);
    expectNear(ActuatorSafety::scaleActuatorCommand(-2.0f, -1.0f, 1.0f, 0.0f, 1.0f), 0.0f);
    expectNear(ActuatorSafety::scaleActuatorCommand(NAN, -1.0f, 1.0f, 0.0f, 1.0f), 0.5f);
    expectNear(ActuatorSafety::scaleActuatorCommand(INFINITY, -1.0f, 1.0f, -10.0f, 10.0f), 0.0f);

    expectNear(ActuatorSafety::clampFinite(2.0f, -1.0f, 1.0f, 0.0f), 1.0f);
    expectNear(ActuatorSafety::clampFinite(-2.0f, -1.0f, 1.0f, 0.0f), -1.0f);
    expectNear(ActuatorSafety::clampFinite(NAN, -1.0f, 1.0f, 0.25f), 0.25f);
    expectNear(ActuatorSafety::clampFinite(0.5f, 1.0f, -1.0f, 0.0f), 0.5f);

    assert(ActuatorSafety::isFiniteRange(-1.0f, 1.0f));
    assert(!ActuatorSafety::isFiniteRange(1.0f, 1.0f));
    assert(!ActuatorSafety::isFiniteRange(NAN, 1.0f));

    return 0;
}
