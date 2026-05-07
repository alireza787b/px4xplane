#pragma once

#include <utility>

namespace ActuatorSafety {

float clampFinite(float value, float minValue, float maxValue, float fallback = 0.0f);
bool isFiniteRange(float minValue, float maxValue);
float scaleActuatorCommand(float input, float inputMin, float inputMax, float outputMin, float outputMax);

} // namespace ActuatorSafety
