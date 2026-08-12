#include "HILSensorFlowController.h"

#include <cassert>
#include <cstdint>

int main()
{
    constexpr uint64_t timeoutUsec = 500000;

    HILSensorFlowController async(HILSensorFlowController::Mode::Async, timeoutUsec);
    assert(async.canSendSensor(0));
    async.markSensorQueued(0, 1000000, 1, 0);
    assert(async.canSendSensor(1));
    assert(async.getDiagnostics().sensors_queued == 1);

    HILSensorFlowController feedback(
        HILSensorFlowController::Mode::ActuatorFeedback, timeoutUsec);

    // One sensor is allowed at bootstrap. Its response timer starts only when
    // the complete MAVLink packet has reached the socket.
    assert(feedback.canSendSensor(1000));
    feedback.markSensorQueued(0, 1000000, 42, 1000);
    assert(feedback.getPendingCompletionToken() == 42);
    assert(!feedback.canSendSensor(1000 + timeoutUsec));
    assert(feedback.getFault() == HILSensorFlowController::Fault::None);
    feedback.observeTransmission(true, 2000);
    assert(!feedback.canSendSensor(2001));

    // Stale generations and timestamps cannot acknowledge the sensor.
    feedback.observeActuator(0, 1000000, 1);
    assert(!feedback.canSendSensor(3000));
    feedback.observeActuator(1, 999999, 1);
    assert(!feedback.canSendSensor(3500));

    // Forced feedback mode requires PX4's lockstep flag.
    feedback.observeActuator(1, 1000000, 0);
    assert(feedback.getFault() == HILSensorFlowController::Fault::MissingLockstepFlag);
    assert(feedback.getDiagnostics().protocol_faults == 1);

    feedback.reset(0);
    feedback.markSensorQueued(0, 1000000, 50, 0);
    feedback.observeTransmission(true, 1000);
    feedback.observeActuator(1, 1000000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(2000));

    feedback.markSensorQueued(1, 1010000, 60, 2000);
    feedback.observeTransmission(true, 3000);
    feedback.observeActuator(2, 1010000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(4000));

    // A missing response faults instead of adding another outstanding sample.
    feedback.markSensorQueued(2, 1020000, 70, 4000);
    feedback.observeTransmission(true, 5000);
    assert(!feedback.canSendSensor(5000 + timeoutUsec));
    assert(feedback.getFault() == HILSensorFlowController::Fault::ResponseTimeout);

    const auto diagnostics = feedback.getDiagnostics();
    assert(diagnostics.sensors_queued == 3);
    assert(diagnostics.sensors_transmitted == 3);
    assert(diagnostics.actuator_acks == 2);
    assert(diagnostics.response_timeouts == 1);
    assert(diagnostics.max_outstanding_sensors == 1);
    assert(diagnostics.sensor_outstanding);
    assert(diagnostics.feedback_established);

    feedback.reset(UINT64_MAX);
    feedback.markSensorQueued(UINT64_MAX, 1, 80, 0);
    feedback.observeTransmission(true, 1);
    feedback.observeActuator(0, 1,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(2));

    return 0;
}
