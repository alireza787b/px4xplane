#include "HILSensorFlowController.h"

#include <cassert>
#include <cstdint>

int main()
{
    constexpr uint64_t timeoutUsec = 500000;
    constexpr uint64_t bootstrapTimeoutUsec = 10000000;

    HILSensorFlowController async(
        HILSensorFlowController::Mode::Async, timeoutUsec, bootstrapTimeoutUsec);
    assert(async.canSendSensor(0));
    async.markSensorQueued(0, 1000000, 1, 0);
    assert(async.canSendSensor(1));
    assert(async.getDiagnostics().sensors_queued == 1);

    HILSensorFlowController feedback(
        HILSensorFlowController::Mode::ActuatorFeedback,
        timeoutUsec, bootstrapTimeoutUsec);

    // A socket write has its own deadline before actuator feedback can matter.
    assert(feedback.canSendSensor(1000));
    feedback.markSensorQueued(0, 1000000, 42, 1000);
    assert(feedback.getPendingCompletionToken() == 42);
    assert(!feedback.canSendSensor(1000 + timeoutUsec - 1));
    assert(feedback.getFault() == HILSensorFlowController::Fault::None);
    assert(!feedback.canSendSensor(1000 + timeoutUsec));
    assert(feedback.getFault() == HILSensorFlowController::Fault::TransmissionTimeout);
    assert(feedback.getDiagnostics().transmission_timeouts == 1);

    // Bootstrap may span many response-timeout intervals, but it cannot
    // silently remain asynchronous forever.
    feedback.reset(0);
    feedback.markSensorQueued(0, 1000000, 43, 1000);
    feedback.observeTransmission(true, 1100);
    assert(feedback.canSendSensor(1000 + bootstrapTimeoutUsec - 1));
    assert(!feedback.canSendSensor(1000 + bootstrapTimeoutUsec));
    assert(feedback.getFault() == HILSensorFlowController::Fault::BootstrapTimeout);
    assert(feedback.getDiagnostics().bootstrap_timeouts == 1);

    // An actuator event observed before the sensor write completes is not
    // reconsidered later as a bootstrap acknowledgement.
    feedback.reset(0);
    feedback.markSensorQueued(0, 1000000, 44, 1000);
    feedback.observeActuator(1, 1000000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    feedback.observeTransmission(true, 1100);
    feedback.observeActuator(1, 1000000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(!feedback.getDiagnostics().feedback_established);

    // Stale timestamps and generations cannot establish the handshake, and a
    // previously rejected event is not accepted if its cached fields change.
    feedback.observeActuator(2, 999999,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    feedback.observeActuator(2, 1000000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    feedback.observeActuator(0, 1000000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(!feedback.getDiagnostics().feedback_established);

    // A causally eligible response must carry PX4's lockstep flag.
    feedback.observeActuator(3, 1000000, 0);
    assert(feedback.getFault() == HILSensorFlowController::Fault::MissingLockstepFlag);
    assert(feedback.getDiagnostics().protocol_faults == 1);

    // A longer startup stream remains bounded by the bootstrap deadline and
    // still establishes from a response to the first completed sensor.
    feedback.reset(0);
    for (uint64_t sample = 0; sample < 20; ++sample) {
        const uint64_t sampleTime = sample * 10000;
        assert(feedback.canSendSensor(sampleTime));
        feedback.markSensorQueued(0, 1000000 + sampleTime,
                                  100 + sample, sampleTime);
        feedback.observeTransmission(true, sampleTime + 100);
    }
    feedback.observeActuator(1, 1050000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.getDiagnostics().feedback_established);
    assert(feedback.getDiagnostics().bootstrap_sensors == 20);
    assert(feedback.getDiagnostics().bootstrap_timeouts == 0);

    // Bootstrap retains its first completed sensor as the causal handshake
    // anchor while continuing to stream startup samples. A response may lag a
    // newer bootstrap sample without becoming ambiguous.
    feedback.reset(0);
    feedback.markSensorQueued(0, 1000000, 50, 0);
    feedback.observeTransmission(true, 1000);
    feedback.markSensorQueued(0, 1100000, 51, 1100);
    feedback.observeTransmission(true, 1200);
    feedback.observeActuator(1, 1050000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.getDiagnostics().feedback_established);
    assert(feedback.canSendSensor(2000));

    feedback.markSensorQueued(1, 1010000, 60, 2000);
    assert(!feedback.canSendSensor(2000 + timeoutUsec - 1));
    assert(feedback.getFault() == HILSensorFlowController::Fault::None);
    feedback.observeTransmission(true, 2000 + timeoutUsec);
    feedback.observeActuator(1, 1010000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(!feedback.canSendSensor(2000 + timeoutUsec + 100));
    feedback.observeActuator(2, 1009999,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(!feedback.canSendSensor(2000 + timeoutUsec + 200));
    feedback.observeActuator(2, 1010000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(!feedback.canSendSensor(2000 + timeoutUsec + 300));
    feedback.observeActuator(3, 1010000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(2000 + timeoutUsec + 400));

    // A missing response faults instead of adding another outstanding sample.
    feedback.markSensorQueued(3, 1020000, 70, 2000 + timeoutUsec + 500);
    feedback.observeTransmission(true, 2000 + timeoutUsec + 600);
    assert(!feedback.canSendSensor(2000 + 2 * timeoutUsec + 600));
    assert(feedback.getFault() == HILSensorFlowController::Fault::ResponseTimeout);

    const auto diagnostics = feedback.getDiagnostics();
    assert(diagnostics.sensors_queued == 4);
    assert(diagnostics.sensors_transmitted == 4);
    assert(diagnostics.actuator_acks == 2);
    assert(diagnostics.bootstrap_sensors == 2);
    assert(diagnostics.response_timeouts == 1);
    assert(diagnostics.max_outstanding_sensors == 1);
    assert(diagnostics.sensor_outstanding);
    assert(diagnostics.feedback_established);

    // Reconnect clears fault, handshake, and all diagnostic state.
    feedback.reset(3);
    assert(feedback.getFault() == HILSensorFlowController::Fault::None);
    assert(!feedback.getDiagnostics().feedback_established);
    assert(feedback.getDiagnostics().sensors_queued == 0);
    assert(feedback.canSendSensor(6000));

    // Generation wrap is valid in both bootstrap and strict feedback states.
    feedback.reset(UINT64_MAX);
    feedback.markSensorQueued(UINT64_MAX, 1, 80, 0);
    feedback.observeTransmission(true, 1);
    feedback.observeActuator(0, 1,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(2));
    feedback.markSensorQueued(0, 2, 81, 2);
    feedback.observeTransmission(true, 3);
    feedback.observeActuator(1, 2,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.canSendSensor(4));

    return 0;
}
