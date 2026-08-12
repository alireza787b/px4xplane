#include "HILSensorFlowController.h"

HILSensorFlowController::HILSensorFlowController(Mode mode, uint64_t responseTimeoutUsec)
{
    configure(mode, responseTimeoutUsec);
}

void HILSensorFlowController::configure(Mode mode, uint64_t responseTimeoutUsec)
{
    _mode = mode;
    _responseTimeoutUsec = responseTimeoutUsec;
    reset();
}

void HILSensorFlowController::reset(uint64_t actuatorGeneration)
{
    _generationAtSend = actuatorGeneration;
    _sensorTimestampUsec = 0;
    _completionToken = 0;
    _sensorSendTimeUsec = 0;
    _transmissionPending = false;
    _sensorOutstanding = false;
    _outstandingSensorCount = 0;
    _feedbackEstablished = false;
    _fault = Fault::None;
    _diagnostics = {};
}

void HILSensorFlowController::observeTransmission(bool complete, uint64_t nowUsec)
{
    if (_mode != Mode::ActuatorFeedback || !_transmissionPending || !complete) {
        return;
    }

    _transmissionPending = false;
    _sensorSendTimeUsec = nowUsec;
    ++_diagnostics.sensors_transmitted;
    _diagnostics.transmission_pending = false;
}

void HILSensorFlowController::observeActuator(uint64_t actuatorGeneration,
                                              uint64_t actuatorTimestampUsec,
                                              uint64_t actuatorFlags)
{
    if (_mode != Mode::ActuatorFeedback || !_sensorOutstanding || _transmissionPending ||
        actuatorGeneration == _generationAtSend ||
        actuatorTimestampUsec < _sensorTimestampUsec) {
        return;
    }

    if ((actuatorFlags & LOCKSTEP_ACTUATOR_FLAG) == 0) {
        _fault = Fault::MissingLockstepFlag;
        ++_diagnostics.protocol_faults;
        return;
    }

    _sensorOutstanding = false;
    _outstandingSensorCount = 0;
    _feedbackEstablished = true;
    ++_diagnostics.actuator_acks;
    _diagnostics.sensor_outstanding = false;
    _diagnostics.feedback_established = true;
}

bool HILSensorFlowController::canSendSensor(uint64_t nowUsec)
{
    if (_mode == Mode::Async) {
        return true;
    }

    if (_fault != Fault::None) {
        return false;
    }

    if (!_sensorOutstanding) {
        return true;
    }

    if (!_transmissionPending && _responseTimeoutUsec > 0 &&
        nowUsec >= _sensorSendTimeUsec &&
        nowUsec - _sensorSendTimeUsec >= _responseTimeoutUsec) {
        _fault = Fault::ResponseTimeout;
        ++_diagnostics.response_timeouts;
    }

    ++_diagnostics.blocked_samples;
    return false;
}

void HILSensorFlowController::markSensorQueued(uint64_t actuatorGeneration,
                                               uint64_t sensorTimestampUsec,
                                               uint64_t completionToken,
                                               uint64_t nowUsec)
{
    ++_diagnostics.sensors_queued;
    if (_mode == Mode::Async) {
        return;
    }

    _generationAtSend = actuatorGeneration;
    _sensorTimestampUsec = sensorTimestampUsec;
    _completionToken = completionToken;
    _sensorSendTimeUsec = nowUsec;
    _transmissionPending = true;
    _outstandingSensorCount = 1;
    _sensorOutstanding = true;
    if (_outstandingSensorCount > _diagnostics.max_outstanding_sensors) {
        _diagnostics.max_outstanding_sensors = _outstandingSensorCount;
    }
    _diagnostics.transmission_pending = true;
    _diagnostics.sensor_outstanding = _sensorOutstanding;
}

uint64_t HILSensorFlowController::getPendingCompletionToken() const
{
    return _completionToken;
}

HILSensorFlowController::Fault HILSensorFlowController::getFault() const
{
    return _fault;
}

HILSensorFlowController::Diagnostics HILSensorFlowController::getDiagnostics() const
{
    Diagnostics diagnostics = _diagnostics;
    diagnostics.transmission_pending = _transmissionPending;
    diagnostics.sensor_outstanding = _sensorOutstanding;
    diagnostics.feedback_established = _feedbackEstablished;
    return diagnostics;
}
