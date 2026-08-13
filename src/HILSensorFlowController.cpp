#include "HILSensorFlowController.h"

HILSensorFlowController::HILSensorFlowController(Mode mode,
                                                 uint64_t responseTimeoutUsec,
                                                 uint64_t bootstrapTimeoutUsec)
{
    configure(mode, responseTimeoutUsec, bootstrapTimeoutUsec);
}

void HILSensorFlowController::configure(Mode mode, uint64_t responseTimeoutUsec,
                                        uint64_t bootstrapTimeoutUsec)
{
    _mode = mode;
    _responseTimeoutUsec = responseTimeoutUsec;
    _bootstrapTimeoutUsec = bootstrapTimeoutUsec;
    reset();
}

void HILSensorFlowController::reset(uint64_t actuatorGeneration)
{
    _generationAtSend = actuatorGeneration;
    _lastObservedActuatorGeneration = actuatorGeneration;
    _queuedActuatorGeneration = actuatorGeneration;
    _sensorTimestampUsec = 0;
    _queuedSensorTimestampUsec = 0;
    _completionToken = 0;
    _transmissionQueueTimeUsec = 0;
    _sensorSendTimeUsec = 0;
    _bootstrapStartTimeUsec = 0;
    _bootstrapStarted = false;
    _bootstrapSensorCompleted = false;
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
    if (_feedbackEstablished || !_bootstrapSensorCompleted) {
        _generationAtSend = _queuedActuatorGeneration;
        _sensorTimestampUsec = _queuedSensorTimestampUsec;
        _bootstrapSensorCompleted = true;
    }
    _sensorSendTimeUsec = nowUsec;
    ++_diagnostics.sensors_transmitted;
    _diagnostics.transmission_pending = false;
}

void HILSensorFlowController::observeActuator(uint64_t actuatorGeneration,
                                              uint64_t actuatorTimestampUsec,
                                              uint64_t actuatorFlags)
{
    if (_mode != Mode::ActuatorFeedback ||
        actuatorGeneration == _lastObservedActuatorGeneration) {
        return;
    }

    _lastObservedActuatorGeneration = actuatorGeneration;

    if (_transmissionPending || actuatorGeneration == _generationAtSend ||
        actuatorTimestampUsec < _sensorTimestampUsec) {
        return;
    }

    // PX4 needs a short sensor bootstrap before it can publish its first
    // HIL_ACTUATOR_CONTROLS response. That first valid response establishes
    // the feedback handshake; subsequent samples use one-response-per-sensor
    // flow control.
    if (!_feedbackEstablished) {
        if (!_bootstrapSensorCompleted) {
            return;
        }
    } else if (!_sensorOutstanding) {
        return;
    }

    if ((actuatorFlags & LOCKSTEP_ACTUATOR_FLAG) == 0) {
        _fault = Fault::MissingLockstepFlag;
        ++_diagnostics.protocol_faults;
        return;
    }

    if (!_feedbackEstablished) {
        _generationAtSend = actuatorGeneration;
        _feedbackEstablished = true;
        ++_diagnostics.actuator_acks;
        _diagnostics.feedback_established = true;
        return;
    }

    _sensorOutstanding = false;
    _outstandingSensorCount = 0;
    _generationAtSend = actuatorGeneration;
    ++_diagnostics.actuator_acks;
    _diagnostics.sensor_outstanding = false;
}

bool HILSensorFlowController::canSendSensor(uint64_t nowUsec)
{
    if (_mode == Mode::Async) {
        return true;
    }

    if (_fault != Fault::None) {
        return false;
    }

    if (_transmissionPending) {
        if (_responseTimeoutUsec > 0 && nowUsec >= _transmissionQueueTimeUsec &&
            nowUsec - _transmissionQueueTimeUsec >= _responseTimeoutUsec) {
            _fault = Fault::TransmissionTimeout;
            ++_diagnostics.transmission_timeouts;
        }

        ++_diagnostics.blocked_samples;
        return false;
    }

    if (!_feedbackEstablished) {
        if (_bootstrapStarted && _bootstrapTimeoutUsec > 0 &&
            nowUsec >= _bootstrapStartTimeUsec &&
            nowUsec - _bootstrapStartTimeUsec >= _bootstrapTimeoutUsec) {
            _fault = Fault::BootstrapTimeout;
            ++_diagnostics.bootstrap_timeouts;
            ++_diagnostics.blocked_samples;
            return false;
        }

        return true;
    }

    if (!_sensorOutstanding) {
        return true;
    }

    if (_responseTimeoutUsec > 0 &&
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

    if (!_feedbackEstablished && !_bootstrapStarted) {
        _bootstrapStarted = true;
        _bootstrapStartTimeUsec = nowUsec;
    }

    _queuedActuatorGeneration = actuatorGeneration;
    _queuedSensorTimestampUsec = sensorTimestampUsec;
    _completionToken = completionToken;
    _transmissionQueueTimeUsec = nowUsec;
    _transmissionPending = true;
    _diagnostics.transmission_pending = true;

    if (!_feedbackEstablished) {
        ++_diagnostics.bootstrap_sensors;
        return;
    }

    _outstandingSensorCount = 1;
    _sensorOutstanding = true;
    if (_outstandingSensorCount > _diagnostics.max_outstanding_sensors) {
        _diagnostics.max_outstanding_sensors = _outstandingSensorCount;
    }
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
