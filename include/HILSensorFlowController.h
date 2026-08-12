#pragma once

#include <cstdint>

class HILSensorFlowController {
public:
    static constexpr uint64_t LOCKSTEP_ACTUATOR_FLAG = 1ULL;

    enum class Mode {
        Async,
        ActuatorFeedback
    };

    enum class Fault {
        None,
        ResponseTimeout,
        MissingLockstepFlag
    };

    struct Diagnostics {
        uint64_t sensors_queued{0};
        uint64_t sensors_transmitted{0};
        uint64_t actuator_acks{0};
        uint64_t blocked_samples{0};
        uint64_t response_timeouts{0};
        uint64_t protocol_faults{0};
        uint64_t max_outstanding_sensors{0};
        bool transmission_pending{false};
        bool sensor_outstanding{false};
        bool feedback_established{false};
    };

    explicit HILSensorFlowController(Mode mode = Mode::Async,
                                     uint64_t responseTimeoutUsec = 500000);

    void configure(Mode mode, uint64_t responseTimeoutUsec);
    void reset(uint64_t actuatorGeneration = 0);
    void observeTransmission(bool complete, uint64_t nowUsec);
    void observeActuator(uint64_t actuatorGeneration, uint64_t actuatorTimestampUsec,
                         uint64_t actuatorFlags);
    bool canSendSensor(uint64_t nowUsec);
    void markSensorQueued(uint64_t actuatorGeneration, uint64_t sensorTimestampUsec,
                          uint64_t completionToken, uint64_t nowUsec);
    uint64_t getPendingCompletionToken() const;
    Fault getFault() const;
    Diagnostics getDiagnostics() const;

private:
    Mode _mode{Mode::Async};
    uint64_t _responseTimeoutUsec{500000};
    uint64_t _generationAtSend{0};
    uint64_t _sensorTimestampUsec{0};
    uint64_t _completionToken{0};
    uint64_t _sensorSendTimeUsec{0};
    bool _transmissionPending{false};
    bool _sensorOutstanding{false};
    uint64_t _outstandingSensorCount{0};
    bool _feedbackEstablished{false};
    Fault _fault{Fault::None};
    Diagnostics _diagnostics{};
};
