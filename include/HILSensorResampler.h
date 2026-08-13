#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

struct HILSensorSample {
    float xacc{0.0f};
    float yacc{0.0f};
    float zacc{0.0f};
    float xgyro{0.0f};
    float ygyro{0.0f};
    float zgyro{0.0f};
    float xmag{0.0f};
    float ymag{0.0f};
    float zmag{0.0f};
    float abs_pressure{0.0f};
    float diff_pressure{0.0f};
    float pressure_alt{0.0f};
    float temperature{0.0f};
};

struct ScheduledHILSensorSample {
    uint64_t timestamp_usec{0};
    HILSensorSample sample{};
    bool frame_endpoint{false};
};

class HILSensorResampler {
public:
    // PX4's default 10 ms EKF prediction period accepts delayed IMU samples up
    // to 20 ms. Keep a 25 percent margin so float timestamp quantization cannot
    // put an interpolated sample on that boundary.
    static constexpr uint64_t DEFAULT_MAX_INTERVAL_USEC = 15000;
    static constexpr size_t DEFAULT_MAX_PENDING_SAMPLES = 64;

    enum class Fault {
        None,
        InvalidTimestamp,
        BacklogOverflow
    };

    struct Diagnostics {
        uint64_t frames_captured{0};
        uint64_t split_frames{0};
        uint64_t samples_generated{0};
        uint64_t samples_consumed{0};
        uint64_t max_generated_interval_usec{0};
        size_t max_pending_samples{0};
        size_t pending_samples{0};
        Fault fault{Fault::None};
    };

    explicit HILSensorResampler(
        uint64_t maxIntervalUsec = DEFAULT_MAX_INTERVAL_USEC,
        size_t maxPendingSamples = DEFAULT_MAX_PENDING_SAMPLES);

    void configure(uint64_t maxIntervalUsec, size_t maxPendingSamples);
    void reset();

    bool appendFrame(uint64_t timestampUsec, const HILSensorSample& sample);
    bool hasPending() const;
    const ScheduledHILSensorSample& front() const;
    void popFront();
    Fault getFault() const;
    Diagnostics getDiagnostics() const;

private:
    static HILSensorSample interpolate(const HILSensorSample& from,
                                       const HILSensorSample& to,
                                       float fraction);
    void appendScheduled(uint64_t timestampUsec, const HILSensorSample& sample,
                         bool frameEndpoint);

    uint64_t _maxIntervalUsec{DEFAULT_MAX_INTERVAL_USEC};
    size_t _maxPendingSamples{DEFAULT_MAX_PENDING_SAMPLES};
    uint64_t _previousFrameTimestampUsec{0};
    uint64_t _lastGeneratedTimestampUsec{0};
    HILSensorSample _previousFrameSample{};
    bool _hasPreviousFrame{false};
    std::deque<ScheduledHILSensorSample> _pending;
    Fault _fault{Fault::None};
    Diagnostics _diagnostics{};
};
