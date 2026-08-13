#include "HILSensorResampler.h"

#include <algorithm>
#include <cassert>

namespace {

float lerp(float from, float to, float fraction)
{
    return from + (to - from) * fraction;
}

} // namespace

HILSensorResampler::HILSensorResampler(uint64_t maxIntervalUsec,
                                       size_t maxPendingSamples)
{
    configure(maxIntervalUsec, maxPendingSamples);
}

void HILSensorResampler::configure(uint64_t maxIntervalUsec,
                                   size_t maxPendingSamples)
{
    _maxIntervalUsec = maxIntervalUsec;
    _maxPendingSamples = maxPendingSamples;
    reset();
}

void HILSensorResampler::reset()
{
    _previousFrameTimestampUsec = 0;
    _lastGeneratedTimestampUsec = 0;
    _previousFrameSample = {};
    _hasPreviousFrame = false;
    _pending.clear();
    _fault = Fault::None;
    _diagnostics = {};
}

bool HILSensorResampler::appendFrame(uint64_t timestampUsec,
                                     const HILSensorSample& sample)
{
    if (_fault != Fault::None) {
        return false;
    }

    if (timestampUsec == 0 || _maxIntervalUsec == 0 || _maxPendingSamples == 0 ||
        (_hasPreviousFrame && timestampUsec <= _previousFrameTimestampUsec)) {
        _fault = Fault::InvalidTimestamp;
        _diagnostics.fault = _fault;
        return false;
    }

    ++_diagnostics.frames_captured;

    if (!_hasPreviousFrame) {
        if (_pending.size() >= _maxPendingSamples) {
            _fault = Fault::BacklogOverflow;
            _diagnostics.fault = _fault;
            return false;
        }

        appendScheduled(timestampUsec, sample, true);
        _previousFrameTimestampUsec = timestampUsec;
        _previousFrameSample = sample;
        _hasPreviousFrame = true;
        return true;
    }

    const uint64_t frameDeltaUsec = timestampUsec - _previousFrameTimestampUsec;
    const uint64_t segmentCount =
        1 + (frameDeltaUsec - 1) / _maxIntervalUsec;

    if (segmentCount > _maxPendingSamples - _pending.size()) {
        _fault = Fault::BacklogOverflow;
        _diagnostics.fault = _fault;
        return false;
    }

    if (segmentCount > 1) {
        ++_diagnostics.split_frames;
    }

    for (uint64_t segment = 1; segment <= segmentCount; ++segment) {
        const float fraction = static_cast<float>(segment) /
                               static_cast<float>(segmentCount);
        const uint64_t scheduledTimestamp = _previousFrameTimestampUsec +
            (frameDeltaUsec * segment) / segmentCount;
        appendScheduled(scheduledTimestamp,
                        interpolate(_previousFrameSample, sample, fraction),
                        segment == segmentCount);
    }

    _previousFrameTimestampUsec = timestampUsec;
    _previousFrameSample = sample;
    return true;
}

bool HILSensorResampler::hasPending() const
{
    return !_pending.empty();
}

const ScheduledHILSensorSample& HILSensorResampler::front() const
{
    assert(!_pending.empty());
    return _pending.front();
}

void HILSensorResampler::popFront()
{
    assert(!_pending.empty());
    _pending.pop_front();
    ++_diagnostics.samples_consumed;
}

HILSensorResampler::Fault HILSensorResampler::getFault() const
{
    return _fault;
}

HILSensorResampler::Diagnostics HILSensorResampler::getDiagnostics() const
{
    Diagnostics diagnostics = _diagnostics;
    diagnostics.pending_samples = _pending.size();
    diagnostics.fault = _fault;
    return diagnostics;
}

HILSensorSample HILSensorResampler::interpolate(const HILSensorSample& from,
                                                const HILSensorSample& to,
                                                float fraction)
{
    HILSensorSample sample;
    sample.xacc = lerp(from.xacc, to.xacc, fraction);
    sample.yacc = lerp(from.yacc, to.yacc, fraction);
    sample.zacc = lerp(from.zacc, to.zacc, fraction);
    sample.xgyro = lerp(from.xgyro, to.xgyro, fraction);
    sample.ygyro = lerp(from.ygyro, to.ygyro, fraction);
    sample.zgyro = lerp(from.zgyro, to.zgyro, fraction);
    sample.xmag = lerp(from.xmag, to.xmag, fraction);
    sample.ymag = lerp(from.ymag, to.ymag, fraction);
    sample.zmag = lerp(from.zmag, to.zmag, fraction);
    sample.abs_pressure = lerp(from.abs_pressure, to.abs_pressure, fraction);
    sample.diff_pressure = lerp(from.diff_pressure, to.diff_pressure, fraction);
    sample.pressure_alt = lerp(from.pressure_alt, to.pressure_alt, fraction);
    sample.temperature = lerp(from.temperature, to.temperature, fraction);
    return sample;
}

void HILSensorResampler::appendScheduled(uint64_t timestampUsec,
                                         const HILSensorSample& sample,
                                         bool frameEndpoint)
{
    if (_lastGeneratedTimestampUsec > 0) {
        const uint64_t intervalUsec = timestampUsec - _lastGeneratedTimestampUsec;
        _diagnostics.max_generated_interval_usec =
            (std::max)(_diagnostics.max_generated_interval_usec, intervalUsec);
    }

    _pending.push_back({timestampUsec, sample, frameEndpoint});
    _lastGeneratedTimestampUsec = timestampUsec;
    ++_diagnostics.samples_generated;
    _diagnostics.max_pending_samples =
        (std::max)(_diagnostics.max_pending_samples, _pending.size());
}
