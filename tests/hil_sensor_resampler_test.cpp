#include "HILSensorResampler.h"
#include "HILSensorFlowController.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

bool near(float actual, float expected, float tolerance = 0.0001f)
{
    return std::fabs(actual - expected) <= tolerance;
}

HILSensorSample sample(float value)
{
    HILSensorSample result;
    result.xacc = value;
    result.yacc = value + 1.0f;
    result.zacc = value + 2.0f;
    result.xgyro = value + 3.0f;
    result.temperature = value + 4.0f;
    return result;
}

} // namespace

int main()
{
    HILSensorResampler resampler;

    assert(resampler.appendFrame(1000000, sample(0.0f)));
    assert(resampler.hasPending());
    assert(resampler.front().timestamp_usec == 1000000);
    assert(near(resampler.front().sample.xacc, 0.0f));
    assert(resampler.front().frame_endpoint);
    resampler.popFront();

    // Reproduce the B2 50.254 ms frame. Four 12.563/12.564 ms samples
    // preserve the complete interval without crossing the 15 ms bound.
    assert(resampler.appendFrame(1050254, sample(4.0f)));
    std::vector<ScheduledHILSensorSample> split;
    while (resampler.hasPending()) {
        split.push_back(resampler.front());
        resampler.popFront();
    }
    assert(split.size() == 4);
    assert(split.back().timestamp_usec == 1050254);
    assert(!split[0].frame_endpoint);
    assert(!split[1].frame_endpoint);
    assert(!split[2].frame_endpoint);
    assert(split[3].frame_endpoint);
    assert(near(split[0].sample.xacc, 1.0f));
    assert(near(split[1].sample.xacc, 2.0f));
    assert(near(split[2].sample.xacc, 3.0f));
    assert(near(split[3].sample.xacc, 4.0f));

    uint64_t previous = 1000000;
    for (const auto& scheduled : split) {
        assert(scheduled.timestamp_usec > previous);
        assert(scheduled.timestamp_usec - previous <=
               HILSensorResampler::DEFAULT_MAX_INTERVAL_USEC);
        previous = scheduled.timestamp_usec;
    }

    auto diagnostics = resampler.getDiagnostics();
    assert(diagnostics.frames_captured == 2);
    assert(diagnostics.split_frames == 1);
    assert(diagnostics.samples_generated == 5);
    assert(diagnostics.samples_consumed == 5);
    assert(diagnostics.max_generated_interval_usec <= 15000);
    assert(diagnostics.max_pending_samples == 4);
    assert(diagnostics.fault == HILSensorResampler::Fault::None);

    // Timestamp regressions fail explicitly rather than creating an invalid
    // integration interval.
    assert(!resampler.appendFrame(1050254, sample(5.0f)));
    assert(resampler.getFault() == HILSensorResampler::Fault::InvalidTimestamp);

    // A transport that cannot drain samples gets a bounded backlog fault.
    HILSensorResampler bounded(15000, 3);
    assert(bounded.appendFrame(1000000, sample(0.0f)));
    assert(!bounded.appendFrame(1050000, sample(1.0f)));
    assert(bounded.getFault() == HILSensorResampler::Fault::BacklogOverflow);
    assert(bounded.getDiagnostics().pending_samples == 1);

    bounded.reset();
    assert(bounded.getFault() == HILSensorResampler::Fault::None);
    assert(bounded.getDiagnostics().pending_samples == 0);

    // Coupled B2 regression: after feedback is established, fifteen
    // consecutive 50.254 ms frames produce bounded samples and retain the
    // one-response-per-sensor invariant.
    HILSensorResampler pipeline;
    HILSensorFlowController feedback(
        HILSensorFlowController::Mode::ActuatorFeedback, 500000, 10000000);
    uint64_t timestamp = 1000000;
    uint64_t generation = 0;
    uint64_t token = 1;

    for (int bootstrap = 0; bootstrap < 20; ++bootstrap) {
        assert(pipeline.appendFrame(timestamp, sample(0.0f)));
        assert(pipeline.hasPending());
        const auto scheduled = pipeline.front();
        assert(feedback.canSendSensor(timestamp));
        feedback.markSensorQueued(generation, scheduled.timestamp_usec,
                                  token++, timestamp);
        feedback.observeTransmission(true, timestamp + 100);
        pipeline.popFront();
        timestamp += 10000;
    }

    feedback.observeActuator(++generation, timestamp - 10000,
                             HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
    assert(feedback.getDiagnostics().feedback_established);

    timestamp -= 10000;
    uint64_t previousScheduledTimestamp = timestamp;
    for (int frame = 0; frame < 15; ++frame) {
        timestamp += 50254;
        assert(pipeline.appendFrame(timestamp, sample(static_cast<float>(frame + 1))));

        while (pipeline.hasPending()) {
            const auto scheduled = pipeline.front();
            assert(scheduled.timestamp_usec > previousScheduledTimestamp);
            assert(scheduled.timestamp_usec - previousScheduledTimestamp <=
                   HILSensorResampler::DEFAULT_MAX_INTERVAL_USEC);
            assert(feedback.canSendSensor(scheduled.timestamp_usec));
            feedback.markSensorQueued(generation, scheduled.timestamp_usec,
                                      token++, scheduled.timestamp_usec);
            feedback.observeTransmission(true, scheduled.timestamp_usec + 100);
            feedback.observeActuator(
                ++generation, scheduled.timestamp_usec,
                HILSensorFlowController::LOCKSTEP_ACTUATOR_FLAG);
            assert(feedback.canSendSensor(scheduled.timestamp_usec + 200));
            previousScheduledTimestamp = scheduled.timestamp_usec;
            pipeline.popFront();
        }
    }

    const auto pipelineDiagnostics = pipeline.getDiagnostics();
    const auto feedbackDiagnostics = feedback.getDiagnostics();
    assert(pipelineDiagnostics.split_frames == 15);
    assert(pipelineDiagnostics.max_generated_interval_usec <= 15000);
    assert(pipelineDiagnostics.pending_samples == 0);
    assert(feedbackDiagnostics.max_outstanding_sensors == 1);
    assert(feedbackDiagnostics.response_timeouts == 0);
    assert(feedbackDiagnostics.protocol_faults == 0);

    return 0;
}
