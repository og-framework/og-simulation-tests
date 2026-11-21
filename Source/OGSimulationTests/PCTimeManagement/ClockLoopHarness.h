#pragma once
// SPDX-License-Identifier: MPL-2.0
// docs/ClockLoopHarness-rationale.md

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "OGSimulation/Mailbox.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ClockDilationController.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationScheduler.h"
#include "OGSimulation/TickDriverConcepts.h"

namespace clockLoopHarness
{
inline constexpr double   kTickHz                    = 60.0;
inline constexpr double   kStepSeconds               = 1.0 / kTickHz;
inline constexpr uint32_t kMaxCatchUpSteps           = 60u;
inline constexpr double   kMinRateScale              = 0.75;
inline constexpr double   kMaxRateScale              = 1.25;
inline constexpr double   kMaxUndilatedFrameSeconds  = 0.4;
inline constexpr double   kPingIntervalSeconds       = 1.0 / 20.0;
inline constexpr double   kPingsBeforeJoinSeconds    = 0.3;
inline constexpr double   kTurnaroundSeconds         = 50e-6;
inline constexpr double   kRenderDelaySteps          = 2.0;
inline constexpr size_t   kMaxClients                = 2u;

class SplitMix64
{
public:
    explicit SplitMix64(uint64_t state)
        : m_state(state)
    {
    }

    uint64_t next()
    {
        m_state += 0x9E3779B97F4A7C15ull;
        uint64_t z = m_state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

private:
    uint64_t m_state;
};

struct Link
{
    double baseSeconds       = 0.028;
    double jitterMeanSeconds = 0.0075;
    double lossProbability   = 0.0;
    double burstProbability  = 0.0;
    double burstExtraSeconds = 0.200;
};

struct Hitch
{
    double atSeconds = -1.0;
    double seconds   = 0.0;
};

struct ClientSpec
{
    uint64_t            seed               = 1u;
    double              joinSeconds        = 2.0;
    double              clockOffsetSeconds = 1000.0;
    double              clockRatePpm       = 0.0;
    double              framesPerSecond    = 100.0;
    double              frameJitter        = 0.0;
    bool                drainAtStepStart   = true;
    Link                up;
    Link                down;
    ClockDilationConfig dilation;
    Hitch               hitch;
    bool                render             = false;
    std::vector<std::pair<double, double>> forcedScales;
};

struct WorldSpec
{
    double   serverStepSeconds  = 0.0;
    uint32_t serverTickAtZero   = 6000u;
    double   sampleEverySeconds = 0.010;
};

struct FrameRecord
{
    double               seconds        = 0.0;
    double               clockSeconds   = 0.0;
    double               trueLeadTicks  = 0.0;
    double               sensorErrorTicks = 0.0;
    bool                 sensorValid    = false;
    bool                 dilated        = false;
    ClockDilationOutput  out;
    ClockCorrectionPhase phase;
    double               rateScale      = 1.0;
    uint32_t             steps          = 0u;
    double               readCentreTicks = std::numeric_limits<double>::quiet_NaN();
};

struct StepRecord
{
    double                               seconds            = 0.0;
    double                               executionLeadTicks = 0.0;
    ClientPredictionClock::AdvanceResult result             = ClientPredictionClock::AdvanceResult::Normal;
};

struct CorrectionEvent
{
    double                               seconds      = 0.0;
    ClientPredictionClock::AdvanceResult result       = ClientPredictionClock::AdvanceResult::Normal;
    bool                                 recentreDebt = false;
};

struct RenderStats
{
    uint32_t frames          = 0u;
    uint32_t checkedFrames   = 0u;
    uint32_t beyondOldest    = 0u;
    uint32_t renderTimeBacks = 0u;
    double   minAdvanceSeconds = std::numeric_limits<double>::infinity();
};

struct WindowLine
{
    double      seconds = 0.0;
    std::string text;
};

struct PairSample
{
    double                           seconds = 0.0;
    std::array<double, kMaxClients>  leadTicks{};
    std::array<bool, kMaxClients>    started{};
};

struct Server
{
    WorldSpec spec;

    double publishSecondsOf(uint32_t tick) const
    {
        return (static_cast<double>(tick) - static_cast<double>(spec.serverTickAtZero)) / kTickHz + spec.serverStepSeconds;
    }

    uint32_t publishedTickAt(double seconds) const
    {
        const double steps = std::floor((seconds - spec.serverStepSeconds) * kTickHz);
        return static_cast<uint32_t>(static_cast<double>(spec.serverTickAtZero) + steps);
    }

    double trueTickAt(double seconds) const { return static_cast<double>(spec.serverTickAtZero) + seconds * kTickHz; }
};

class LoopClient
{
public:
    struct Manager
    {
        const NetworkTimeEstimator*  estimator = nullptr;
        const ClientPredictionClock* clock     = nullptr;
        const TimeConfig*            time      = nullptr;

        bool                         runsPrediction() const { return true; }
        const NetworkTimeEstimator&  getNetworkEstimator() const { return *estimator; }
        const ClientPredictionClock& getClientClock() const { return *clock; }
        const TimeConfig&            getTimeConfig() const { return *time; }
    };

    LoopClient(const ClientSpec& spec, const Server& server)
        : m_spec(spec)
        , m_server(server)
        , m_rng(spec.seed)
        , m_estimator(m_time, nullptr)
        , m_clock(m_time, m_estimator, nullptr)
        , m_controller(spec.dilation)
        , m_window(spec.dilation)
    {
        m_manager.estimator = &m_estimator;
        m_manager.clock     = &m_clock;
        m_manager.time      = &m_time;
        m_nextFrameClock    = clockAt(spec.joinSeconds);
        m_nextAuthorityTick = m_server.publishedTickAt(spec.joinSeconds) + 1u;
        m_pingOrigin        = clockAt(spec.joinSeconds - kPingsBeforeJoinSeconds);
        for (double ping = m_pingOrigin; ping < m_nextFrameClock; ping += kPingIntervalSeconds)
            sendPing(ping);
        m_nextPingClock = m_pingOrigin
                          + kPingIntervalSeconds * (std::floor((m_nextFrameClock - m_pingOrigin) / kPingIntervalSeconds) + 1.0);
    }

    LoopClient(const LoopClient&) = delete;
    LoopClient& operator=(const LoopClient&) = delete;

    double nextFrameSeconds() const { return trueAt(m_nextFrameClock); }

    void frame()
    {
        const double clockNow = m_nextFrameClock;
        const double now      = trueAt(clockNow);
        generateAuthority(now);
        if (!m_spec.drainAtStepStart)
            drain(now);

        if (!m_scheduler.has_value())
        {
            m_host = clockNow;
            m_scheduler.emplace(SchedulerConfig{ kStepSeconds, MaxCatchUpSteps{ kMaxCatchUpSteps }, kMinRateScale, kMaxRateScale },
                                m_host);
        }
        else
        {
            const double frameSeconds = clockNow - m_lastFrameClock;
            m_host += std::min(frameSeconds, kMaxUndilatedFrameSeconds);
            m_lostHostSeconds += std::max(0.0, frameSeconds - kMaxUndilatedFrameSeconds);
        }
        m_lastFrameClock = clockNow;
        if (clockNow >= m_nextPingClock)
        {
            sendPing(clockNow);
            m_nextPingClock = m_pingOrigin
                              + kPingIntervalSeconds * (std::floor((clockNow - m_pingOrigin) / kPingIntervalSeconds) + 1.0);
        }

        SimulationScheduler& scheduler = *m_scheduler;
        TickRateDoor door(scheduler, m_tickOffset, false, m_refusedRateChanges);

        FrameRecord record;
        record.seconds          = now;
        record.clockSeconds     = clockNow;
        record.trueLeadTicks    = deadlineTickAt(door, m_host) - m_server.trueTickAt(now);
        record.sensorErrorTicks = m_estimator.serverTickEstimator().serverTickAt(clockNow) - m_server.trueTickAt(now);
        record.sensorValid      = m_estimator.serverTickEstimator().isValid(clockNow);

        bool forced = false;
        while (m_nextForced < m_spec.forcedScales.size() && m_spec.forcedScales[m_nextForced].first <= now)
        {
            door.setRateScale(m_spec.forcedScales[m_nextForced].second);
            ++m_nextForced;
            forced = true;
        }
        if (!forced && m_nextForced == 0u && m_spec.dilation.mode != ClockDilationMode::Off)
        {
            record.out     = clockDilation::beforeClientPump(door, m_host, clockNow, m_manager, m_controller, m_window);
            record.dilated = true;
            if (m_window.isDue(clockNow))
            {
                m_windowLines.push_back(WindowLine{ now, m_window.formatLine() });
                m_window.restart();
            }
        }
        record.rateScale = door.rateScale();

        const uint32_t steps = scheduler.pump(m_host);
        record.steps         = steps;
        ClockTargetRead firstRead;
        for (uint32_t index = 0u; index < steps; ++index)
        {
            step(scheduler, scheduler.physicsStepCount() - steps + index, now);
            if (index == 0u)
                firstRead = m_clock.publishedTargetRead();
        }
        if (steps > 0u)
            record.readCentreTicks = static_cast<double>(firstRead.targetTick)
                                     - m_estimator.serverTickEstimator().serverTickAt(clockNow);

        if (m_spec.render)
            render(scheduler);

        record.phase = m_clock.publishedCorrectionPhase();
        notePhase(record.phase, now);
        m_frames.push_back(record);

        const bool hitchFrame = !m_hitchDone && m_spec.hitch.atSeconds >= 0.0 && now >= m_spec.hitch.atSeconds;
        m_hitchDone           = m_hitchDone || hitchFrame;
        m_nextFrameClock      = clockNow + (hitchFrame ? m_spec.hitch.seconds : framePeriodSeconds());
    }

    bool started() const { return m_scheduler.has_value(); }

    double trueLeadAt(double seconds) const
    {
        const SimulationScheduler& scheduler = *m_scheduler;
        const double hostNow = m_host + (clockAt(seconds) - m_lastFrameClock);
        const double tick = static_cast<double>(m_tickOffset + static_cast<int64_t>(scheduler.physicsStepCount()))
                            + (hostNow - scheduler.nextDeadline()) / scheduler.currentStepInterval();
        return tick - m_server.trueTickAt(seconds);
    }

    const ClientSpec&                   spec() const { return m_spec; }
    const std::vector<FrameRecord>&     frames() const { return m_frames; }
    const std::vector<StepRecord>&      steps() const { return m_steps; }
    const std::vector<CorrectionEvent>& corrections() const { return m_corrections; }
    const std::vector<WindowLine>&      windowLines() const { return m_windowLines; }
    double                              lostHostSeconds() const { return m_lostHostSeconds; }
    const RenderStats&                  renderStats() const { return m_render; }
    const NetworkTimeEstimator&         estimator() const { return m_estimator; }
    const ClientPredictionClock&        clock() const { return m_clock; }
    const SimulationScheduler&          scheduler() const { return *m_scheduler; }
    uint64_t                            refusedRateChanges() const { return m_refusedRateChanges; }
    std::optional<double>               firstHardResyncSeconds() const { return m_firstHardResync; }
    std::optional<double>               joinSettledSeconds() const { return m_joinSettled; }

    uint32_t countAfter(double seconds, ClientPredictionClock::AdvanceResult result) const
    {
        return static_cast<uint32_t>(std::count_if(m_corrections.begin(), m_corrections.end(),
            [&](const CorrectionEvent& event) { return event.seconds >= seconds && event.result == result; }));
    }

private:
    struct Message
    {
        double            arrivalSeconds = 0.0;
        bool              pong           = false;
        uint32_t          authorityTick  = 0u;
        SimClockRttSample sample;
    };

    double clockAt(double seconds) const
    {
        return m_spec.clockOffsetSeconds + (1.0 + m_spec.clockRatePpm * 1e-6) * seconds;
    }

    double trueAt(double clockSeconds) const
    {
        return (clockSeconds - m_spec.clockOffsetSeconds) / (1.0 + m_spec.clockRatePpm * 1e-6);
    }

    double framePeriodSeconds()
    {
        const double periodSeconds = 1.0 / m_spec.framesPerSecond;
        if (m_spec.frameJitter <= 0.0)
            return periodSeconds;
        return periodSeconds * (1.0 + m_spec.frameJitter * (2.0 * m_rng.unit() - 1.0));
    }

    static double deadlineTickAt(const TickRateDoor& door, double hostNow)
    {
        return static_cast<double>(door.tickOffset() + static_cast<int64_t>(door.physicsStepCount()))
               + (hostNow - door.nextDeadline()) / door.currentInterval();
    }

    std::optional<double> draw(const Link& link)
    {
        const double lossDraw  = m_rng.unit();
        const double burstDraw = m_rng.unit();
        const double jitter    = -link.jitterMeanSeconds * std::log(1.0 - m_rng.unit());
        if (lossDraw < link.lossProbability)
            return std::nullopt;
        return link.baseSeconds + jitter + (burstDraw < link.burstProbability ? link.burstExtraSeconds : 0.0);
    }

    void generateAuthority(double now)
    {
        while (m_server.publishSecondsOf(m_nextAuthorityTick) <= now)
        {
            const double sent = m_server.publishSecondsOf(m_nextAuthorityTick);
            if (const std::optional<double> delay = draw(m_spec.down))
            {
                m_lastAuthorityArrival = std::max(m_lastAuthorityArrival, sent + *delay);
                Message message;
                message.arrivalSeconds = m_lastAuthorityArrival;
                message.authorityTick  = m_nextAuthorityTick;
                m_authority.push_back(message);
            }
            ++m_nextAuthorityTick;
        }
    }

    void sendPing(double clockSeconds)
    {
        const std::optional<double> up   = draw(m_spec.up);
        const std::optional<double> down = draw(m_spec.down);
        if (!up.has_value() || !down.has_value())
            return;
        const double sent      = trueAt(clockSeconds);
        m_lastPingReceive      = std::max(m_lastPingReceive, sent + *up);
        const double received  = m_lastPingReceive;
        const double answered  = received + kTurnaroundSeconds;
        const uint32_t tick    = m_server.publishedTickAt(answered);
        const double age       = answered - m_server.publishSecondsOf(tick);
        m_lastPongArrival      = std::max(m_lastPongArrival, answered + *down);

        Message message;
        message.arrivalSeconds              = m_lastPongArrival;
        message.pong                        = true;
        message.sample.t0Seconds            = clockSeconds;
        message.sample.t1Seconds            = received;
        message.sample.t2Seconds            = answered;
        message.sample.t3Seconds            = clockAt(m_lastPongArrival);
        message.sample.rttSeconds           = (message.sample.t3Seconds - message.sample.t0Seconds) - (answered - received);
        message.sample.thetaSeconds         = ((received - clockSeconds) + (answered - message.sample.t3Seconds)) / 2.0;
        message.sample.serverTick           = tick;
        message.sample.serverTickAgeSeconds = std::round(age * 1e6) * 1e-6;
        m_pongs.push_back(message);
    }

    void drain(double now)
    {
        for (;;)
        {
            const bool authorityDue = !m_authority.empty() && m_authority.front().arrivalSeconds <= now;
            const bool pongDue      = !m_pongs.empty() && m_pongs.front().arrivalSeconds <= now;
            if (!authorityDue && !pongDue)
                return;
            const bool takePong = pongDue && (!authorityDue || m_pongs.front().arrivalSeconds <= m_authority.front().arrivalSeconds);
            if (takePong)
            {
                m_estimator.noteSimClockSample(m_pongs.front().sample);
                m_pongs.pop_front();
            }
            else
            {
                m_estimator.recordAuthorityTick(m_authority.front().authorityTick);
                m_authority.pop_front();
            }
        }
    }

    void step(SimulationScheduler& scheduler, uint64_t physicsStep, double now)
    {
        if (m_spec.drainAtStepStart)
            drain(now);
        const bool recentreDebt = m_clock.isRecentring();
        const ClientPredictionClock::AdvanceResult result = m_clock.advancePrediction();
        m_tickOffset = static_cast<int64_t>(m_clock.getPredictionTick()) - static_cast<int64_t>(physicsStep);
        m_steps.push_back(
            StepRecord{ now, static_cast<double>(m_clock.getPredictionTick()) - m_server.trueTickAt(now), result });
        if (result != ClientPredictionClock::AdvanceResult::Normal)
            m_corrections.push_back(CorrectionEvent{ now, result, recentreDebt });
        if (result == ClientPredictionClock::AdvanceResult::HardResync && !m_firstHardResync.has_value())
            m_firstHardResync = now;

        if (!m_spec.render)
            return;
        Snapshot* const slot = m_snapshots.beginWrite();
        if (slot == nullptr)
            return;
        *slot                     = Snapshot{};
        slot->physicsStep         = physicsStep;
        slot->stepDeadlineSeconds = scheduler.stepDeadline(physicsStep);
        slot->tick                = static_cast<SimTick>(m_clock.getPredictionTick());
        m_snapshots.commit();
        ++m_published;
    }

    void render(const SimulationScheduler& scheduler)
    {
        RenderInterpolationParams params;
        params.renderTimeSeconds = m_host - kRenderDelaySteps * scheduler.config().dtSeconds / scheduler.rateScale();
        RenderInterpolationFrame frame;
        if (!interpolateRenderPoses(m_snapshots, params, m_poses, frame))
            return;
        ++m_render.frames;
        if (m_published >= 4u)
        {
            ++m_render.checkedFrames;
            m_render.beyondOldest += frame.beyondOldest ? 1u : 0u;
            if (m_lastRenderTime.has_value())
            {
                const double advance       = params.renderTimeSeconds - *m_lastRenderTime;
                m_render.renderTimeBacks  += advance < 0.0 ? 1u : 0u;
                m_render.minAdvanceSeconds = std::min(m_render.minAdvanceSeconds, advance);
            }
            m_lastRenderTime = params.renderTimeSeconds;
        }
    }

    void notePhase(const ClockCorrectionPhase& phase, double now)
    {
        if (!m_firstHardResync.has_value() || m_joinSettled.has_value())
            return;
        if (!phase.recentreArmed && !phase.recentring && !phase.stallDebt && !phase.awaitsFirstRtt)
            m_joinSettled = now;
    }

    using Snapshot = RenderSnapshotT<1>;

    ClientSpec                        m_spec;
    const Server&                     m_server;
    SplitMix64                        m_rng;
    TimeConfig                        m_time;
    NetworkTimeEstimator              m_estimator;
    ClientPredictionClock             m_clock;
    ClockDilationController           m_controller;
    ClockDilationWindow               m_window;
    Manager                           m_manager;
    std::optional<SimulationScheduler> m_scheduler;
    int64_t                           m_tickOffset         = 0;
    uint64_t                          m_refusedRateChanges = 0u;
    double                            m_host               = 0.0;
    double                            m_lastFrameClock     = 0.0;
    double                            m_nextFrameClock     = 0.0;
    double                            m_pingOrigin         = 0.0;
    double                            m_nextPingClock      = 0.0;
    uint32_t                          m_nextAuthorityTick  = 0u;
    double                            m_lastAuthorityArrival = 0.0;
    double                            m_lastPingReceive    = 0.0;
    double                            m_lastPongArrival    = 0.0;
    std::deque<Message>               m_authority;
    std::deque<Message>               m_pongs;
    size_t                            m_nextForced         = 0u;
    bool                              m_hitchDone          = false;
    std::vector<FrameRecord>          m_frames;
    std::vector<StepRecord>           m_steps;
    std::vector<CorrectionEvent>      m_corrections;
    std::vector<WindowLine>           m_windowLines;
    double                            m_lostHostSeconds    = 0.0;
    std::optional<double>             m_firstHardResync;
    std::optional<double>             m_joinSettled;
    SnapshotChannel<Snapshot, 4>      m_snapshots;
    Snapshot                          m_poses;
    uint64_t                          m_published          = 0u;
    std::optional<double>             m_lastRenderTime;
    RenderStats                       m_render;
};

class World
{
public:
    World(const WorldSpec& spec, const std::vector<ClientSpec>& clients)
    {
        m_server.spec = spec;
        for (const ClientSpec& client : clients)
            m_clients.push_back(std::make_unique<LoopClient>(client, m_server));
        m_nextSample = 0.0;
    }

    void runUntil(double seconds)
    {
        for (;;)
        {
            LoopClient* next = nullptr;
            for (const std::unique_ptr<LoopClient>& client : m_clients)
            {
                if (next == nullptr || client->nextFrameSeconds() < next->nextFrameSeconds())
                    next = client.get();
            }
            const double frameAt = next != nullptr ? next->nextFrameSeconds() : seconds;
            if (m_nextSample <= frameAt && m_nextSample <= seconds)
            {
                sample(m_nextSample);
                m_nextSample += m_server.spec.sampleEverySeconds;
                continue;
            }
            if (next == nullptr || frameAt > seconds)
                return;
            next->frame();
        }
    }

    const LoopClient&              client(size_t index) const { return *m_clients[index]; }
    const std::vector<PairSample>& samples() const { return m_samples; }
    const Server&                  server() const { return m_server; }

private:
    void sample(double seconds)
    {
        PairSample row;
        row.seconds = seconds;
        for (size_t index = 0u; index < m_clients.size() && index < kMaxClients; ++index)
        {
            row.started[index] = m_clients[index]->started();
            if (row.started[index])
                row.leadTicks[index] = m_clients[index]->trueLeadAt(seconds);
        }
        m_samples.push_back(row);
    }

    Server                                   m_server;
    std::vector<std::unique_ptr<LoopClient>> m_clients;
    double                                   m_nextSample = 0.0;
    std::vector<PairSample>                  m_samples;
};

inline double percentileOf(std::vector<double> values, double percent)
{
    if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const size_t rank = static_cast<size_t>(std::ceil(percent / 100.0 * static_cast<double>(values.size())));
    return values[std::clamp<size_t>(rank, 1u, values.size()) - 1u];
}

inline double meanOf(const std::vector<double>& values)
{
    double sum = 0.0;
    for (double value : values)
        sum += value;
    return values.empty() ? std::numeric_limits<double>::quiet_NaN() : sum / static_cast<double>(values.size());
}
} // namespace clockLoopHarness
