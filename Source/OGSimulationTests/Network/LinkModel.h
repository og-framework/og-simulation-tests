// SPDX-License-Identifier: MPL-2.0
// docs/LinkModel-rationale.md

#pragma once

#if defined(JPH_VERSION_MAJOR)
#define OG_LINK_MODEL_JOLT_BEFORE_INCLUDES 1
#else
#define OG_LINK_MODEL_JOLT_BEFORE_INCLUDES 0
#endif

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(JPH_VERSION_MAJOR)
#define OG_LINK_MODEL_JOLT_AFTER_INCLUDES 1
#else
#define OG_LINK_MODEL_JOLT_AFTER_INCLUDES 0
#endif

static_assert(OG_LINK_MODEL_JOLT_AFTER_INCLUDES == OG_LINK_MODEL_JOLT_BEFORE_INCLUDES,
    "LinkModel.h includes only the STL and OGSimulation/ headers: one of its includes brought a Jolt header in "
    "(JPH_VERSION_MAJOR became defined). Was the og-input-latency task 3 acceptance criterion that the helper names "
    "no UE or Jolt symbol.");

#undef OG_LINK_MODEL_JOLT_BEFORE_INCLUDES
#undef OG_LINK_MODEL_JOLT_AFTER_INCLUDES

namespace linkModel
{
    enum class JitterShape : std::uint8_t
    {
        Uniform,
        TwoPoint,
    };

    struct Config
    {
        std::uint64_t seed                    = 1u;
        double        lossProbability         = 0.0;
        double        baseDelayTicks          = 0.0;
        JitterShape   jitterShape             = JitterShape::Uniform;
        double        jitterSpreadTicks       = 0.0;
        double        twoPointHighProbability = 0.5;
        double        reorderProbability      = 0.0;
        std::uint32_t reorderWindowPackets    = 0u;
    };

    inline constexpr std::uint64_t kLossStreamSalt    = 0x6C6F73734C696E6Bull;
    inline constexpr std::uint64_t kJitterStreamSalt  = 0x6A69747465724C6Bull;
    inline constexpr std::uint64_t kReorderStreamSalt = 0x72656F726465724Cull;

    class SplitMix64
    {
    public:
        constexpr explicit SplitMix64(std::uint64_t state)
            : m_state(state)
        {
        }

        constexpr std::uint64_t next()
        {
            m_state += 0x9E3779B97F4A7C15ull;
            std::uint64_t z = m_state;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            return z ^ (z >> 31);
        }

        constexpr double unit()
        {
            return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
        }

    private:
        std::uint64_t m_state;
    };

    static_assert(SplitMix64(0u).next() == 0xE220A8397B1DCDAFull,
        "SplitMix64 must produce the reference splitmix64 sequence, or every seeded pin in LinkModelPinsTodayTest.cpp "
        "moves (og-input-latency task 3).");

    class LinkModel
    {
    public:
        using Payload = std::vector<std::uint8_t>;
        using Tick    = double;

        explicit LinkModel(const Config& config)
            : m_config(config)
            , m_lossStream(config.seed ^ kLossStreamSalt)
            , m_jitterStream(config.seed ^ kJitterStreamSalt)
            , m_reorderStream(config.seed ^ kReorderStreamSalt)
        {
        }

        void send(Payload payload, Tick nowTick)
        {
            const std::uint64_t sendIndex = m_sentCount++;

            const bool   lost      = m_lossStream.unit() < m_config.lossProbability;
            const double jitter    = drawJitterTicks();
            const bool   reordered = m_reorderStream.unit() < m_config.reorderProbability
                                  && m_config.reorderWindowPackets > 0u;

            if (lost)
            {
                m_lostSends.push_back(sendIndex);
                return;
            }

            if (reordered)
            {
                m_reorderedSends.push_back(sendIndex);
            }

            m_inFlight.push_back(InFlight{ sendIndex, nowTick + m_config.baseDelayTicks + jitter, reordered, 0u,
                                           std::move(payload) });
        }

        template <typename Callback>
        std::size_t deliverDue(Tick nowTick, Callback&& callback)
        {
            std::size_t delivered = 0u;
            for (;;)
            {
                const std::size_t pick = pickDeliverable(nowTick);
                if (pick == m_inFlight.size())
                {
                    return delivered;
                }

                InFlight packet = std::move(m_inFlight[pick]);
                m_inFlight.erase(m_inFlight.begin() + static_cast<std::ptrdiff_t>(pick));

                for (InFlight& other : m_inFlight)
                {
                    if (other.held && other.sendIndex < packet.sendIndex)
                    {
                        ++other.laterDeliveries;
                    }
                }

                ++m_deliveredCount;
                ++delivered;
                callback(static_cast<const Payload&>(packet.payload));
            }
        }

        std::uint64_t sentCount() const { return m_sentCount; }
        std::uint64_t deliveredCount() const { return m_deliveredCount; }
        std::size_t   inFlightCount() const { return m_inFlight.size(); }

        const std::vector<std::uint64_t>& lostSends() const { return m_lostSends; }
        const std::vector<std::uint64_t>& reorderedSends() const { return m_reorderedSends; }

    private:
        struct InFlight
        {
            std::uint64_t sendIndex       = 0u;
            double        dueTick         = 0.0;
            bool          held            = false;
            std::uint32_t laterDeliveries = 0u;
            Payload       payload;
        };

        double drawJitterTicks()
        {
            const double u = m_jitterStream.unit();
            if (m_config.jitterShape == JitterShape::TwoPoint)
            {
                return (u < m_config.twoPointHighProbability) ? m_config.jitterSpreadTicks : 0.0;
            }
            return u * m_config.jitterSpreadTicks;
        }

        std::size_t pickDeliverable(Tick nowTick) const
        {
            std::size_t best = m_inFlight.size();
            for (std::size_t i = 0; i < m_inFlight.size(); ++i)
            {
                const InFlight& candidate = m_inFlight[i];
                if (candidate.dueTick > nowTick)
                {
                    continue;
                }
                if (candidate.held && candidate.laterDeliveries < m_config.reorderWindowPackets)
                {
                    continue;
                }
                if (best == m_inFlight.size()
                    || candidate.dueTick < m_inFlight[best].dueTick
                    || (candidate.dueTick == m_inFlight[best].dueTick
                        && candidate.sendIndex < m_inFlight[best].sendIndex))
                {
                    best = i;
                }
            }
            return best;
        }

        Config                     m_config;
        SplitMix64                 m_lossStream;
        SplitMix64                 m_jitterStream;
        SplitMix64                 m_reorderStream;
        std::uint64_t              m_sentCount      = 0u;
        std::uint64_t              m_deliveredCount = 0u;
        std::vector<InFlight>      m_inFlight;
        std::vector<std::uint64_t> m_lostSends;
        std::vector<std::uint64_t> m_reorderedSends;
    };

    static_assert(std::is_same_v<LinkModel::Payload, std::vector<std::uint8_t>>
                      && std::is_same_v<LinkModel::Tick, double>
                      && std::is_same_v<decltype(&LinkModel::send), void (LinkModel::*)(LinkModel::Payload, LinkModel::Tick)>,
        "LinkModel carries STL byte vectors in the tick domain and names no engine type in its surface. Was the "
        "og-input-latency task 3 acceptance criterion that the helper names no UE or Jolt symbol.");
} // namespace linkModel
