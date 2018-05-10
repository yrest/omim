#include "map/extrapolation/extrapolator.hpp"

#include "geometry/distance_on_sphere.hpp"

#include "platform/platform.hpp"

#include "base/logging.hpp"

#include <chrono>
#include <memory>

namespace
{
uint64_t constexpr kMaxExtrapolationTimeMs = 1000;
uint64_t constexpr kExtrapolationPeriodMs = 200;
double constexpr kMaxExtrapolationSpeedMPS = 120.0;

double Delta(double param1, double param2, uint64_t timeBetweenPointsMs, uint64_t timeAfterParam2Ms)
{
  return (param2 - param1) * timeAfterParam2Ms / timeBetweenPointsMs;
}

class LinearExtrapolator
{
public:
  LinearExtrapolator(uint64_t timeBetweenMs, uint64_t timeAfterMs)
    : m_timeBetweenMs(timeBetweenMs), m_timeAfterMs(timeAfterMs)
  {
  }

  double Extrapolate(double x1, double x2)
  {
    return x2 + (x2 - x1) / m_timeBetweenMs * m_timeAfterMs;
  }

private:
  uint64_t m_timeBetweenMs;
  uint64_t m_timeAfterMs;
};
}  // namespace

namespace extrapolation
{
using namespace std;

location::GpsInfo LinearExtrapolation(location::GpsInfo const & gpsInfo1,
                                      location::GpsInfo const & gpsInfo2,
                                      uint64_t timeAfterPoint2Ms)
{
  auto const timeBetweenPointsMs =
      static_cast<uint64_t>((gpsInfo2.m_timestamp - gpsInfo1.m_timestamp) * 1000);

  location::GpsInfo extrapolated = gpsInfo2;
  LinearExtrapolator extrapolator(timeBetweenPointsMs, timeAfterPoint2Ms);

  extrapolated.m_timestamp += timeAfterPoint2Ms;

  extrapolated.m_longitude = extrapolator.Extrapolate(gpsInfo1.m_longitude, gpsInfo2.m_longitude);

  extrapolated.m_latitude = extrapolator.Extrapolate(gpsInfo1.m_latitude, gpsInfo2.m_latitude);

  extrapolated.m_horizontalAccuracy =
      extrapolator.Extrapolate(gpsInfo1.m_horizontalAccuracy, gpsInfo2.m_horizontalAccuracy);

  extrapolated.m_altitude = extrapolator.Extrapolate(gpsInfo1.m_altitude, gpsInfo2.m_altitude);

  if (gpsInfo1.HasVerticalAccuracy() && gpsInfo2.HasVerticalAccuracy())
  {
    extrapolated.m_verticalAccuracy =
        extrapolator.Extrapolate(gpsInfo1.m_verticalAccuracy, gpsInfo2.m_verticalAccuracy);
  }

  if (gpsInfo1.HasBearing() && gpsInfo2.HasBearing())
    extrapolated.m_bearing = extrapolator.Extrapolate(gpsInfo1.m_bearing, gpsInfo2.m_bearing);

  if (gpsInfo1.HasSpeed() && gpsInfo2.HasSpeed())
    extrapolated.m_speed = extrapolator.Extrapolate(gpsInfo1.m_speed, gpsInfo2.m_speed);

  return extrapolated;
}

// Extrapolator::Routine ---------------------------------------------------------------------------
Extrapolator::Routine::Routine(ExtrapolatedLocationUpdate const & update)
  : m_extrapolatedLocationUpdate(update)
{
}

void Extrapolator::Routine::Do()
{
  while (!IsCancelled())
  {
    {
      GetPlatform().RunTask(Platform::Thread::Gui, [this]() {
        lock_guard<mutex> guard(m_mutex);
        uint64_t const extrapolationTimeMs = kExtrapolationPeriodMs * m_extrapolationCounter;
        if (extrapolationTimeMs >= kMaxExtrapolationTimeMs)
          return;

        if (DoesExtrapolationWork(extrapolationTimeMs))
        {
          location::GpsInfo gpsInfo =
              LinearExtrapolation(m_beforeLastGpsInfo, m_lastGpsInfo, extrapolationTimeMs);
          m_extrapolatedLocationUpdate(gpsInfo);
        }
        else
        {
          if (m_lastGpsInfo.m_source != location::EUndefined)
          {
            location::GpsInfo gpsInfo = m_lastGpsInfo;
            m_extrapolatedLocationUpdate(gpsInfo);
          }
        }
      });

      lock_guard<mutex> guard(m_mutex);
      if (m_extrapolationCounter != m_extrapolationCounterUndefined)
        ++m_extrapolationCounter;
    }
    // @TODO(bykoinako) Method m_extrapolatedLocationUpdate() is run on gui thread every
    // |kExtrapolationPeriodMs| milliseconds. But after changing GPS position
    // (that means after a call of method Routine::SetGpsInfo())
    // m_extrapolatedLocationUpdate() should be run immediately on gui thread.
    this_thread::sleep_for(std::chrono::milliseconds(kExtrapolationPeriodMs));
  }
}

void Extrapolator::Routine::SetGpsInfo(location::GpsInfo const & gpsInfo)
{
  lock_guard<mutex> guard(m_mutex);
  m_beforeLastGpsInfo = m_lastGpsInfo;
  m_lastGpsInfo = gpsInfo;
  m_extrapolationCounter = 0;
}

bool Extrapolator::Routine::DoesExtrapolationWork(uint64_t extrapolationTimeMs) const
{
  // Note. It's possible that m_beforeLastGpsInfo.m_timestamp >= m_lastGpsInfo.m_timestamp.
  // It may happen in rare cases because GpsInfo::m_timestamp is not monotonic generally.
  // Please see comment in declaration of class GpsInfo for details.

  if (m_extrapolationCounter == m_extrapolationCounterUndefined ||
      m_lastGpsInfo.m_source == location::EUndefined ||
      m_beforeLastGpsInfo.m_source == location::EUndefined ||
      m_beforeLastGpsInfo.m_timestamp >= m_lastGpsInfo.m_timestamp)
  {
    return false;
  }

  double const distM =
      ms::DistanceOnEarth(m_beforeLastGpsInfo.m_latitude, m_beforeLastGpsInfo.m_longitude,
                          m_lastGpsInfo.m_latitude, m_lastGpsInfo.m_longitude);
  double const timeS = m_lastGpsInfo.m_timestamp - m_beforeLastGpsInfo.m_timestamp;

  // Switching off extrapolation based on speed.
  return distM / timeS < kMaxExtrapolationSpeedMPS;
  // @TODO(bykoianko) Switching off extrapolation based on acceleration should be implemented.
}

// Extrapolator ------------------------------------------------------------------------------------
Extrapolator::Extrapolator(ExtrapolatedLocationUpdate const & update)
  : m_extrapolatedLocationThread()
{
  m_extrapolatedLocationThread.Create(make_unique<Routine>(update));
}

void Extrapolator::OnLocationUpdate(location::GpsInfo & info)
{
  auto * routine = m_extrapolatedLocationThread.GetRoutineAs<Routine>();
  CHECK(routine, ());
  routine->SetGpsInfo(info);
}
}  // namespace extrapolation
