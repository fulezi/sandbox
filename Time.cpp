

#include "Time.h"

namespace Soleil {

  static Time globalTime;

  Time::Time()
    : previousTime(-1)
    , deltaTime(0)
  {
  }

  void Time::startFrame(const float time) noexcept
  {
    deltaTime    = time - previousTime;
    previousTime = time;
  }

  void Time::StartFrame(const float time) noexcept { globalTime.startFrame(time); }

  float Time::DeltaTime(void) noexcept {
    return globalTime.deltaTime;
    //return 0.016f;
  }

} // Soleil
