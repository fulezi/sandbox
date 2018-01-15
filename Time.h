
#ifndef SOLEIL__TIME_H_
#define SOLEIL__TIME_H_

namespace Soleil {

  class Time
  {
  public:
    Time();
    void startFrame(const float time) noexcept;

  public:
    static void StartFrame(const float time) noexcept;
    static float DeltaTime(void) noexcept;

  protected:
    float previousTime;
    float deltaTime;
  };

} // Soleil

#endif /* SOLEIL__TIME_H_ */
