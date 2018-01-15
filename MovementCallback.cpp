
#include "MovementCallback.h"

#include "Logger.h"
#include "Time.h"

#include <cassert>
#include <osg/MatrixTransform>

#include <chrono>
#include <iostream>

namespace Soleil {

  class Timer
  {
  public:
    Timer()
      : beg_(clock_::now())
    {
    }
    void   reset() { beg_ = clock_::now(); }
    double elapsed()
    {
      auto a =
        std::chrono::duration_cast<second_>(clock_::now() - beg_).count();
      reset();
      return a;
    }

  private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1>> second_;
    std::chrono::time_point<clock_> beg_;
  };

  bool MovementCallback::run(osg::Object* object, osg::Object* data)
  {
    assert(object->asNode());
    assert(object->asNode()->asTransform());
    assert(object->asNode()->asTransform()->asMatrixTransform());

    // TODO: Cache casts?
    osg::MatrixTransform* target =
      object->asNode()->asTransform()->asMatrixTransform();

    update();
    auto m = target->getMatrix();
    m.setTrans(movement.point);
    target->setMatrix(m);

    return traverse(object, data);
  }

  void MovementCallback::update()
  {
    const float deltaTime = Time::DeltaTime();

    const osg::Vec3 acceleration = movement.force / movement.mass;

    movement.velocity += acceleration;

    movement.velocity.x() = movement.velocity.x() *
                            std::pow(movement.friction * movement.mass, deltaTime);
    movement.velocity.y() = movement.velocity.y() *
                            std::pow(movement.friction * movement.mass, deltaTime);
    // const osg::Vec3 previousTemp = movement.point;
    movement.point += movement.velocity * deltaTime;

    if (movement.point.z() < 0.0f) {
      movement.point.z()    = 0.0f;
      movement.velocity.z() = 0.0f;
    }

    static Timer t;

    SOLEIL__LOGGER_DEBUG(movement.point);
  }

} // Soleil
