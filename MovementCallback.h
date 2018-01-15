
#ifndef SOLEIL__MOVEMENTCALLBACK_H_
#define SOLEIL__MOVEMENTCALLBACK_H_

#include <osg/Callback>
#include <osg/Vec3>

namespace Soleil {

  struct Movement
  {
    osg::Vec3 force;
    osg::Vec3 velocity;
    float friction;
    float     mass;

    osg::Vec3 point;

    Movement()
      : force(0.0f, 0.0f, -9.31f)
      , velocity(0.0f, 0.0f, 0.0f)
      , friction(0.002f)
      , mass(1.0f)
      , point(0.0f, 0.0f, 0.0f)
    {
    }
  };

  class MovementCallback : public osg::Callback
  {
  public:
    bool run(osg::Object* object, osg::Object* data) override;

  protected:
    void update();

  public:
    Movement movement;
  };

} // Soleil

#endif /* SOLEIL__MOVEMENTCALLBACK_H_ */
