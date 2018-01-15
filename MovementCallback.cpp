
#include "MovementCallback.h"

#include "Logger.h"
#include "Time.h"

#include <cassert>
#include <osg/MatrixTransform>

namespace Soleil {

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
    const osg::Vec3 acceleration = movement.force / movement.mass;
    movement.velocity += acceleration;

    //const float deltaTime = Time::DeltaTime();
    const float deltaTime = 0.016f;

    const osg::Vec3 powFriction(std::pow(movement.friction.x(), deltaTime),
                                std::pow(movement.friction.y(), deltaTime),
                                std::pow(movement.friction.z(), deltaTime));
    movement.velocity = osg::componentMultiply(movement.velocity, powFriction);

    const osg::Vec3 previousTemp = movement.point;
    movement.point += movement.velocity * deltaTime;

    if (movement.point.z() < 0.0f) {
      movement.point.z()    = 0.0f;
      movement.velocity.z() = 0.0f;
    }
  }

} // Soleil
