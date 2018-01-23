
#include "MovementCallback.h"

#include "Logger.h"
#include "SceneManager.h"
#include "Time.h"
#include "Utils.h"

#include <cassert>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>

#include <chrono>
#include <iostream>

namespace Soleil {

  bool MovementCallback::run(osg::Object* object, osg::Object* data)
  {
    assert(object->asNode());
    assert(object->asNode()->asTransform());
    assert(object->asNode()->asTransform()->asMatrixTransform());

    // TODO: Cache casts?
    osg::MatrixTransform* target =
      object->asNode()->asTransform()->asMatrixTransform();

    const osg::Vec3 previousPoint = movement.point;
    update();

    assert(data->asNodeVisitor());

    const osg::Vec3 direction = normalize(movement.velocity);

    osg::Vec3 normal;
    distanceToObject = 0.0f;
// TODO: Cache the bounding box calculation
#if 0
    osg::ComputeBoundsVisitor v;
    object->asNode()->accept(v);
#endif

    osg::Vec3 Direction =
      osg::Vec3(movement.velocity.x(), movement.velocity.y(), 0.0f) * 0.016f;
    Direction.normalize();

    osg::Vec3 end =
      movement.point + Direction * object->asNode()->getBound().radius() * 1.5f;

    if (SceneManager::SegmentCollision(previousPoint, end, &normal,
                                       &distanceToObject)) {
      movement.velocity =
        reflect(direction, normal) * 100.0f; // TODO: Dependent on the velocity

      SOLEIL__LOGGER_DEBUG("REFLECTING: ", movement.velocity);
      movement.point = previousPoint;
    } else {
      osg::Matrix m = target->getMatrix();
      m.setTrans(movement.point);
      target->setMatrix(m);
    }

    return traverse(object, data);
  }

  void MovementCallback::update()
  {
    const float deltaTime = Time::DeltaTime();

    const osg::Vec3 acceleration = movement.force / movement.mass;

    movement.velocity += acceleration;

    movement.velocity.x() *=
      std::pow(movement.friction * movement.mass, deltaTime);
    movement.velocity.y() *=
      std::pow(movement.friction * movement.mass, deltaTime);
    movement.point += movement.velocity * 0.016f;

    if (movement.point.z() < 0.0f) {
      movement.point.z()    = 0.0f;
      movement.velocity.z() = 0.0f;
    }
  }

} // Soleil
