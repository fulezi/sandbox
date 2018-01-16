
#include "MovementCallback.h"

#include "Logger.h"
#include "SceneManager.h"
#include "Time.h"
#include "Utils.h"

#include <cassert>
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

    update();
    osg::Matrix m     = target->getMatrix();
    osg::Matrix copie = m;
    m.setTrans(movement.point);
    target->setMatrix(m);

    assert(data->asNodeVisitor());

    osg::Matrix worldSpace =
      osg::computeLocalToWorld(data->asNodeVisitor()->getNodePath());

    const osg::Vec3 direction = normalize(movement.velocity);

    osg::Vec3 normal;
    if (SceneManager::RayCollision(worldSpace, *object->asNode(), direction,
                                   &normal)) {
      target->setMatrix(copie);
      // movement.velocity.x() = -(movement.velocity.x()) * 3.0f;
      // movement.velocity.y() = -(movement.velocity.y()) * 3.0f;

      // movement.force.x() = 0.0f;
      // movement.force.y() = 0.0f;
      movement.velocity = reflect(direction, normal) * 100.0f;
    }

    return traverse(object, data);
  }

  void MovementCallback::update()
  {
    const float deltaTime = Time::DeltaTime();

    const osg::Vec3 acceleration = movement.force / movement.mass;

    movement.velocity += acceleration;

    movement.velocity.x() =
      movement.velocity.x() *
      std::pow(movement.friction * movement.mass, deltaTime);
    movement.velocity.y() =
      movement.velocity.y() *
      std::pow(movement.friction * movement.mass, deltaTime);
    // const osg::Vec3 previousTemp = movement.point;
    movement.point += movement.velocity * 0.016f;

    if (movement.point.z() < 0.0f) {
      movement.point.z()    = 0.0f;
      movement.velocity.z() = 0.0f;
    }

    SOLEIL__LOGGER_DEBUG(movement.point);
  }

} // Soleil
