
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

    const osg::Vec3 direction = normalize(movement.velocity);

    osg::Vec3 normal;
    if (SceneManager::RayCollision(*object->asNode(), direction, &normal)) {
      target->setMatrix(copie);

      movement.velocity =
        reflect(direction, normal) * 100.0f; // TODO: Dependent on the velocity
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
  }

} // Soleil
