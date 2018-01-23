
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
    // osg::Matrix m     = target->getMatrix();
    // osg::Matrix copie = m;
    // m.setTrans(movement.point);
    // target->setMatrix(m);

    assert(data->asNodeVisitor());

    // SOLEIL__LOGGER_DEBUG(">>", movement.velocity);
    const osg::Vec3 direction = normalize(movement.velocity);

    osg::Vec3 normal;
    distanceToObject = 0.0f;

    // TODO: Cache the bounding box calculation
    osg::ComputeBoundsVisitor v;
    object->asNode()->accept(v);
    // osg::BoundingBox  box = v.getBoundingBox();
    // osg::BoundingBox  boxWS;
    // const osg::Matrix worldSpace =
    //   osg::computeLocalToWorld(data->asNodeVisitor()->getNodePath());
    // for (int i = 0; i < 7; ++i) {
    //   boxWS.expandBy(box.corner(i) * worldSpace);
    // }

    // if (SceneManager::RayCollision(box, direction, &normal,
    //                                &distanceToObject)) {

    osg::Vec3 Direction =
      osg::Vec3(movement.velocity.x(), movement.velocity.y(), 0.0f) * 0.016f;
    Direction.normalize();

    osg::Vec3 end =
      movement.point + Direction * object->asNode()->getBound().radius() * 1.5f;
    // previousPoint +
    // (osg::Vec3(movement.velocity.x(), movement.velocity.y(), 0.0f) * 0.016f *
    //  object->asNode()->getBound().radius() *
    //  10.0f); // object->asNode()->getBound().radius()
    if (SceneManager::SegmentCollision(previousPoint, end, &normal,
                                       &distanceToObject)) {
      // target->setMatrix(copie);

      movement.velocity =
        reflect(direction, normal) * 100.0f; // TODO: Dependent on the velocity

      // if (movement.velocity

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
    // const osg::Vec3 previousTemp = movement.point;
    movement.point += movement.velocity * 0.016f;

    if (movement.point.z() < 0.0f) {
      movement.point.z()    = 0.0f;
      movement.velocity.z() = 0.0f;
    }
  }

} // Soleil
