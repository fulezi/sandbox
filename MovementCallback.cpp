
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

// TODO: Temp:
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/RadialShooter>
#include <osgParticle/SectorPlacer>
static osg::ref_ptr<osgParticle::Emitter>
CreateExplosionEmitter(const osg::Vec3& position)
{
  constexpr float scale = 1.0f;

  osg::ref_ptr<osgParticle::RandomRateCounter> rrc =
    new osgParticle::RandomRateCounter;
  rrc->setRateRange(800, 1000);

  osg::ref_ptr<osgParticle::ModularEmitter> emitter =
    new osgParticle::ModularEmitter;
  // emitter->setParticleSystem(ps);
  emitter->setCounter(rrc);
  emitter->setEndless(false);
  emitter->setLifeTime(.10f);

  osg::ref_ptr<osgParticle::RadialShooter> shooter =
    new osgParticle::RadialShooter;
  osg::ref_ptr<osgParticle::SectorPlacer> placer =
    new osgParticle::SectorPlacer;

  emitter->setPlacer(placer);
  emitter->setShooter(shooter);

  placer->setCenter(position);
  placer->setRadiusRange(0.0f * scale, 0.25f * scale);

  shooter->setThetaRange(0.0f, osg::PI * 2.0f);
  shooter->setInitialSpeedRange(1.0f * scale, 10.0f * scale);

  return emitter;
}



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

// TODO: Temp:
      SceneManager::AddParticleEmitter(0, CreateExplosionEmitter(end));

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
