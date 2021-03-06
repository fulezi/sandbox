
#include <limits>
#include <memory>

#include "SceneManager.h"

#include "Logger.h"

#include <functional>
#include <osg/ComputeBoundsVisitor>
#include <osg/Drawable>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/RayIntersector>

static std::unique_ptr<SceneManager> sceneManager;

void
SceneManager::Init(osg::ref_ptr<osg::Group> sceneRoot)
{
  sceneManager            = std::make_unique<SceneManager>();
  sceneManager->sceneRoot = sceneRoot;
}

bool
SceneManager::SegmentCollision(const osg::Vec3& start, const osg::Vec3& end,
                               osg::Vec3* normal, float* distanceToObject)
{
  assert(sceneManager && "Call SceneManager::Init method");

  osg::ref_ptr<osgUtil::LineSegmentIntersector> lineSegment =
    new osgUtil::LineSegmentIntersector(
      osgUtil::Intersector::CoordinateFrame::MODEL, start, end, nullptr,
      osgUtil::Intersector::IntersectionLimit::LIMIT_NEAREST); // NO_LIMIT
                                                               // LIMIT_NEAREST
  osgUtil::IntersectionVisitor visitor(lineSegment);

  sceneManager->sceneRoot->accept(visitor);

  // TODO: Do polytope fetch. Collision is taken from the center of mass wich
  // allows wings to avoid colliding

  if (lineSegment->containsIntersections()) {
    float nearestDistance = std::numeric_limits<float>::max();
    SOLEIL__LOGGER_DEBUG("----");
    for (const auto& intersection : lineSegment->getIntersections()) {
      const float distance =
        (start - intersection.getWorldIntersectPoint()).length();

      if (nearestDistance > distance) {
        nearestDistance = distance;

        if (normal) {
          *normal = intersection.getWorldIntersectNormal();
        }
      }

      SOLEIL__LOGGER_DEBUG("COLLISION between collider and ",
                           intersection.nodePath.back()->getName(),
                           ". Normal: ", intersection.getWorldIntersectNormal(),
                           ". Distance=", distance);
    }

    if (distanceToObject) {
      *distanceToObject = nearestDistance;
    }
    if (normal) {
      normal->normalize();
    }

    return true;
  }
  return false;
}

void
SceneManager::RegisterParticleSystem(
  ObjectID id, osg::ref_ptr<osgParticle::ParticleSystem> system)
{
  assert(sceneManager && "Call SceneManager::Init method");

  sceneManager->particleSystems.emplace(id, system);
  sceneManager->sceneRoot->addChild(system);
  // TODO: Only one ParticleSystemUpdater
  // TODO: A specific group for particle systems
}

void
SceneManager::AddParticleEmitter(ObjectID particleSystemID,
                                 osg::ref_ptr<osgParticle::Emitter> emitter)
{
  assert(sceneManager && "Call SceneManager::Init method");

  osg::ref_ptr<osgParticle::ParticleSystem> ps =
    sceneManager->particleSystems[particleSystemID];
  assert(ps && "No particle system found with this name");

  emitter->setParticleSystem(ps);
  sceneManager->sceneRoot->addChild(emitter);

  // TODO: A specific group for emitter
  // TODO: Remove all emitters that are finished (have a
  // SceneManager::frameUpdate method)

}
