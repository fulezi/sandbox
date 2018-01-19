
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
SceneManager::RayCollision(const osg::BoundingBox& box,
                           const osg::Vec3& direction, osg::Vec3* normal,
                           float* distanceToObject)
{
  osg::ref_ptr<osgUtil::RayIntersector> ray = new osgUtil::RayIntersector(
    osgUtil::Intersector::CoordinateFrame::MODEL, box.center(), direction,
    nullptr, osgUtil::Intersector::IntersectionLimit::NO_LIMIT); // LIMIT_NEAREST
  osgUtil::IntersectionVisitor visitor(ray);

  sceneManager->sceneRoot->accept(visitor);

  // TODO: Do polytope fetch. Collision is taken from the center of mass wich
  // allows wings to avoid colliding

  if (ray->containsIntersections()) {
    // TODO: Do polytope intersection

    if (*distanceToObject) {
      *distanceToObject = std::numeric_limits<float>::max();
    }
    for (const auto& intersection : ray->getIntersections()) {
      if (distanceToObject) {
        // TODO: Closest
        *distanceToObject = osg::minimum(
          *distanceToObject, static_cast<float>(intersection.distance));
      }

      if (box.contains(intersection.getWorldIntersectPoint())) {
        if (normal) {
          *normal = intersection.getWorldIntersectNormal();
	  normal->normalize();
        }

        SOLEIL__LOGGER_DEBUG(
          "COLLISION between collider and ",
          intersection.nodePath.back()->getName(),
          ". Normal: ", intersection.getWorldIntersectNormal());

        return true;
      } else {
        SOLEIL__LOGGER_DEBUG("ALMOST collision between collider and ",
                             intersection.nodePath.back()->getName(),
                             ". Distance=", intersection.distance);
      }
    }
  }
  return false;
}

// TODO: Idea -> Each frame compute all the bounding box in specific position
