
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

// TODO: Idea -> Each frame compute all the bounding box in specific position
