
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
SceneManager::RayCollision(osg::Node& node, const osg::Vec3& direction,
                           osg::Vec3* normal)
{
  osg::ref_ptr<osgUtil::RayIntersector> ray = new osgUtil::RayIntersector(
    osgUtil::Intersector::CoordinateFrame::MODEL, node.getBound().center(),
    direction, nullptr, osgUtil::Intersector::IntersectionLimit::LIMIT_NEAREST);
  osgUtil::IntersectionVisitor visitor(ray);

  sceneManager->sceneRoot->accept(visitor);

  // TODO: Do polytope fetch. Collision is taken from the center of mass wich
  // allows wings to avoid colliding
  

  if (ray->containsIntersections()) {
    // TODO: Do polytope intersection
    if (node.getBound().contains(
          ray->getFirstIntersection().getWorldIntersectPoint())) {
      if (normal) {
        *normal = ray->getFirstIntersection().getLocalIntersectNormal();
      }

      SOLEIL__LOGGER_DEBUG(
        "COLLISION between ", node.getName(), " and ",
        ray->getFirstIntersection().nodePath.back()->getName());

      return true;
    }
  }
  return false;
}

// TODO: Idea -> Each frame compute all the bounding box in specific position
