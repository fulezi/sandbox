
#include <memory>

#include "SceneManager.h"

#include "Logger.h"

#include <osg/Drawable>

static std::unique_ptr<SceneManager> sceneManager;

void
SceneManager::Init(osg::ref_ptr<osg::Group> sceneRoot)
{
  sceneManager            = std::make_unique<SceneManager>();
  sceneManager->sceneRoot = sceneRoot;
}

// --- Collision ------------------------------

struct PruningBoxVisitor : public osg::NodeVisitor
{
public:
  PruningBoxVisitor(osg::Node* node);

public:
  void apply(osg::Node& node) override;
  void apply(osg::Drawable& node) override;

public:
  osg::Node* collider;
  bool       collided;
};

PruningBoxVisitor::PruningBoxVisitor(osg::Node* node)
  : osg::NodeVisitor(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN)
  , collider(node)
  , collided(false)
{
}

void
PruningBoxVisitor::apply(osg::Node& node)
{
  osg::Transform* transformGroup = node.asTransform();

  osg::BoundingSphere debug_bound;

  bool collided = false;
  /* If the group is from the Transform familiy, the resulting bounding sphere
   * seems to be already in world space. However if it's a Group the bounding
   * box is in the local space (actually Group does not have a notion of
   * position) */
  if (transformGroup) {
    assert(std::string(node.className()).compare("Group") != 0);

    debug_bound = node.getBound();

    if (node.getBound().intersects(collider->getBound())) {
      collided = true;
    }
  } else {
    // if (true) {
    // const osg::Matrix worldSpace =
    //   osg::computeLocalToWorld(this->getNodePath());
    const osg::Matrix worldSpace =
      osg::computeLocalToWorld(this->getNodePath());
    const osg::BoundingSphere& bounds = node.getBound();
    const osg::BoundingSphere  boundsWS(bounds.center() * worldSpace,
                                       bounds.radius());

    // for (const auto p : this->getNodePath()) {
    //   SOLEIL__LOGGER_DEBUG("> ", p->getName(), ": ", p->className());
    // }

    // SOLEIL__LOGGER_DEBUG(node.getName(), ": ", node.className(),
    //                      "----------------");
    // SOLEIL__LOGGER_DEBUG(worldSpace);
    // SOLEIL__LOGGER_DEBUG("1=", worldSpace * osg::Vec4(bounds.center(),
    // 1.0f));
    // SOLEIL__LOGGER_DEBUG(
    //   "2=", osg::Matrix::translate(10, 1, 1) * osg::Vec3(2.0f, 2.0f, 2.0f));
    // SOLEIL__LOGGER_DEBUG("3=",
    //                      osg::Matrix::translate(10, 1, 1) *
    //                        osg::Vec4(2.0f, 2.0f, 2.0f, 1.0f));
    // SOLEIL__LOGGER_DEBUG("3=",
    //                      osg::Matrix::translate(10, 1, 1).preMult(
    //                        osg::Vec4(2.0f, 2.0f, 2.0f, 1.0f)));
    // SOLEIL__LOGGER_DEBUG("4=", bounds.center() * worldSpace);

    debug_bound = boundsWS;
    if (boundsWS.intersects(collider->getBound())) {
      collided = true;
    }
  }

  // &node != collider &&
  if (collided) {
    this->traverse(node);
  } else {
    SOLEIL__LOGGER_DEBUG(
      "NO collision between ", collider->getName(), " and ", node.getName(),
      "(", node.className(), transformGroup, ")\t",
      "NODE >>>Center:", debug_bound.center(), "Radius: ", debug_bound.radius(),
      " - COLLIDER >>>Center:", collider->getBound().center(),
      "Radius: ", collider->getBound().radius());
  }
}

void
PruningBoxVisitor::apply(osg::Drawable& node)
{
  const osg::Matrix worldSpace = osg::computeLocalToWorld(this->getNodePath());
  const osg::BoundingSphere& bounds = node.getBound();
  const osg::BoundingSphere  boundsWS(bounds.center() * worldSpace,
                                     bounds.radius());

  const osg::BoundingSphere debug_bound = boundsWS;

  if (boundsWS.intersects(collider->getBound())) {
    collided = true;
    SOLEIL__LOGGER_DEBUG("COLLISION between ", collider->getName(), " and ",
                         node.getName());

    this->traverse(node);
  } else {
    SOLEIL__LOGGER_DEBUG(
      "NO collision between ", collider->getName(), " and ", node.getName(),
      "(", node.className(), ")", "NODE >>>Center:", debug_bound.center(),
      "Radius: ", debug_bound.radius(),
      " - COLLIDER >>>Center:", collider->getBound().center(),
      "Radius: ", collider->getBound().radius());
  }
}

bool
SceneManager::IsColliding(osg::Node* node)
{
  PruningBoxVisitor visitor(node);

  sceneManager->sceneRoot->accept(visitor);
  return visitor.collided;
}
