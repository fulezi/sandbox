
#include <memory>

#include "SceneManager.h"

#include "Logger.h"

#include <functional>
#include <osg/ComputeBoundsVisitor>
#include <osg/Drawable>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

static std::unique_ptr<SceneManager> sceneManager;

// ComputeRigidBodiesVisitor ---------------------------
struct ComputeRigidBodiesVisitor : public osg::NodeVisitor
{
public:
  ComputeRigidBodiesVisitor(osg::ref_ptr<osg::Group> sceneRoot);

public:
  void apply(osg::Drawable& node) override;

public:
  osg::ref_ptr<osg::Group> sceneRoot;
};

ComputeRigidBodiesVisitor::ComputeRigidBodiesVisitor(
  osg::ref_ptr<osg::Group> sceneRoot)
  : osg::NodeVisitor(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN)
  , sceneRoot(sceneRoot)
{
}

void
ComputeRigidBodiesVisitor::apply(osg::Drawable& node)
{
  // osg::ref_ptr<osg::Node> referenced = &node;
  // if (sceneManager->rigidBodies.count(referenced) > 0) return;

  const osg::Matrix worldSpace = osg::computeLocalToWorld(this->getNodePath());

  const osg::BoundingSphere boundsWS(node.getBound().center() * worldSpace,
                                     node.getBound().radius());

  const osg::BoundingBox& nodeBox = node.getBoundingBox();
  osg::BoundingBox        boxWS;
  boxWS.expandBy(osg::Vec3(nodeBox.xMin(), nodeBox.yMin(), nodeBox.zMin()) *
                 worldSpace);
  boxWS.expandBy(osg::Vec3(nodeBox.xMax(), nodeBox.yMax(), nodeBox.zMax()) *
                 worldSpace);

  RigidBody body{boxWS, boundsWS};
  // sceneManager->rigidBodies[referenced] = body;
  sceneManager->rigidBodies.push_back(body); // TODO: Node name
}
// ComputeRigidBodiesVisitor ---------------------------

void
SceneManager::Init(osg::ref_ptr<osg::Group> sceneRoot)
{
  sceneManager            = std::make_unique<SceneManager>();
  sceneManager->sceneRoot = sceneRoot;

  ComputeRigidBodiesVisitor visitor(sceneRoot);
  sceneRoot->accept(visitor);

  SOLEIL__LOGGER_DEBUG("Initialized ", sceneManager->rigidBodies.size(),
                       " rigid bodies.");
}

// --- Collision ------------------------------

void
SceneManager::RegisterRigidBody(osg::Node& node)
{
  osg::ComputeBoundsVisitor visitor;
  node.accept(visitor);

  // assert(sceneManager->rigidBodies.count(&node) < 1 &&
  //        "Update not yet supported");

  // sceneManager->rigidBodies[&node] =
  //   RigidBody{visitor.getBoundingBox(), node.getBound()};

  SOLEIL__LOGGER_DEBUG("// TODO: Usefull?");
}

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
  RigidBody  colliderBody;
};

PruningBoxVisitor::PruningBoxVisitor(osg::Node* node)
  : osg::NodeVisitor(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN)
  , collider(node)
  , collided(false)
{
  osg::ref_ptr<osg::Node> referenced = node;

  // assert(sceneManager->rigidBodies.count(node) > 0 &&
  //        "Node was node found in RigidBodies");
  // const RigidBody& body = sceneManager->rigidBodies[referenced];
  // colliderBody          = body;

  osg::ComputeBoundsVisitor visitor;
  node->accept(visitor);

  colliderBody = RigidBody{visitor.getBoundingBox(), node->getBound()};


  // TODO: Temporary use Key ID based system
  static RigidBody* r = nullptr;

  if (r)
    {
      r->box = colliderBody.box;
      r->sphere = colliderBody.sphere;
    }
  else
    {
      sceneManager->rigidBodies.push_back(colliderBody);
      r = &(sceneManager->rigidBodies.back());
    }

  
  SceneManager::DebugGenerateRigidBodiesShapes();
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

  if (&node != collider && collided) {
    this->traverse(node);
  } // else {
  //   SOLEIL__LOGGER_DEBUG(
  //     "NO collision between ", collider->getName(), " and ", node.getName(),
  //     "(", node.className(), transformGroup, ")\t",
  //     "NODE >>>Center:", debug_bound.center(), "Radius: ", debug_bound.radius(),
  //     " - COLLIDER >>>Center:", collider->getBound().center(),
  //     "Radius: ", collider->getBound().radius());
  // }
}

void
PruningBoxVisitor::apply(osg::Drawable& node)
{
#if 0
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
#else
  const osg::Matrix worldSpace = osg::computeLocalToWorld(this->getNodePath());
  const osg::BoundingBox& bounds = node.getBoundingBox();
  osg::BoundingBox        boundsWS;
  boundsWS.expandBy(osg::Vec3(bounds.xMin(), bounds.yMin(), bounds.zMin()) *
                 worldSpace);
  boundsWS.expandBy(osg::Vec3(bounds.xMax(), bounds.yMax(), bounds.zMax()) *
                 worldSpace);

  const osg::BoundingBox debug_bound = boundsWS;

  if (boundsWS.intersects(colliderBody.box)) {
    collided = true;
    SOLEIL__LOGGER_DEBUG("COLLISION between ", collider->getName(), " and ",
                         node.getName());

    this->traverse(node);
  } else {
    SOLEIL__LOGGER_DEBUG("NO collision between ", collider->getName(), " and ",
                         node.getName(), "(", node.className(), ")",
                         "NODE >>>Center:", debug_bound.center(),
                         "Radius: ", debug_bound.radius(),
                         " - COLLIDER >>>Center:", colliderBody.box.center(),
                         "Radius: ", colliderBody.box.radius());
  }

#endif
}

bool
SceneManager::IsColliding(osg::Node* node)
{
  PruningBoxVisitor visitor(node);

  sceneManager->sceneRoot->accept(visitor);
  return visitor.collided;
}

// Debug -----------------------------------------------

// ComputeRigiBodyShapes ---------------------------
// struct ComputeRigiBodyShapes : public osg::NodeVisitor
// {
// public:
//   ComputeRigiBodyShapes();

// public:
//   void apply(osg::Drawable& node) override;

// public:
//   osg::ref_ptr<osg::Group> sceneRoot;
// };

// ComputeRigiBodyShapes::ComputeRigiBodyShapes()
//   : osg::NodeVisitor(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN)
//   , sceneRoot(sceneRoot)
// {
// }

// void
// ComputeRigiBodyShapes::apply(osg::Drawable& node)
// {
//   osg::ref_ptr<osg::Node> referenced = &node;
//   if (sceneManager->rigidBodies.count(referenced) > 0) return;

//   RigidBody body{node.getBoundingBox(), node.getBound()};
//   sceneManager->rigidBodies[referenced] = body;
// }
// ComputeRigiBodyShapes ---------------------------

osg::ref_ptr<osg::Group>
SceneManager::DebugGenerateRigidBodiesShapes()
{
  if (sceneManager->DebugBodiesShape) {
    sceneManager->DebugBodiesShape->removeChildren(
      0, sceneManager->DebugBodiesShape->getNumChildren());
  } else {
    sceneManager->DebugBodiesShape = new osg::Group;
    sceneManager->DebugBodiesShape->getOrCreateStateSet()->setMode(
      GL_LIGHTING, osg::StateAttribute::OFF);
    sceneManager->DebugBodiesShape->getOrCreateStateSet()->setMode(
      GL_BLEND, osg::StateAttribute::ON);
    sceneManager->DebugBodiesShape->getOrCreateStateSet()->setRenderingHint(
      osg::StateSet::TRANSPARENT_BIN);
    sceneManager->DebugBodiesShape->setDataVariance(
      osg::Object::DataVariance::DYNAMIC);
  }

  for (const auto& body : sceneManager->rigidBodies) {
    SceneManager::DebugAddBodyShape(body);
  }

  return sceneManager->DebugBodiesShape;
}

void
SceneManager::DebugAddBodyShape(const RigidBody& body)
{
  assert(sceneManager->DebugBodiesShape);

  osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;

  // Box
  {
    osg::ref_ptr<osg::Box> shape =
      new osg::Box(body.box.center(), body.box.radius());
    osg::ref_ptr<osg::ShapeDrawable> drawableBox =
      new osg::ShapeDrawable(shape);
    drawableBox->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 0.75f));
    mt->addChild(drawableBox);
  }

  // Sphere
  {
    osg::ref_ptr<osg::Sphere> shape =
      new osg::Sphere(body.sphere.center(), body.box.radius());
    osg::ref_ptr<osg::ShapeDrawable> drawableBox =
      new osg::ShapeDrawable(shape);
    drawableBox->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.25f));
    mt->addChild(drawableBox);
  }

  sceneManager->DebugBodiesShape->addChild(mt);
}

// TODO: Idea -> Each frame compute all the bounding box in specific position
