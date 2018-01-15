
#ifndef SOLEIL__SCENEMANAGER_H_
#define SOLEIL__SCENEMANAGER_H_

#include <map>
#include <osg/Group>
#include <vector>
#include <osg/BoundingBox>


struct RigidBody
{
  osg::BoundingBox    box;
  osg::BoundingSphere sphere;
};

class SceneManager
{
public:
  // typedef osg::ref_ptr<osg::Node> KeyRigigBody;
  // typedef std::map<KeyRigigBody, RigidBody> RigidBodiesList;
  typedef std::vector<RigidBody> RigidBodiesList;

public:
  // static SceneManager* Instance(void);
  static void Init(osg::ref_ptr<osg::Group> sceneRoot);

public:
  // Deprecated?
  static bool IsColliding(osg::Node* node);
  static bool RayCollision(osg::Node& node, const osg::Vec3& direction, osg::Vec3* normal = nullptr);
  static void RegisterRigidBody(osg::Node& node);

public:
  osg::ref_ptr<osg::Group> sceneRoot;
  RigidBodiesList          rigidBodies;

  // Options -----------------------------
  // public:
  //   bool renderGameObjects;

  // Debug -------------------------------
public:
  static osg::ref_ptr<osg::Group> DebugGenerateRigidBodiesShapes();
  osg::ref_ptr<osg::Group>        DebugBodiesShape;
  // std::map <KeyRigigBody, osg::ref_ptr<osg::MatrixTransform>
  // DebugBodieToShape;
  static void DebugAddBodyShape(const RigidBody& body);
};

#endif /* SOLEIL__SCENEMANAGER_H_ */
