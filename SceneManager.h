
#ifndef SOLEIL__SCENEMANAGER_H_
#define SOLEIL__SCENEMANAGER_H_

#include <map>
#include <osg/BoundingBox>
#include <osg/Group>
#include <vector>

class SceneManager
{
public:
  // static SceneManager* Instance(void);
  static void Init(osg::ref_ptr<osg::Group> sceneRoot);

public:
  static bool RayCollision(const osg::BoundingBox& box,
                           const osg::Vec3&        direction,
                           osg::Vec3*              normal           = nullptr,
                           float*                  distanceToObject = nullptr);

public:
  osg::ref_ptr<osg::Group> sceneRoot;

  // Debug -------------------------------
};

#endif /* SOLEIL__SCENEMANAGER_H_ */
