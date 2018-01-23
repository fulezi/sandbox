
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
  static bool SegmentCollision(const osg::Vec3& start, const osg::Vec3& end,
                               osg::Vec3* normal           = nullptr,
                               float*     distanceToObject = nullptr);

public:
  osg::ref_ptr<osg::Group> sceneRoot;

  // Debug -------------------------------
};

#endif /* SOLEIL__SCENEMANAGER_H_ */
