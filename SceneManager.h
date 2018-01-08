
#ifndef SOLEIL__SCENEMANAGER_H_
#define SOLEIL__SCENEMANAGER_H_

#include <osg/Group>

class SceneManager
{
public:
  static void Init(osg::ref_ptr<osg::Group> sceneRoot);

public:
  static bool IsColliding(osg::Node* node);

public:
  osg::ref_ptr<osg::Group> sceneRoot;
};

#endif /* SOLEIL__SCENEMANAGER_H_ */
