
#ifndef SOLEIL__SCENEMANAGER_H_
#define SOLEIL__SCENEMANAGER_H_

#include <map>
#include <osg/BoundingBox>
#include <osg/Group>
#include <osgParticle/Emitter>
#include <osgParticle/ParticleSystem>
#include <vector>

class SceneManager
{
public:
  // static SceneManager* Instance(void);
  static void Init(osg::ref_ptr<osg::Group> sceneRoot);

public: // Particles ------------
  typedef std::size_t ObjectID;

  static void RegisterParticleSystem(
    ObjectID id, osg::ref_ptr<osgParticle::ParticleSystem> system);
  static void AddParticleEmitter(ObjectID particleSystemID,
                                 osg::ref_ptr<osgParticle::Emitter> emitter);

protected:
  std::map<ObjectID, osg::ref_ptr<osgParticle::ParticleSystem>> particleSystems;

public:
  static bool SegmentCollision(const osg::Vec3& start, const osg::Vec3& end,
                               osg::Vec3* normal           = nullptr,
                               float*     distanceToObject = nullptr);

public:
  osg::ref_ptr<osg::Group> sceneRoot;

  // Debug -------------------------------
};

#endif /* SOLEIL__SCENEMANAGER_H_ */
