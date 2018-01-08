

#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/Group>

struct Inputs
{
  osg::Vec3 dpad;
  bool      jump;

  Inputs()
    : dpad(0.0f, 0.0f, 0.0f)
    , jump(false)
  {
  }
};

void initGame(osg::ref_ptr<osg::MatrixTransform> player);
void updateGameplay(const double delta, const Inputs& PlayerInputs);
