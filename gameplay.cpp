
#include "gameplay.h"

osg::ref_ptr<osg::MatrixTransform> player;

void
initGame(osg::ref_ptr<osg::MatrixTransform> playerNode)
{
  player = playerNode;
}

void
updateGameplay(const double delta, const Inputs& PlayerInputs)
{
  constexpr float playerSpeed  = 25.0f;
  constexpr float gravitySpeed = 0.931f;

  const auto      playerMatrix   = player->getMatrix();
  const osg::Vec3 playerPosition = playerMatrix.getTrans();

  osg::Vec3 movement = playerPosition + PlayerInputs.dpad * delta * playerSpeed;
  if (PlayerInputs.jump && movement.z() <= 0.0f) {
    movement.z() = 20.0f;
  } else if (movement.z() > 0.0f) {
    movement.z() = osg::maximum(0.0f, movement.z() - gravitySpeed);
  }

  // TODO: Physic
  player->setMatrix(osg::Matrix::translate(movement));
}
