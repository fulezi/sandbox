

#include <osg/io_utils>

#include "MovementCallback.h"
#include "SceneManager.h"
#include "Utils.h"
#include "gameplay.h"

#include <cassert>
#include <iostream>
#include <vector>

osg::ref_ptr<osg::MatrixTransform> player;

Soleil::Movement*                      movement;
osg::ref_ptr<Soleil::MovementCallback> mov;

void
initGame(osg::ref_ptr<osg::MatrixTransform> playerNode)
{
  player = playerNode;

  mov = new Soleil::MovementCallback;
  playerNode->addUpdateCallback(mov);
  playerNode->setDataVariance(osg::Object::DYNAMIC);
  movement       = &(mov->movement);
  movement->mass = 1;
}

void
updateGameplay(const double delta, const Inputs& PlayerInputs)
{
  constexpr float playerSpeed = 3.0f;

  const auto      playerMatrix   = player->getMatrix();
  const osg::Vec3 playerPosition = playerMatrix.getTrans();

#if 1
  movement->force.x() = PlayerInputs.dpad.x() * playerSpeed;
  movement->force.y() = PlayerInputs.dpad.y() * playerSpeed;
  if (PlayerInputs.jump && playerPosition.z() <= 0.0f) {
    movement->velocity.z() = 100.0f / movement->mass;
  }
#else
  // ia test
  movement->force.x() = 0.0f * playerSpeed;
  movement->force.y() = 1.0f * playerSpeed;
  if (mov->distanceToObject > 0.0f && mov->distanceToObject < 3.0f &&
      playerPosition.z() <= 0.0f) {
    movement->velocity.z() = 100.0f / movement->mass;
  }

// if (PlayerInputs.jump && playerPosition.z() <= 0.0f) {
//   movement->velocity.z() = 100.0f / movement->mass;
// }
#endif
}
