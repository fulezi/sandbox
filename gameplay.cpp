

#include <osg/io_utils>

#include "MovementCallback.h"
#include "SceneManager.h"
#include "Utils.h"
#include "gameplay.h"

#include <cassert>
#include <iostream>
#include <vector>

osg::ref_ptr<osg::MatrixTransform> player;

Soleil::Movement* movement;

void
initGame(osg::ref_ptr<osg::MatrixTransform> playerNode)
{
  player = playerNode;

  osg::ref_ptr<Soleil::MovementCallback> mov = new Soleil::MovementCallback;
  playerNode->addUpdateCallback(mov);
  playerNode->setDataVariance(osg::Object::DYNAMIC);
  movement = &(mov->movement);
  movement->mass = 1;
}

void
updateGameplay(const double delta, const Inputs& PlayerInputs)
{
  constexpr float playerSpeed = 3.0f;

  const auto      playerMatrix   = player->getMatrix();
  const osg::Vec3 playerPosition = playerMatrix.getTrans();

  movement->force.x() = PlayerInputs.dpad.x() * playerSpeed;
  movement->force.y() = PlayerInputs.dpad.y() * playerSpeed;
  if (PlayerInputs.jump && playerPosition.z() <= 0.0f) {
    movement->velocity.z() = 100.0f / movement->mass;
  }

}
