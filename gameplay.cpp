

#include <osg/io_utils>

#include "gameplay.h"

#include <cassert>
#include <iostream>
#include <vector>

osg::ref_ptr<osg::MatrixTransform> player;

/* --- Movement --- */
struct Movement
{
  osg::Vec3 force;
  osg::Vec3 velocity;
  osg::Vec3 friction;
  float     mass;

  osg::MatrixTransform* target;

  void update(const float deltaTime);
  Movement(osg::MatrixTransform* target);
};

Movement::Movement(osg::MatrixTransform* target)
  : force(0.0f, 0.0f, 0.0f)
  , velocity(0.0f, 0.0f, 0.0f)
  , friction(0.1f, 0.1f, 0.0f)
  , mass(10.0f)
  , target(target)
{
}

void
Movement::update(const float deltaTime)
{
  static float Time = 0.0f;

  Time += deltaTime;
  const osg::Vec3 acceleration = force / mass;
  // velocity += acceleration - osg::componentMultiply(friction, velocity);
  // v = (a/f) - (a/f)*exp(-f*t)
  const osg::Vec3 expMinusFTimeT =
    osg::componentMultiply((-friction * Time), (-friction * Time));
  std::cout << "expMinusFTimeT:" << expMinusFTimeT << "\n";
  const osg::Vec3 accelerationDivFriction(
    acceleration.x() / friction.x(), acceleration.y() / friction.y(), 0.0f);
  std::cout << "accelerationDivFriction" << accelerationDivFriction << "\n";

  velocity = accelerationDivFriction -
             osg::componentMultiply(accelerationDivFriction, expMinusFTimeT);

  target->setMatrix(target->getMatrix() * osg::Matrix::translate(velocity));
}
/* --- Movement --- */

std::vector<Movement> movements;

void
initGame(osg::ref_ptr<osg::MatrixTransform> playerNode)
{
  player = playerNode;

  assert(movements.size() == 0 &&
         "We assume player to be the first (0th) entry");
  movements.push_back(Movement(playerNode));
}

void
updateGameplay(const double delta, const Inputs& PlayerInputs)
{
  movements[0].force = PlayerInputs.dpad;

  for (auto& mov : movements) {
    mov.update(delta);
  }
  // constexpr float playerSpeed  = 25.0f;
  // constexpr float gravitySpeed = 0.931f;

  // const auto      playerMatrix   = player->getMatrix();
  // const osg::Vec3 playerPosition = playerMatrix.getTrans();

  // osg::Vec3 movement = playerPosition + PlayerInputs.dpad * delta *
  // playerSpeed;
  // if (PlayerInputs.jump && movement.z() <= 0.0f) {
  //   movement.z() = 20.0f;
  // } else if (movement.z() > 0.0f) {
  //   movement.z() = osg::maximum(0.0f, movement.z() - gravitySpeed);
  // }

  // // TODO: Physic
  // player->setMatrix(osg::Matrix::translate(movement));
}
