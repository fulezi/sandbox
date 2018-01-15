

#include <osg/io_utils>

#include "MovementCallback.h"
#include "SceneManager.h"
#include "Utils.h"
#include "gameplay.h"

#include <cassert>
#include <iostream>
#include <vector>

osg::ref_ptr<osg::MatrixTransform> player;

// /* --- Movement --- */
// struct Movement
// {
//   osg::Vec3 force;
//   osg::Vec3 velocity;
//   osg::Vec3 friction;
//   float     mass;

//   osg::Vec3 point;

//   osg::MatrixTransform* target;

//   void update(const float deltaTime);
//   Movement(osg::MatrixTransform* target);

//   // TODO: Temp:
//   osg::ref_ptr<osg::Node> node;
// };

// Movement::Movement(osg::MatrixTransform* target)
//   : force(0.0f, 0.0f, -9.31f)
//   , velocity(0.0f, 0.0f, 0.0f)
//   , friction(0.002f, 0.002f, 1.0f)
//   , mass(1.0f)
//   , point(0.0f, 0.0f, 0.0f)
//   , target(target)
// {
// }

// void
// Movement::update(const float deltaTime)
// {
//   const osg::Vec3 acceleration = force / mass;
//   velocity += acceleration;
//   // float f = std::pow(friction.x(),deltaTime);
//   const osg::Vec3 powFriction(std::pow(friction.x(), deltaTime),
//                               std::pow(friction.y(), deltaTime),
//                               std::pow(friction.z(), deltaTime));
//   velocity = osg::componentMultiply(velocity, powFriction);

//   const osg::Vec3 previousTemp = point;
//   point += velocity * deltaTime;

//   if (point.z() < 0.0f) {
//     point.z() = 0.0f;
//     velocity.z() = 0.0f;
//   }

//   auto m = target->getMatrix();
//   m.setTrans(point);
//   target->setMatrix(m);

//   //if (SceneManager::IsColliding(node)) {
//   osg::Vec3 normal;
//   osg::Vec3 dir(velocity.x(), velocity.y(), 0.0f);
//   dir.normalize();
//   if (SceneManager::RayCollision(*node, dir, &normal)) {
//     point = previousTemp; // TODO: Z velocity

//     // TODO: Clean
//     auto m = target->getMatrix();
//     m.setTrans(point);
//     target->setMatrix(m);

//     // test bounce
//     // velocity.x() = -(velocity.x()) * 3.0f;
//     // velocity.y() = -(velocity.y()) * 3.0f;
//     //velocity = reflect(dir, normal);
//   }
// }
// /* --- Movement --- */

// std::vector<Movement> movements;

Soleil::Movement* movement;

void
initGame(osg::ref_ptr<osg::MatrixTransform> playerNode)
{
  player = playerNode;

  // assert(movements.size() == 0 &&
  //        "We assume player to be the first (0th) entry");
  // movements.push_back(Movement(playerNode));
  // movements[0].point = playerNode->getMatrix().getTrans();
  // movements[0].node  = playerNode;

  osg::ref_ptr<Soleil::MovementCallback> mov = new Soleil::MovementCallback;
  playerNode->addUpdateCallback(mov);
  movement = &(mov->movement);
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
    movement->velocity.z() = 100.0f;
  }

  // for (auto& mov : movements) {
  //   mov.update(delta);
  // }

  // TODO: A Visitor that create Debug Shape
  SceneManager::DebugGenerateRigidBodiesShapes();
}
