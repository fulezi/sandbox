/*
 * Copyright (C) 2017  Florian GOLESTIN
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <cassert>

#include <osg/BlendEquation>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/TexGen>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <osg/io_utils>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgGA/CameraManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowedScene>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ViewDependentShadowMap>
#include <osgShadow/ViewDependentShadowTechnique>
#include <osgViewer/Viewer>

#include <osgViewer/ViewerEventHandlers>

#include <osg/Timer>

#include "Logger.h"
#include "Ribbon.h"
#include "SceneManager.h"
#include "SkyBox.h"
#include "Time.h"
#include "Utils.h"
#include "easing.h"
#include "stringutils.h"

#include "gameplay.h"

/* --- Singletons --- */

Inputs PlayerInputs;

/* --- Singletons --- */

/* --- FollowNodeCamera --- */
class FollowNodeCamera : public osgGA::CameraManipulator
{
public:
  FollowNodeCamera(osg::Transform* target, osg::Vec3 offset);

public:
  osg::Matrixd getMatrix() const override;
  osg::Matrixd getInverseMatrix() const override;
  bool handle(const osgGA::GUIEventAdapter& event,
              osgGA::GUIActionAdapter&      action) override;

  void MyupdateCamera();

public:
  void setByMatrix(const osg::Matrixd& matrix) override;
  void setByInverseMatrix(const osg::Matrixd& matrix) override;

protected:
  void updateKeyboard(const osgGA::GUIEventAdapter& event,
                      osgGA::GUIActionAdapter&      action);

protected:
  osg::Transform* target;
  osg::Vec3       offset;
  float           smoothSpeed;

  osg::Vec3   currentPosition;
  osg::Vec3   currentCenter;
  osg::Matrix viewMatrix;
};

FollowNodeCamera::FollowNodeCamera(osg::Transform* target, osg::Vec3 offset)
  : target(target)
  , offset(offset)
  , smoothSpeed(1.0f)
  , currentPosition(target->getBound().center() + offset)
  , currentCenter(target->getBound().center())
{
}

osg::Matrixd
FollowNodeCamera::getMatrix() const
{
  return viewMatrix;
}

osg::Matrixd
FollowNodeCamera::getInverseMatrix() const
{
  return osg::Matrix::inverse(getMatrix());
}

void
FollowNodeCamera::setByMatrix(const osg::Matrixd& /*matrix*/)
{
  assert(false && "// TODO: ");
}

void
FollowNodeCamera::setByInverseMatrix(const osg::Matrixd& /*matrix*/)
{
  assert(false && "// TODO: ");
}

void
FollowNodeCamera::updateKeyboard(const osgGA::GUIEventAdapter& event,
                                 osgGA::GUIActionAdapter& /*action*/)
{
  using KEY  = osgGA::GUIEventAdapter::KeySymbol;
  using TYPE = osgGA::GUIEventAdapter::EventType;

  const float movement = (event.getEventType() == TYPE::KEYDOWN) ? 1.0f : 0.0f;
  switch (event.getKey()) {
    case KEY::KEY_A: PlayerInputs.dpad.x() = -movement; break;
    case KEY::KEY_D: PlayerInputs.dpad.x() = movement; break;
    case KEY::KEY_W: PlayerInputs.dpad.y() = movement; break;
    case KEY::KEY_S: PlayerInputs.dpad.y() = -movement; break;
    case KEY::KEY_Space:
      PlayerInputs.jump =
        (event.getEventType() == TYPE::KEYDOWN) ? true : false;
      break;
  }
}

bool
FollowNodeCamera::handle(const osgGA::GUIEventAdapter& event,
                         osgGA::GUIActionAdapter&      action)
{
  using TYPE = osgGA::GUIEventAdapter::EventType;

  // ------------------
  // Update inputs ----
  switch (event.getEventType()) {
    case TYPE::KEYDOWN:
      updateKeyboard(event, action);
      return true;
      break;
    case TYPE::KEYUP:
      updateKeyboard(event, action);
      return true;
      break;
    default: return false;
  }
}

void
FollowNodeCamera::MyupdateCamera()
{
#define SMOOTH_CAM 1

// -----------------------
// Update view matrix ----
#if SMOOTH_CAM
  const osg::Vec3& targetCenter = target->getBound().center();
  currentCenter                 = mix(currentCenter, targetCenter, smoothSpeed);
#else
  const osg::Vec3& targetCenter = target->getBound().center();
  currentCenter                 = targetCenter;
#endif

#if SMOOTH_CAM
  const osg::Vec3 finalTranslation = targetCenter + offset;
  currentPosition = mix(currentPosition, finalTranslation, smoothSpeed);
#else
  const osg::Vec3 finalTranslation = targetCenter + offset;
  currentPosition                  = finalTranslation;
#endif

  viewMatrix = osg::Matrix::inverse(osg::Matrix::lookAt(
    currentPosition, currentCenter, osg::Vec3(0.0f, 0.0f, 1.0f)));

#undef SMOOTH_CAM
}

/* --- FollowNodeCamera --- */

/* --- Node Callback --- */

class UpdateGamePlay : public osg::NodeCallback
{
public:
  void operator()(osg::Node* node, osg::NodeVisitor* nv) override;

private:
  double lastUpdate;
};

void
UpdateGamePlay::operator()(osg::Node* /*node*/, osg::NodeVisitor* nv)
{
  const double newTime = nv->getFrameStamp()->getReferenceTime();
  const double delta   = nv->getFrameStamp()->getReferenceTime() - lastUpdate;

  Soleil::Time::StartFrame(nv->getFrameStamp()->getReferenceTime());
  updateGameplay(delta, PlayerInputs);

  lastUpdate = newTime;
}

/* --- Node Callback --- */

int
main(int /*argc*/, char* const /*argv*/[])
{

  ///////////////////////
  // Setting the Scene //
  ///////////////////////

  unsigned int rcvShadowMask  = 1;
  unsigned int castShadowMask = 2;
  unsigned int collisionMask  = 4;
  unsigned int renderMask     = 16;

  osg::ref_ptr<osg::LightSource> source = new osg::LightSource;
  // source->getLight()->setPosition(osg::Vec4(-4.0, -4.0, 15.0, 0.0));
  // source->getLight()->setDirection(osg::Vec3(0.0f, 0.0f, 0.0f));
  source->getLight()->setPosition(osg::Vec4(-1.0, -1.0, 1.0, 0.0));
  source->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
  source->getLight()->setDiffuse(osg::Vec4(0.5, 0.5, 0.5, 1.0));
  source->getLight()->setLightNum(0);
  source->setName("SUN");
  source->setCullingActive(false);
  //
  osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  // sm->setAmbientBias(osg::Vec2(0.9f, 0.9f));
  // sm->setPolygonOffset(osg::Vec2(.20f, .20f));
  // sm->setLight(source);
  // sm->setTextureSize(osg::Vec2s(2048, 2048));
  // sm->setTextureUnit(1);
  //   osg::ref_ptr<osgShadow::SoftShadowMap> sm =
  //     new osgShadow::SoftShadowMap;
  // sm->setSoftnessWidth(.00000001f);
  // osg::ref_ptr<osgShadow::ViewDependentShadowMap> sm =
  //   new osgShadow::ViewDependentShadowMap;
  // osg::ref_ptr<osgShadow::LightSpacePerspectiveShadowMapVB> sm =
  //   new osgShadow::LightSpacePerspectiveShadowMapVB;
  //
  osg::ref_ptr<osgShadow::ShadowedScene> shadowroot =
    new osgShadow::ShadowedScene;
  shadowroot->setShadowTechnique(sm);
  shadowroot->setReceivesShadowTraversalMask(rcvShadowMask);
  shadowroot->setCastsShadowTraversalMask(castShadowMask);
  //
  shadowroot->addChild(source);
  //

  osg::ref_ptr<osg::Group> root = new osg::Group();
  osg::ref_ptr<osg::Node>  skycube =
    osgDB::readNodeFile("../media/ZincSkybox.osgt");
  assert(skycube);
  osg::ref_ptr<SkyBox> skybox = new SkyBox;
  skybox->addChild(skycube);
  skybox->setName("Skybox");
  skycube->setName("SkyCube");
  osg::ref_ptr<osg::Node> floor =
    osgDB::readNodeFile("../media/PlaneFloor.osgt");
  floor->setNodeMask(rcvShadowMask);
  floor->setName("Floor");
  shadowroot->addChild(floor);
  root->addChild(skybox);

  osg::ref_ptr<osg::Node> playerNode =
    osgDB::readNodeFile("../media/CubePlayer.osgt");
  osg::ref_ptr<osg::MatrixTransform> player = new osg::MatrixTransform;
  player->addChild(playerNode);
  player->setName("Player");
  playerNode->setName("PlayerNode");
  player->setNodeMask(rcvShadowMask | castShadowMask);
  shadowroot->addChild(player);
  osg::ref_ptr<FollowNodeCamera> cameraman =
    new FollowNodeCamera(player, osg::Vec3(0, -35, 15));
  root->addEventCallback(new UpdateGamePlay());

  // Ribbon:
  osg::ref_ptr<Soleil::Ribbon> ribbon =
    new Soleil::Ribbon(100, 2.0f, osg::Vec3(1.0f, 0.0f, 1.0f));
  osg::ref_ptr<osg::Geode> ribbons = new osg::Geode;
  ribbons->addDrawable(ribbon);
  ribbons->getOrCreateStateSet()->setMode(GL_LIGHTING,
                                          osg::StateAttribute::OFF);
  ribbons->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  ribbons->getOrCreateStateSet()->setRenderingHint(
    osg::StateSet::TRANSPARENT_BIN);
  root->addChild(ribbons);
  // // TODO: on this node?
  player->addUpdateCallback(new Soleil::RibbonCallback(ribbon));

  // root->addChild(sm->makeDebugHUD());

  // Obstacle:
  osg::ref_ptr<osg::Node> ObstacleNode =
    osgDB::readNodeFile("../media/Obstacle.osgt");
  osg::ref_ptr<osg::Group> obstacles = new osg::Group;
  for (int i = 0; i < 5; ++i) {
    osg::ref_ptr<osg::MatrixTransform> t = new osg::MatrixTransform;
    t->setMatrix(osg::Matrix::translate(
      Random(-30.0f, 20.0f), Random(10.0f, 30.0f) * 2.0f * (float)i, 0.0f));
    t->addChild(ObstacleNode);
    t->setNodeMask(rcvShadowMask | castShadowMask);
    t->setName("Obstacle");

    obstacles->addChild(t);
  }
  obstacles->setName("Obstacle Group");
  obstacles->setNodeMask(rcvShadowMask | castShadowMask);
#if 0
  // TODO: Shadow should be baked at this point. In addition, the sadow map is
  // too small to include all the elements. It however render correctly with
  // ViewDependentshadowmap
  shadowroot->addChild(obstacles);
#else
  root->addChild(obstacles);
#endif

  osgViewer::Viewer viewer;
  viewer.setLightingMode(osg::View::NO_LIGHT);
  viewer.setCameraManipulator(cameraman);

  // ribbon->setNodeMask(ribbon->getNodeMask() & ~rcvShadowMask);
  root->addChild(shadowroot);
  viewer.setSceneData(root);

  osg::ref_ptr<osgViewer::StatsHandler> stats = new osgViewer::StatsHandler;
  stats->setKeyEventTogglesOnScreenStats(
    osgGA::GUIEventAdapter::KeySymbol::KEY_F1);
  stats->setKeyEventPrintsOutStats(osgGA::GUIEventAdapter::KeySymbol::KEY_F2);
  viewer.addEventHandler(stats);

  SceneManager::Init(obstacles);
  SceneManager::RegisterRigidBody(*player);
// Do render only debug:
#if 0
  {
    for (unsigned int i = 0; i < root->getNumChildren(); ++i) {
      root->getChild(i)->setNodeMask(root->getChild(i)->getNodeMask() &
                                     ~renderMask);
    }
    viewer.getCamera()->setCullMask(renderMask);

    root->addChild(SceneManager::DebugGenerateRigidBodiesShapes());
  }
#endif
  initGame(player);

  osgViewer::ViewerBase::ThreadingModel th =
    osgViewer::ViewerBase::SingleThreaded;
  viewer.setThreadingModel(th);
  viewer.setRunMaxFrameRate(30);

  // return viewer.run();
  cameraman->MyupdateCamera();
  while (!viewer.done()) {
    viewer.frame();

    // Update the camera after all node where updated
    cameraman->MyupdateCamera();

    // std::string txt;
    // std::cin >> txt;
  }
  return 0;
}
