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

#include <osg/Timer>

#include "Logger.h"
#include "SkyBox.h"
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

public:
  void setByMatrix(const osg::Matrixd& matrix) override;
  void setByInverseMatrix(const osg::Matrixd& matrix) override;

protected:
  void updateKeyboard(const osgGA::GUIEventAdapter& event,
                      osgGA::GUIActionAdapter&      action);

protected:
  osg::Transform* target;
  osg::Vec3       offset;
};

FollowNodeCamera::FollowNodeCamera(osg::Transform* target, osg::Vec3 offset)
  : target(target)
  , offset(offset)
{
}

osg::Matrixd
FollowNodeCamera::getMatrix() const
{
  const osg::Vec3& targetCenter = target->getBound().center();
  return osg::Matrix::inverse(osg::Matrix::lookAt(
    targetCenter + offset, targetCenter, osg::Vec3(0, 0, 1)));
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
                                 osgGA::GUIActionAdapter&      action)
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
UpdateGamePlay::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
  const double newTime = nv->getFrameStamp()->getReferenceTime();
  const double delta   = nv->getFrameStamp()->getReferenceTime() - lastUpdate;

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

  unsigned int rcvShadowMask  = 0x1;
  unsigned int castShadowMask = 0x2;

  osg::ref_ptr<osg::LightSource> source = new osg::LightSource;
  // source->getLight()->setPosition(osg::Vec4(-4.0, -4.0, 15.0, 0.0));
  // source->getLight()->setDirection(osg::Vec3(0.0f, 0.0f, 0.0f));
  source->getLight()->setPosition(osg::Vec4(-1.0, -1.0, 1.0, 0.0));
  source->getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
  source->getLight()->setDiffuse(osg::Vec4(0.8, 0.8, 0.8, 1.0));
  source->getLight()->setLightNum(0);
  source->setName("SUN");
  source->setCullingActive(false);
  //
  // osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
  // sm->setAmbientBias(osg::Vec2(0.9f, 0.9f));
  // sm->setPolygonOffset(osg::Vec2(.20f, .20f));
  // sm->setLight(source);
  // sm->setTextureSize(osg::Vec2s(2048, 2048));
  // sm->setTextureUnit(1);
  //   osg::ref_ptr<osgShadow::SoftShadowMap> sm =
  //     new osgShadow::SoftShadowMap;
  // sm->setSoftnessWidth(.00000001f);
  osg::ref_ptr<osgShadow::ViewDependentShadowMap> sm =
    new osgShadow::ViewDependentShadowMap;
  // osg::ref_ptr<osgShadow::LightSpacePerspectiveShadowMapCB> sm =
  //   new osgShadow::LightSpacePerspectiveShadowMapCB;
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
  osg::ref_ptr<osg::Node> floor =
    osgDB::readNodeFile("../media/PlaneFloor.osgt");
  floor->setNodeMask(rcvShadowMask);
  shadowroot->addChild(floor);
  root->addChild(skybox);
  // root->addChild(shadowroot);
  shadowroot->addChild(root);

  osg::ref_ptr<osg::Node> playerNode =
    osgDB::readNodeFile("../media/CubePlayer.osgt");
  osg::ref_ptr<osg::MatrixTransform> player = new osg::MatrixTransform;
  player->addChild(playerNode);
  player->setNodeMask(rcvShadowMask | castShadowMask); // rcvShadowMask |
  root->addChild(player);
  osg::ref_ptr<FollowNodeCamera> cameraman =
    new FollowNodeCamera(player, osg::Vec3(0, -35, 15));
  root->addEventCallback(new UpdateGamePlay());

  // root->addChild(sm->makeDebugHUD());

  osgViewer::Viewer viewer;
  viewer.setLightingMode(osg::View::NO_LIGHT);
  viewer.setCameraManipulator(cameraman);
  viewer.setSceneData(shadowroot);

  initGame(player);

  return viewer.run();
}
