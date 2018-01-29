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
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/RadialShooter>
#include <osgParticle/SectorPlacer>
#include <osgShadow/LightSpacePerspectiveShadowMap>
#include <osgShadow/ParallelSplitShadowMap>
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

#define ZINC__SHADOWMAP 0          // 1 To use shadowmap instead of VDSM
#define ZINC_LEVEL_SHADOW_BAKED 0  // 1 To not add shadow to the loaded level
#define ZINC_DISPLAY_BOUNDINGBOX 0 // 1 To display bounds
#define ZINC_SINGLETHREAD 0
#define ZINC_USETESTLEVEL 0

/* --- Singletons --- */

Inputs PlayerInputs;

/* --- Singletons --- */

/* --- Displays boundingbox --- */
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
class DisplayBoundingBoxVisitor : public osg::NodeVisitor
{
public:
  osg::ref_ptr<osg::Geode> boxes;

public:
  DisplayBoundingBoxVisitor();

public:
  void apply(osg::Drawable& drawable) override;
};

DisplayBoundingBoxVisitor::DisplayBoundingBoxVisitor()
  : NodeVisitor(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN)
{
  boxes = new osg::Geode;
  boxes->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  boxes->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
  boxes->getOrCreateStateSet()->setRenderingHint(
    osg::StateSet::TRANSPARENT_BIN);
  boxes->setDataVariance(osg::Object::DataVariance::DYNAMIC);
  boxes->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(
    osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE));
}

void
DisplayBoundingBoxVisitor::apply(osg::Drawable& drawable)
{
  const osg::Matrix worldSpace = osg::computeLocalToWorld(this->getNodePath());
  osg::BoundingBox  box        = drawable.computeBoundingBox();

  osg::BoundingBox boxWS;
  for (int i = 0; i < 7; ++i) {
    boxWS.expandBy(box.corner(i) * worldSpace);
  }

  {
    // osg::ref_ptr<osg::Box> shape = new osg::Box(boxWS.center(),
    // boxWS.radius());
    osg::ref_ptr<osg::Box> shape =
      new osg::Box(box.center() * worldSpace, boxWS.xMax() - boxWS.xMin(),
                   boxWS.yMax() - boxWS.yMin(), boxWS.zMax() - boxWS.zMin());
    osg::ref_ptr<osg::ShapeDrawable> drawableBox =
      new osg::ShapeDrawable(shape);
    drawableBox->setColor(osg::Vec4(0.0f, 0.0f, 1.0f, 0.75f));
    boxes->addDrawable(drawableBox);
  }

  // Sphere
  {
    osg::BoundingSphere       sphere(boxWS);
    osg::ref_ptr<osg::Sphere> shape =
      new osg::Sphere(sphere.center(), sphere.radius());
    osg::ref_ptr<osg::ShapeDrawable> drawableBox =
      new osg::ShapeDrawable(shape);
    drawableBox->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 0.25f));
    boxes->addChild(drawableBox);
  }
}

/* --- Displays boundingbox --- */

/* --- Explosion with particles --- */
static osg::ref_ptr<osgParticle::ParticleSystem>
CreateExplosionPS(osg::Group& parent)
{
  constexpr float                           scale = 1.0f;
  osg::ref_ptr<osgParticle::ParticleSystem> ps =
    new osgParticle::ParticleSystem;

  ps->setDefaultAttributes("Images/smoke.rgb", false, false);

  float radius  = 0.4f * scale;
  float density = 1.2f; // 1.0kg/m^3

  auto& defaultParticleTemplate = ps->getDefaultParticleTemplate();
  defaultParticleTemplate.setLifeTime(1.0 + 0.1 * scale);
  defaultParticleTemplate.setSizeRange(osgParticle::rangef(0.75f, 3.0f));
  defaultParticleTemplate.setAlphaRange(osgParticle::rangef(0.1f, 1.0f));
  defaultParticleTemplate.setColorRange(osgParticle::rangev4(
    osg::Vec4(1.0f, 0.8f, 0.2f, 1.0f), osg::Vec4(1.0f, 0.4f, 0.1f, 0.0f)));
  defaultParticleTemplate.setRadius(radius);
  defaultParticleTemplate.setMass(density * radius * radius * radius * osg::PI *
                                  4.0f / 3.0f);

  // TODO: Try         _program = new osgParticle::FluidProgram;

  osg::ref_ptr<osgParticle::ModularProgram> program =
    new osgParticle::ModularProgram;
  program->setParticleSystem(ps);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(ps);
  parent.addChild(program);
  parent.addChild(geode);

  // parent->addChild(parent2);

  osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater =
    new osgParticle::ParticleSystemUpdater;
  updater->addParticleSystem(ps);
  parent.addChild(updater);

  // MeshSetLineMode(geometry);

  return ps;
}

osg::ref_ptr<osgParticle::Emitter>
CreateExplosionEmitter()
{
  constexpr float scale = 1.0f;

  osg::ref_ptr<osgParticle::RandomRateCounter> rrc =
    new osgParticle::RandomRateCounter;
  rrc->setRateRange(800, 1000);

  osg::ref_ptr<osgParticle::ModularEmitter> emitter =
    new osgParticle::ModularEmitter;
  // emitter->setParticleSystem(ps);
  emitter->setCounter(rrc);
  emitter->setEndless(false);
  emitter->setLifeTime(.10f);

  osg::ref_ptr<osgParticle::RadialShooter> shooter =
    new osgParticle::RadialShooter;
  osg::ref_ptr<osgParticle::SectorPlacer> placer =
    new osgParticle::SectorPlacer;

  emitter->setPlacer(placer);
  emitter->setShooter(shooter);

  // placer->setCenter(osg::Vec3(0, 0, 60));
  placer->setRadiusRange(0.0f * scale, 0.25f * scale);

  shooter->setThetaRange(0.0f, osg::PI * 2.0f);
  shooter->setInitialSpeedRange(1.0f * scale, 10.0f * scale);

  return emitter;
}

/* --- Explosion with particles --- */

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
  // assert(false && "// TODO: ");
}

void
FollowNodeCamera::setByInverseMatrix(const osg::Matrixd& /*matrix*/)
{
  // assert(false && "// TODO: ");
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
#if ZINC__SHADOWMAP == 1
  osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
#else
  osg::ref_ptr<osgShadow::ViewDependentShadowMap> sm =
    new osgShadow::ViewDependentShadowMap;
// osg::ref_ptr<osgShadow::LightSpacePerspectiveShadowMapVB> sm =
//   new osgShadow::LightSpacePerspectiveShadowMapVB;
// osg::ref_ptr<osgShadow::ParallelSplitShadowMap> sm =
//   new osgShadow::ParallelSplitShadowMap;
#endif

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
  root->addChild(skybox);

  osg::ref_ptr<osg::Node> playerNode =
    osgDB::readNodeFile("../media/CubePlayer.osgt");
  osg::ref_ptr<osg::MatrixTransform> player = new osg::MatrixTransform;
  player->setMatrix(player->getMatrix() *
                    osg::Matrix::scale(osg::Vec3(0.5f, 0.5f, 0.5f)));
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
#if ZINC_USETESTLEVEL
  osg::ref_ptr<osg::Node> floor =
    osgDB::readNodeFile("../media/PlaneFloor.osgt");
  floor->setNodeMask(rcvShadowMask);
  floor->setName("Floor");
  shadowroot->addChild(floor);

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
#elif 1
  osg::ref_ptr<osg::Group> obstacles =
    // osgDB::readNodeFile("../media/Rounded.osgt")->asGroup();
    osgDB::readNodeFile("../media/level1.osgt")->asGroup();
#endif
  obstacles->setName("Obstacle Group");
  obstacles->setNodeMask(rcvShadowMask | castShadowMask);

#if ZINC_LEVEL_SHADOW_BAKED
  root->addChild(obstacles);
#else
  // TODO: Shadow should be baked at this point. In addition, the sadow map is
  // too small to include all the elements. It however render correctly with
  // ViewDependentshadowmap
  shadowroot->addChild(obstacles);
#endif

  osgViewer::Viewer viewer;
  // viewer.setUpViewOnSingleScreen(1);
  // viewer.setUpViewerAsEmbeddedInWindow(0, 0, 1920, 1080);
  // viewer.realize();

  viewer.setLightingMode(osg::View::NO_LIGHT);
  osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> ks =
    new osgGA::KeySwitchMatrixManipulator;
  ks->addMatrixManipulator(osgGA::GUIEventAdapter::KeySymbol::KEY_F11, "Race",
                           cameraman);
  ks->addMatrixManipulator(osgGA::GUIEventAdapter::KeySymbol::KEY_F12, "Free",
                           new osgGA::TrackballManipulator);
  viewer.setCameraManipulator(ks);
  // viewer.setCameraManipulator(cameraman);

  // ribbon->setNodeMask(ribbon->getNodeMask() & ~rcvShadowMask);
  root->addChild(shadowroot);
  viewer.setSceneData(root);

  osg::ref_ptr<osgViewer::StatsHandler> stats = new osgViewer::StatsHandler;
  stats->setKeyEventTogglesOnScreenStats(
    osgGA::GUIEventAdapter::KeySymbol::KEY_F1);
  stats->setKeyEventPrintsOutStats(osgGA::GUIEventAdapter::KeySymbol::KEY_F2);
  viewer.addEventHandler(stats);

  SceneManager::Init(obstacles);
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

#if ZINC_SINGLETHREAD
  osgViewer::ViewerBase::ThreadingModel th =
    osgViewer::ViewerBase::SingleThreaded;
  viewer.setThreadingModel(th);
  viewer.setRunMaxFrameRate(60.0);
#endif

  osg::ref_ptr<DisplayBoundingBoxVisitor> displayBoundingbox =
    new DisplayBoundingBoxVisitor;
  root->addChild(displayBoundingbox->boxes);

  /* --- Explosion with particles --- */
  osg::ref_ptr<osgParticle::ParticleSystem> ps = CreateExplosionPS(*root);
  SceneManager::RegisterParticleSystem(0, ps);
  SceneManager::AddParticleEmitter(0, CreateExplosionEmitter());

  cameraman->MyupdateCamera();
  while (!viewer.done()) {
    viewer.frame();

    // Update the camera after all node where updated
    cameraman->MyupdateCamera();

#if ZINC_DISPLAY_BOUNDINGBOX
    displayBoundingbox->boxes->removeChildren(
      0, displayBoundingbox->boxes->getNumChildren());
    obstacles->accept(*displayBoundingbox);
#endif
  }
  return 0;
}
