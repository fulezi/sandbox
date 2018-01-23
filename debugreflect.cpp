

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

// Obstacle:
#if 0
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
  viewer.setLightingMode(osg::View::NO_LIGHT);
  osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> ks =
    new osgGA::KeySwitchMatrixManipulator;
  ks->addMatrixManipulator(osgGA::GUIEventAdapter::KeySymbol::KEY_F12, "Free",
                           new osgGA::TrackballManipulator);
  viewer.setCameraManipulator(ks);

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

  osgViewer::ViewerBase::ThreadingModel th =
    osgViewer::ViewerBase::SingleThreaded;
  viewer.setThreadingModel(th);
  viewer.setRunMaxFrameRate(60.0);

  osg::ref_ptr<DisplayBoundingBoxVisitor> displayBoundingbox =
    new DisplayBoundingBoxVisitor;
  root->addChild(displayBoundingbox->boxes);

  // Set Axis --------------------------------------------------------
  osg::ref_ptr<osg::Node> taxis = osgDB::readNodeFile("../media/taxis.osgt");
  // Set Direction ---------------------------------------------------
  const osg::Vec3 directionStart(2.0f, 110.0f, 0.0f);
  const osg::Vec3 directionEnd(directionStart.x() + 10.0f, directionStart.y(),
                               directionStart.z());
  osg::ref_ptr<osg::MatrixTransform> DirectionNode(new osg::MatrixTransform);
  DirectionNode->addChild(Soleil::GetNodeByName(*taxis, "Direction"));
  DirectionNode->setMatrix(osg::Matrix::rotate(-osg::PI_2f, 0, 0, 1) *
                           osg::Matrix::translate(directionStart));
  root->addChild(DirectionNode);

  osg::Vec3 collisionNormal;
  float     collisionDistance;
  assert(SceneManager::SegmentCollision(directionStart, directionEnd,
                                        &collisionNormal, &collisionDistance));

  // Set Collision Normal ----------------------------------------------
  osg::ref_ptr<osg::MatrixTransform> NormalNode(new osg::MatrixTransform);
  NormalNode->addChild(Soleil::GetNodeByName(*taxis, "Normal"));
  NormalNode->setMatrix(
    osg::Matrix::rotate(osg::Vec3(0, 1, 0), collisionNormal) *
    osg::Matrix::translate((osg::Vec3(0, 1, 0) * collisionDistance) *
                           DirectionNode->getMatrix()));
  root->addChild(NormalNode);

  // Set Reflect --------------------------------------------------------
  osg::Vec3 normalizedDirection = directionEnd - directionStart;
  normalizedDirection.normalize();

  osg::ref_ptr<osg::MatrixTransform> ReflectNode(new osg::MatrixTransform);
  ReflectNode->addChild(Soleil::GetNodeByName(*taxis, "Reflect"));
  ReflectNode->setMatrix(
    osg::Matrix::rotate(osg::Vec3(0, 1, 0),
                        reflect(normalizedDirection, collisionNormal)) *
    osg::Matrix::translate((osg::Vec3(0, 1, 0) * collisionDistance) *
                           DirectionNode->getMatrix()));
  root->addChild(ReflectNode);

  while (!viewer.done()) {
    viewer.frame();

#if ZINC_DISPLAY_BOUNDINGBOX
    displayBoundingbox->boxes->removeChildren(
      0, displayBoundingbox->boxes->getNumChildren());
    obstacles->accept(*displayBoundingbox);
#endif
  }
  return 0;
}
