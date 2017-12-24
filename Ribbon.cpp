
#include <cassert>

#include <osg/MatrixTransform>
#include <osg/Node>

#include <iostream> // TODO: temp

#include "Ribbon.h"

namespace Soleil {

  Ribbon::Ribbon(int numberOfPoints, float width, const osg::Vec3& color)
    : numberOfPoints(numberOfPoints)
    , halfWidth(width / 2.0f)
    , color(color)
    , vertices(new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX,
                                  numberOfPoints))
    , normals(new osg::Vec3Array(osg::Array::Binding::BIND_PER_VERTEX,
                                 numberOfPoints))
    , colors(new osg::Vec4Array(osg::Array::Binding::BIND_PER_VERTEX,
                                numberOfPoints))
  {
    const osg::Vec3 origin(0.0f, 0.0f, 0.0f);
    const osg::Vec3 normal(0.0f, 0.0f, 1.0f);

    assert(numberOfPoints > 0 && "Cannot have less than one point");

    for (int i = 0; i < numberOfPoints - 1; i += 2) {
      (*vertices)[i]     = origin;
      (*vertices)[i + 1] = origin;

      (*normals)[i]     = normal;
      (*normals)[i + 1] = normal;

      // TODO: Use easing function is necessary:
      const float alpha =
        1.0f - (numberOfPoints - i) / static_cast<float>(numberOfPoints);
      (*colors)[i]     = osg::Vec4(color, alpha);
      (*colors)[i + 1] = osg::Vec4(color, alpha);
    }

    // TODO: use double-buffering
    setDataVariance(osg::Object::DataVariance::DYNAMIC);
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    setVertexArray(vertices);
    setNormalArray(normals);
    setColorArray(colors);
    addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, numberOfPoints));
  }

  Ribbon::~Ribbon() {}

  void Ribbon::updateTail(const osg::Matrix& modelMatrix)
  {
    for (int i = 0; i < numberOfPoints - 3; i += 2) {
      (*vertices)[i]     = (*vertices)[i + 2];
      (*vertices)[i + 1] = (*vertices)[i + 3];
      (*normals)[i]      = (*normals)[i + 2];
      (*normals)[i + 1]  = (*normals)[i + 3];
    }
    (*vertices)[numberOfPoints - 2] =
      osg::Vec3(0.0f, -halfWidth, 1.0f) * modelMatrix;
    (*vertices)[numberOfPoints - 1] =
      osg::Vec3(0.0f, halfWidth, 1.0f) *
      modelMatrix; // TODO: Change 1.0f customizable
    vertices->dirty();

    osg::Vec3 normal = osg::Vec3(0.0f, 0.0f, 1.0f) * modelMatrix;
    normal.normalize();
    (*normals)[numberOfPoints - 2] = normal;
    (*normals)[numberOfPoints - 1] = normal;
    normals->dirty();

    dirtyBound();
  }

  RibbonCallback::RibbonCallback(Ribbon* ribbon)
    : ribbon(ribbon)
  {
  }

  void RibbonCallback::operator()(osg::Node* node, osg::NodeVisitor* visitor)
  {
    const osg::MatrixTransform* nodeTransform =
      static_cast<osg::MatrixTransform*>(node);
    // TODO: Avoid cast?
    if (nodeTransform && ribbon.valid()) {
      const osg::Matrix matrix = nodeTransform->getMatrix();

      ribbon->updateTail(matrix);
    }

    traverse(node, visitor);
  }

} // Soleil
