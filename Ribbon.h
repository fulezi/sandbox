
#ifndef SOLEIL__RIBBON_H_
#define SOLEIL__RIBBON_H_

#include <osg/Geometry>
#include <osg/Vec3>

namespace Soleil {

  class Ribbon : public osg::Geometry
  {
  public:
    Ribbon(int numberOfPoints, float width, const osg::Vec3& color);

  protected:
    ~Ribbon();
    
  public:
    void updateTail(const osg::Matrix& modelMatrix);
    
  private:
    int       numberOfPoints;
    float     halfWidth;
    osg::Vec3 color;

  private:
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec3Array> normals;
    osg::ref_ptr<osg::Vec4Array> colors;
    // TODO: Use the one in geometry
  };

  class RibbonCallback : public osg::NodeCallback
  {
  public:
    RibbonCallback(Ribbon* ribbon);
    void operator()(osg::Node* node, osg::NodeVisitor* visitor);

  private:
    osg::observer_ptr<Ribbon> ribbon;
  };

} // Soleil

#endif /* SOLEIL__RIBBON_H_ */
