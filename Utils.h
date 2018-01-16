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

#ifndef SOLEIL__UTILS_H_
#define SOLEIL__UTILS_H_

#include <osg/Group>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <string>

#include <iostream>

namespace Soleil {

  struct NameVisitor : public osg::NodeVisitor
  {
    std::string name;
    osg::Node*  found = nullptr;

    NameVisitor(osg::NodeVisitor::TraversalMode mode)
      : osg::NodeVisitor(mode)
    {
    }

    void apply(osg::Node& node) override
    {
      std::cout << "==" << node.getName() << "\n";
      if (node.getName() == name)
        found = &node;
      else {
        traverse(node);
      }
    }
  };

  // osg::Node* GetNodeByName(osg::Node& root, const std::string& name)
  // {
  //   if (root.getName() == name) return &root;
  //   NameVisitor v(osg::NodeVisitor::TraversalMode::TRAVERSE_ALL_CHILDREN);
  //   v.name = name;

  //   root.accept(v);
  //   return v.found;
  // }

} // Soleil

template <typename T>
int
Sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

template <typename T, typename U>
T
mix(T x, T y, U a)
{
  return x * (1.0 - a) + y * a;
}

template <typename T>
T
reflect(T incidence, T normal)
{
  // TODO:  osg::componentMultiply
  return incidence -
         osg::componentMultiply(
           normal, (osg::componentMultiply(normal, incidence) * 2.0f));
}

template <typename T>
T
Random(T maxValue)
{
  return static_cast<T>(rand()) / (static_cast<T>(RAND_MAX / maxValue));
}
template <typename T>
T
Random(T minValue, T maxValue)
{
  //
  return static_cast<T>(rand() + minValue) /
         (static_cast<T>(RAND_MAX / maxValue));
}

inline osg::Vec3
normalize(const osg::Vec3& node)
{
  osg::Vec3 normalized = node;
  normalized.normalize();
  return normalized;
}

#endif /* SOLEIL__UTILS_H_ */
