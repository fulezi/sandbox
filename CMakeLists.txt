cmake_minimum_required (VERSION 3.5)

project(zinc-osg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -W -Wall -Wextra")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# set(OSG_DIR /home/florian/jeux/OpenSceneGraph/build/INST)
find_package(OpenSceneGraph REQUIRED osgDB osgUtil osgViewer osgGA osgShadow)



include_directories(
  ${OPENSCENEGRAPH_INCLUDE_DIRS}
  )

add_executable(zinc-osg
  main.cpp
  easing.c
  SkyBox.cpp
  Logger.cpp
  SceneManager.cpp

  Ribbon.cpp
  gameplay.cpp)


target_link_libraries(zinc-osg
  ${OPENSCENEGRAPH_LIBRARIES}
  )

