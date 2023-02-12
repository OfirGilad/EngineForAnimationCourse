#pragma once

#include "Scene.h"

#include <utility>

#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>

class Libigl_703_Scene : public cg3d::Scene
{
public:
    explicit Libigl_703_Scene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> camel, sphere1, cube;
};

