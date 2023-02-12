#pragma once

#include "Scene.h"

#include <utility>

#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readOBJ.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

class Libigl_404_Scene : public cg3d::Scene
{
public:
    explicit Libigl_404_Scene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> camel, sphere1, cube;

    typedef
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;

    // W - weights matrix
    // BE - Edges between joints
    // C - joints positions
    // P - parents
    // M - weights per vertex per joint matrix
    // U - new vertices position after skinning
    Eigen::MatrixXd V, W, C, U, M;
    Eigen::MatrixXi F, BE;
    Eigen::VectorXi P;
    std::vector<RotationList > poses; // rotations of joints for animation
    double anim_t = 0.0;
    double anim_t_dir = 0.015;
    bool use_dqs = false;
    bool recompute = true;
};

