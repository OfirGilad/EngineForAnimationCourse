#pragma once

#include "Scene.h"

#include <utility>
#include <vector>

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

using namespace cg3d;
using namespace std;
using namespace Eigen;
using namespace igl;

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;


    // In Progress
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void reset();
    void edges_cost_calculation(int index, int edge, Eigen::MatrixXd& V);
    void simplificationX();
    bool collapse_edgeX(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int id);


private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> camel, sphere1 ,cube;
    std::shared_ptr<cg3d::Model> autoModel;

    // Global Variables
    igl::opengl::glfw::Viewer viewer;
    igl::min_heap< std::tuple<double, int, int> > Q;
    vector<tuple<Eigen::MatrixXd, Eigen::MatrixXi>> mesh_list;

    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C;

    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    int num_collapsed;


    // In Progress
    typedef std::set<std::pair<double, int> > PriorityQueue;
    std::vector < PriorityQueue> Qx;
    std::vector < std::vector<PriorityQueue::iterator > > Qit;
    std::vector < std::vector <Eigen::Matrix4d> > Qmatrix;
};
