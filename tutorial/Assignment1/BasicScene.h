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
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;

    void set_mesh_data();
    void original_reset();
    void original_simplification();
    void level_up();
    void level_down();

    // Part 2 - In Progress

    void new_reset();
    void initData();
    void Q_matrix_calculation();
    void edges_cost_calculation(int edge);
    void new_simplification();
    bool new_collapse_edge();


private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> bunny, sphere1 ,cube;
    std::shared_ptr<cg3d::Model> autoModel;

    // Global Variables
    igl::min_heap<std::tuple<double, int, int>> Q;

    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
    Eigen::MatrixXd V, C;

    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    Eigen::MatrixXd VN, FN, T;

    int num_collapsed;
    int index;
    int current_available_collapses;
    bool manual_reset_selected;

    // Part 2 - In Progress

    typedef std::set<std::pair<double, int>> PriorityQueue;
    PriorityQueue new_Q;		                            // priority queue - cost for every edge
    std::vector<PriorityQueue::iterator> Qit;
    std::vector <Eigen::Matrix4d> Qmatrix;                  // list of Q matrix for each vertical
};
