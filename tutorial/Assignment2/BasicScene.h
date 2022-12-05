#pragma once

#include "Scene.h"
#include "igl/AABB.h"

#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;

    void AlignedBoxTransformer(Eigen::AlignedBox<double, 3>& aligned_box, std::shared_ptr<cg3d::Model> cube_model);
    bool CollisionCheck(igl::AABB<Eigen::MatrixXd, 3>* aligned_box1, igl::AABB<Eigen::MatrixXd, 3>* aligned_box2, int level);
    bool BoxesIntersectionCheck(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2);

    // Simplification Support
    void level_up(int object_index);
    void level_down(int object_index);
    void level_reset(int object_index);
    void set_mesh_data(int object_index);
    void new_reset();
    void init_data(int object_index);
    void Q_matrix_calculation(int object_index);
    void edges_cost_calculation(int edge, int object_index);
    void new_simplification(int object_index);
    bool new_collapse_edge(int object_index);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl, sphere1 ,cube;
    std::shared_ptr<cg3d::Model> object1, object2, object1_cube, object2_cube, object1_hit_cube, object2_hit_cube;

    std::shared_ptr<cg3d::Model> autoModel1, autoModel2;
    std::vector<std::shared_ptr<cg3d::Model>> autoModels;

    igl::AABB<Eigen::MatrixXd, 3> object1Tree, object2Tree;
    float object_velocity_x, object_velocity_y;
    float object1_rotation_z, object2_rotation_z;
    bool print_collision_status;

    // Simplification support
    int max_support;
    std::vector<int> num_collapsed;
    std::vector<int> indices;
    std::vector<int> current_available_collapses;

    std::vector<Eigen::VectorXi> EMAP;
    std::vector<Eigen::MatrixXi> F, E, EF, EI;
    std::vector<Eigen::VectorXi> EQ;
    std::vector<Eigen::MatrixXd> V, C;

    std::vector<Eigen::MatrixXi> OF;
    std::vector<Eigen::MatrixXd> OV;
    Eigen::MatrixXd VN, FN, T;

    typedef std::set<std::pair<double, int>> PriorityQueue;
    std::vector<PriorityQueue> new_Q; // priority queue - cost for every edge
    std::vector<std::vector<PriorityQueue::iterator>> Q_iter;
    std::vector<std::vector<Eigen::Matrix4d>> Q_matrix; // list of Q matrix for each vertical
};
