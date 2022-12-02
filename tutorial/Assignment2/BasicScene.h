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

    void DrawObjectBox(Eigen::AlignedBox<double, 3>& aligned_box);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cyl, sphere1 ,cube;
    std::shared_ptr<cg3d::Model> object1, object2;

    std::shared_ptr<cg3d::Model> autoModel1, autoModel2;
    igl::AABB<Eigen::MatrixXd, 3> object1Tree, object2Tree;
    float object_velocity_x, object_velocity_y;
};
