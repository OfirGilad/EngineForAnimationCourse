#pragma once

#include "Scene.h"

#include <utility>
#include <vector>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void EdgesCostCalculation(auto mesh);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> camel, sphere1 ,cube;

    // Global Variables
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C;

    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;
    int num_collapsed;
};
