#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

 
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)}; // empty material
   // SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/camel_b.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};

    /*sphere1 = Model::Create( "sphere",sphereMesh, material);
    cyl = Model::Create( "cyl", cylMesh, material);
    cube = Model::Create( "cube", cubeMesh, material);*/

    object1 = Model::Create("sphere", sphereMesh, material);
    object2 = Model::Create("sphere", sphereMesh, material);

    object1->showWireframe = true;
    object2->showWireframe = true;

    object1->Translate({ -2, 0, 0 });
    object2->Translate({ 2, 0, 0 });

    //sphere1->Scale(2);
    //sphere1->showWireframe = true;
    //sphere1->Translate({-3,0,0});
    //cyl->Translate({3,0,0});
    //cyl->Scale(0.12f);
    //cyl->showWireframe = true;
    //cube->showWireframe = true;
    camera->Translate(10, Axis::Z);
    //root->AddChild(sphere1);
    //root->AddChild(cyl);
    //root->AddChild(cube);

    root->AddChild(object1);
    root->AddChild(object2);
    
    //auto mesh = cube->GetMeshList();
    //Eigen::VectorXi EMAP;
    //Eigen::MatrixXi F,E,EF,EI;
    //Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    //Eigen::MatrixXd V, C;
    //int num_collapsed;

    // Function to reset original mesh and data structures
    //V = mesh[0]->data[0].vertices;
    //F = mesh[0]->data[0].faces;
    // igl::read_triangle_mesh("data/cube.off",V,F);
    //igl::edge_flaps(F,E,EMAP,EF,EI);
    //std::cout<< "vertices: \n" << V <<std::endl;
    //std::cout<< "faces: \n" << F <<std::endl;
    
    //std::cout<< "edges: \n" << E.transpose() <<std::endl;
    //std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    //std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    //std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

    // Start of new code
    Eigen::MatrixXi F1, F2;
    Eigen::MatrixXd V1, V2;

    auto mesh = object1->GetMeshList();
    V1 = mesh[0]->data[0].vertices;
    F1 = mesh[0]->data[0].faces;
    igl::AABB<Eigen::MatrixXd, 3> object1Tree;
    object1Tree.init(V1, F1);

    mesh = object2->GetMeshList();
    V2 = mesh[0]->data[0].vertices;
    F2 = mesh[0]->data[0].faces;
    igl::AABB<Eigen::MatrixXd, 3> object2Tree;
    object2Tree.init(V2, F2);



    object_velocity_x = 0.001;
    object_velocity_y = 0.0;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);

    object1->Translate({ object_velocity_x, object_velocity_y, 0 });
}

void BasicScene::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            object_velocity_x = 0.0;
            object_velocity_y = 0.0;
            break;
        case GLFW_KEY_UP:
            object_velocity_x = 0.0;
            object_velocity_y = 0.001;
            break;
        case GLFW_KEY_DOWN:
            object_velocity_x = 0.0;
            object_velocity_y = -0.001;
            break;
        case GLFW_KEY_RIGHT:
            object_velocity_x = 0.001;
            object_velocity_y = 0.0;
            break;
        case GLFW_KEY_LEFT:
            object_velocity_x = -0.001;
            object_velocity_y = 0.0;
            break;
        }
    }
}
