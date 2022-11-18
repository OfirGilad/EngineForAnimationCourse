#include "BasicScene.h"
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

#include <vector>


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
    auto camelMesh{IglLoader::MeshFromFiles("camel_igl","data/camel_b.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};
    
    sphere1 = Model::Create( "sphere",sphereMesh, material);
    camel = Model::Create( "camel", camelMesh, material);
    cube = Model::Create( "cube", cubeMesh, material);
    sphere1->Scale(2);
    sphere1->showWireframe = true;
    sphere1->Translate({-3,0,0});
    camel->Translate({5,0,0});
    camel->Scale(0.12f);
    camel->showWireframe = true;
    cube->showWireframe = true;
    camera->Translate(30, Axis::Z);
    root->AddChild(sphere1);
    root->AddChild(camel);
    root->AddChild(cube);
    
    auto mesh = sphere1->GetMeshList();
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F,E,EF,EI;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C;
    int num_collapsed;

    Eigen::MatrixXi OF;
    Eigen::MatrixXd OV;

    // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;

    OF = F;
    OV = V;

    // igl::read_triangle_mesh("data/cube.off",V,F);
    igl::edge_flaps(F,E,EMAP,EF,EI);
    //std::cout<< "vertices: \n" << V <<std::endl;
    //std::cout<< "faces: \n" << F <<std::endl;
    //
    //std::cout<< "edges: \n" << E.transpose() <<std::endl;
    //std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    //std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    //std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

    // New Code - Start

    using namespace std;
    using namespace Eigen;
    using namespace igl;

    igl::opengl::glfw::Viewer viewer;
    igl::min_heap< std::tuple<double, int, int> > Q;

    //int index = 0;
    //vector<tuple<Eigen::MatrixXd, Eigen::MatrixXi>> mesh_list;
    

    const auto& reset = [&]()
    {
        F = OF;
        V = OV;
        edge_flaps(F, E, EMAP, EF, EI);
        C.resize(E.rows(), V.cols());
        VectorXd costs(E.rows());
        // https://stackoverflow.com/questions/2852140/priority-queue-clear-method
        // Q.clear();
        Q = {};
        EQ = Eigen::VectorXi::Zero(E.rows());
        {
            Eigen::VectorXd costs(E.rows());
            igl::parallel_for(E.rows(), [&](const int e)
            {
                double cost = e;
                RowVectorXd p(1, 3);
                shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
                C.row(e) = p;
                costs(e) = cost;
            }, 10000);
            for (int e = 0;e < E.rows();e++)
            {
                Q.emplace(costs(e), e, 0);
            }
        }

        num_collapsed = 0;
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        viewer.data().set_face_based(true);
        counter = 0;
    };

    const auto& pre_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool
    {
        //tuple<Eigen::MatrixXd, Eigen::MatrixXi> prev_values = { V, F };
        //mesh_list.push_back(prev_values);

        if (counter == 10) {
            reset();
            return false;
        }

        // If animating then collapse 10% of edges
        if (viewer.core().is_animating && !Q.empty())
        {
            bool something_collapsed = false;
            // collapse edge
            const int max_iter = std::ceil(0.01 * Q.size());
            for (int j = 0;j < max_iter;j++)
            {
                if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
                {
                    break;
                }
                something_collapsed = true;
                num_collapsed++;
            }

            if (something_collapsed)
            {
                viewer.data().clear();
                viewer.data().set_mesh(V, F);
                viewer.data().set_face_based(true);
                viewer.core().is_animating = false;
                counter += 1;
            }
        }
        return false;
    };

    /*const auto& back_draw = [&]()
    {
        index -= 1;
        viewer.data().clear();
        viewer.data().set_mesh(get<0>(mesh_list[index]), get<1>(mesh_list[index]));
        viewer.data().set_face_based(true);
        viewer.core().is_animating = false;

        return false;
    };*/

    const auto& key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
    {
        switch (key)
        {
        case ' ':
            viewer.core().is_animating ^= 1;
            break;
        case 'R':
        case 'r':
            reset();
            break;
        default:
            return false;
        }
        return true;
    };

    reset();
    viewer.core().background_color.setConstant(1);
    viewer.core().is_animating = false;
    viewer.callback_key_down = key_down;
    viewer.callback_pre_draw = pre_draw;
    viewer.launch();

    // New Code - End
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    // cube->Rotate(0.01f, Axis::All);
    sphere1->Rotate(0.005f, Axis::Y);
    cube->Rotate(0.005f, Axis::Y);
    camel->Rotate(0.005f, Axis::Y);
}

void BasicScene::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        igl::opengl::glfw::Viewer viewer;

        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        // New Code - Start
        case GLFW_KEY_SPACE:
            viewer.core().is_animating = true;
            break;
        // New Code - End
        }
    }
}
