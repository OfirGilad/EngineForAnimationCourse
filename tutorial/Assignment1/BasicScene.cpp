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


 #include "AutoMorphingModel.h"

using namespace cg3d;
using namespace std;
using namespace Eigen;
using namespace igl;

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

    // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;

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

    auto morph_function = [](Model* model, cg3d::Visitor* visitor) {
        return (model->GetMeshList())[0]->data.size()-1;
    };
    autoModel = AutoMorphingModel::Create(*sphere1, morph_function);

    //int MAX_COLLAPSES = 10;

    OF = F;
    OV = V;
    int index = 0;
    int current_available_collapses = 1;
    bool manual_reset_selected = false;


    const auto& reset = [&]()
    {
        if (manual_reset_selected)
        {
            manual_reset_selected = false;
            mesh_list.clear();
            mesh_list.push_back({OV, OF});
            current_available_collapses = 1;
        }
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
        index = 0;
    };

    const auto& level_up = [&]()
    {
        index--;
        if (index < 0) {
            index = max(0, current_available_collapses - 1);
        }
        V = get<0>(mesh_list[index]);
        F = get<1>(mesh_list[index]);
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        viewer.data().set_face_based(true);
    };

    const auto& level_down = [&]()
    {
        index++;
        if (index >= current_available_collapses) {
            index = 0;
        }
        V = get<0>(mesh_list[index]);
        F = get<1>(mesh_list[index]);
        viewer.data().clear();
        viewer.data().set_mesh(V, F);
        viewer.data().set_face_based(true);
    };

    const auto& pre_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool
    {
        //if (index == MAX_COLLAPSES) {
        //    reset();
        //    return false;
        //}
        if (index != current_available_collapses - 1) {
            viewer.core().is_animating = false;
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
                current_available_collapses++;
                index++;
                mesh_list.push_back({V, F});   
            }
        }
        return false;
    };

    const auto& key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
    {
        switch (key)
        {
        case ' ':
            viewer.core().is_animating ^= 1;
            break;
        case 'R':
        case 'r':
            manual_reset_selected = true;
            reset();
            break;
        // Go UP
        case 'W':
        case 'w':
            level_up();
            break;
        // Go DOWN
        case 'S':
        case 's':
            level_down();
            break;
        default:
            return false;
        }
        return true;
    };

    mesh_list.push_back({OV, OF});
    reset();
    viewer.core().background_color.setConstant(1);
    viewer.core().is_animating = false;
    viewer.callback_key_down = key_down;
    viewer.callback_pre_draw = pre_draw;
    //viewer.launch();

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

            break;
        // New Code - End
        }
    }
}

//////////////////////

// In progress
void BasicScene::reset() {
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
}

// compute cost and potential placement and place in queue
void BasicScene::edges_cost_calculation(int index, int edge, Eigen::MatrixXd& V)
{
    //vertexes of the edge
    int v1 = E(edge, 0);
    int v2 = E(edge, 1);

    Eigen::Matrix4d Qedge = Qmatrix[index][v1] + Qmatrix[index][v2];

    Eigen::Matrix4d Qposition = Qedge; //we will use this to find v` position
    Qposition.row(3) = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector4d vposition;
    double cost;
    bool isInversable;
    Qposition.computeInverseWithCheck(Qposition, isInversable);
    if (isInversable) {
        vposition = Qposition * (Eigen::Vector4d(0, 0, 0, 1));
        cost = vposition.transpose() * Qedge * vposition;
    }
    else {
        //find min error from v1 v2 v1+v2/2
        Eigen::Vector4d v1p;
        v1p << V.row(v1), 1;;
        double cost1 = v1p.transpose() * Qedge * v1p;

        Eigen::Vector4d v2p;
        v1p << V.row(v2), 1;;
        double cost2 = v2p.transpose() * Qedge * v2p;

        Eigen::Vector4d v12p;
        v1p << ((V.row(v1) + V.row(v2)) / 2), 1;;
        double cost3 = v12p.transpose() * Qedge * v12p;
        if (cost1 < cost2 && cost1 < cost3) {
            vposition = v1p;
            cost = cost1;
        }
        else if (cost2 < cost1 && cost2 < cost3) {
            vposition = v2p;
            cost = cost2;
        }
        else {
            vposition = v12p;
            cost = cost3;
        }
    }
    Eigen::Vector3d pos;
    pos[0] = vposition[0];
    pos[1] = vposition[1];
    pos[2] = vposition[2];
    C.row(edge) = pos;
    Qit[index][edge] = Qx[index].insert(std::pair<double, int>(cost, edge)).first;
}

void BasicScene::simplificationX() {
    int id = data().id;
    Eigen::MatrixXd& V = data().V;  //vertice matrix
    Eigen::MatrixXi& F = data().F; //faces matrix
    bool something_collapsed = false;
    // collapse edge
    const int max_iter = std::ceil(0.05 * Qx[id].size());//collapse 5%
    for (int j = 0; j < max_iter; j++)
    {
        if (!collapse_edgeX(V, F, id)) {
            break;
        }
        something_collapsed = true;
        num_collapsed++;
    }

    if (something_collapsed)
    {
        //data().clear();
        data().set_mesh(V, F);
        data().set_face_based(true);
        data().dirty = 157;
    }
}

bool BasicScene::collapse_edgeX(Eigen::MatrixXd& V, Eigen::MatrixXi& F, int id) {
    PriorityQueue& curr_Q = Qx[id];
    std::vector<PriorityQueue::iterator >& curr_Qit = Qit[id];
    int e1, e2, f1, f2; //be used in the igl collapse_edge function
    if (curr_Q.empty())
    {
        // no edges to collapse
        return false;
    }
    std::pair<double, int> pair = *(curr_Q.begin());
    if (pair.first == std::numeric_limits<double>::infinity())
    {
        // min cost edge is infinite cost
        return false;
    }
    curr_Q.erase(curr_Q.begin()); //delete from the queue
    int e = pair.second; //the lowest cost edge in the queue
    //the 2 vertix of the edge
    int v1 = E.row(e)[0];
    int v2 = E.row(e)[1];

    curr_Qit[e] = curr_Q.end();

    //get the  list of faces around the end point the edge
    std::vector<int> N = igl::circulation(e, true, EMAP, EF, EI);
    std::vector<int> Nd = igl::circulation(e, false, EMAP, EF, EI);
    N.insert(N.begin(), Nd.begin(), Nd.end());

    //collapse the edges
    bool is_collapsed = igl::collapse_edge(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2);
    if (is_collapsed) {


        // Erase the two, other collapsed edges
        curr_Q.erase(curr_Qit[e1]);
        curr_Qit[e1] = curr_Q.end();
        curr_Q.erase(curr_Qit[e2]);
        curr_Qit[e2] = curr_Q.end();

        //update the Q matrix for the 2 veterixes we collapsed 
        Qmatrix[id][v1] = Qmatrix[id][v1] + Qmatrix[id][v2];
        Qmatrix[id][v2] = Qmatrix[id][v1] + Qmatrix[id][v2];

        Eigen::VectorXd newPosition;
        // update local neighbors
        // loop over original face neighbors
        for (auto n : N)
        {
            if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; v++)
                {
                    // get edge id
                    const  int ei = EMAP(v* F.rows() + n);
                    // erase old entry
                    curr_Q.erase(curr_Qit[ei]);
                    // compute cost and potential placement and place in queue
                    edges_cost_calculation(id, ei, V);
                    newPosition = C.row(ei);
                }
            }
        }
        std::cout << "edge " << e << ",cost " << pair.first << ",new position (" << newPosition[0] << ","
            << newPosition[1] << "," << newPosition[2] << ")" << std::endl;
    }
    else
    {
        // reinsert with infinite weight (the provided cost function must **not**
        // have given this un-collapsable edge inf cost already)
        pair.first = std::numeric_limits<double>::infinity();
        curr_Qit[e] = curr_Q.insert(pair).first;
    }
    return is_collapsed;
}