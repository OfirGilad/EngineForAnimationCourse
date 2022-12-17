#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include "igl/per_vertex_normals.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/per_face_normals.h"
#include "igl/circulation.h"
#include "igl/collapse_edge.h"

#include "AutoMorphingModel.h"

using namespace cg3d;
using namespace std;

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
    auto material{ std::make_shared<Material>("material", program)};
    // SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
    auto empty_material{ std::make_shared<Material>("material", program) }; // empty material
    auto hit_material{ std::make_shared<Material>("material", program) }; // hit material

    material->AddTexture(0, "textures/box0.bmp", 2);
    hit_material->AddTexture(0, "textures/grass.bmp", 2);

    // Sphere Meshes
    auto sphereMesh1{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto sphereMesh2{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};

    // Bunny Meshes
    auto bunnylMesh1{IglLoader::MeshFromFiles("cyl_igl","data/bunny.off")};
    auto bunnylMesh2{IglLoader::MeshFromFiles("cyl_igl","data/bunny.off")};

    // Cube Meshes
    auto cubeMesh1{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};
    auto cubeMesh2{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};
    auto cubeMesh3{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};
    auto cubeMesh4{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};

    //sphere1 = Model::Create( "sphere",sphereMesh, material);
    //cyl = Model::Create( "cyl", cylMesh, material);
    //cube = Model::Create( "cube", cubeMesh, material);

    
    //sphere1->Scale(2);
    //sphere1->showWireframe = true;
    //sphere1->Translate({-3,0,0});
    //cyl->Translate({3,0,0});
    //cyl->Scale(0.12f);
    //cyl->showWireframe = true;
    //cube->showWireframe = true;
    //camera->Translate(10, Axis::Z);
    //root->AddChild(sphere1);
    //root->AddChild(cyl);
    //root->AddChild(cube);
    
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
    camera->Translate(10, Axis::Z);

    // Spheres
    //object1 = Model::Create("sphere1", sphereMesh1, material);
    //object2 = Model::Create("sphere2", sphereMesh2, material);
    //object1->showWireframe = true;
    //object2->showWireframe = true;

    // Bunnies
    object1 = Model::Create("bunny1", bunnylMesh1, material);
    object2 = Model::Create("bunny2", bunnylMesh2, material);
    object1->showWireframe = true;
    object2->showWireframe = true;

    // Biggest bounding boxes
    object1_cube = Model::Create("cube1", cubeMesh1, empty_material);
    object2_cube = Model::Create("cube2", cubeMesh2, empty_material);

    // Smallest bounding boxes
    object1_hit_cube = Model::Create("cube3", cubeMesh3, hit_material);
    object2_hit_cube = Model::Create("cube4", cubeMesh4, hit_material);
    
    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() * 0 + current_index;
    };
    autoModel1 = AutoMorphingModel::Create(*object1, morph_function);
    autoModel2 = AutoMorphingModel::Create(*object2, morph_function);

    root->AddChild(autoModel1);
    root->AddChild(autoModel2);

    autoModel1->AddChild(object1_cube);
    autoModel2->AddChild(object2_cube);
    object1_cube->showFaces = false;
    object2_cube->showFaces = false;
    object1_cube->showWireframe = true;
    object2_cube->showWireframe = true;

    // Spheres
    //autoModel1->Translate({ -1.5, 0, 0 });
    //autoModel2->Translate({ 1.5, 0, 0 });

    // Bunnies
    autoModel1->Translate({ -0.3, -0.1, 9 });
    autoModel2->Translate({ 0.3, -0.1, 9 });

    // Initialize object1 tree
    auto mesh = autoModel1->GetMeshList();
    V.push_back(mesh[0]->data[0].vertices);
    F.push_back(mesh[0]->data[0].faces);
    object1Tree.init(V[0], F[0]);

    // Initialize object2 tree
    mesh = autoModel2->GetMeshList();
    V.push_back(mesh[0]->data[0].vertices);
    F.push_back(mesh[0]->data[0].faces);
    object2Tree.init(V[1], F[1]);

    // Creating each object's biggest bounding box 
    AlignedBoxTransformer(object1Tree.m_box, object1_cube);
    AlignedBoxTransformer(object2Tree.m_box, object2_cube);

    print_collision_status = true;
    object_velocity_x = 0.001;
    object_velocity_y = 0.0;
    object1_rotation_z = -0.001;
    object2_rotation_z = 0.001;


    // Simplification Support
    //OV.push_back(V[0]);
    //OV.push_back(V[1]);
    //OF.push_back(F[0]);
    //OF.push_back(F[1]);
    //indices.push_back(0);
    //indices.push_back(0);
    //current_available_collapses.push_back(1);
    //current_available_collapses.push_back(1);
    //autoModels.push_back(autoModel1);
    //autoModels.push_back(autoModel2);
    //new_reset();
    //max_support = 5;
    //for (int i = 0; i < max_support; i++) {
    //    new_simplification(0);
    //    new_simplification(1);
    //}
    //for (int i = 0; i < max_support; i++) {
    //    level_up(0);
    //    level_up(1);
    //}
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);

    autoModel1->Translate({ object_velocity_x, object_velocity_y, 0 });

    // Check if a collision occurred
    if (print_collision_status && CollisionCheck(&object1Tree, &object2Tree, 0)) 
    {
        autoModel1->Rotate(0.0, Axis::Z);
        autoModel2->Rotate(0.0, Axis::Z);
        
        object_velocity_x = 0.0;
        object_velocity_y = 0.0;

        object1_rotation_z = 0.0;
        object2_rotation_z = 0.0;

        print_collision_status = false;
        std::cout << "Collision Detected!" << std::endl;


        // Simplification Support
        //level_reset(0);
        //level_reset(1);
    }
    // No collision occurred
    else 
    {
        autoModel1->Rotate(object1_rotation_z, Axis::Z);
        autoModel2->Rotate(object2_rotation_z, Axis::Z);


        // Simplification Support
        //level_reset(0);
        //level_reset(1);
    }
    
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
            print_collision_status = true;
            object_velocity_x = 0.0;
            object_velocity_y = 0.001;
            object1_rotation_z = 0.0;
            object2_rotation_z = 0.001;
            break;
        case GLFW_KEY_DOWN:
            print_collision_status = true;
            object_velocity_x = 0.0;
            object_velocity_y = -0.001;
            object1_rotation_z = 0.0;
            object2_rotation_z = 0.001;
            break;
        case GLFW_KEY_RIGHT:
            print_collision_status = true;
            object_velocity_x = 0.001;
            object_velocity_y = 0.0;
            object1_rotation_z = -0.001;
            object2_rotation_z = 0.001;
            break;
        case GLFW_KEY_LEFT:
            print_collision_status = true;
            object_velocity_x = -0.001;
            object_velocity_y = 0.0;
            object1_rotation_z = 0.001;
            object2_rotation_z = 0.001;
            break;
        case GLFW_KEY_R:
            new_reset();
            break;
        }
    }
}

void BasicScene::AlignedBoxTransformer(Eigen::AlignedBox<double, 3>& aligned_box, std::shared_ptr<cg3d::Model> cube_model)
{
    // Get the aligned box coordinates
    Eigen::RowVector3d BottomRightCeil = aligned_box.corner(aligned_box.BottomRightCeil);
    Eigen::RowVector3d BottomRightFloor = aligned_box.corner(aligned_box.BottomRightFloor);
    Eigen::RowVector3d BottomLeftCeil = aligned_box.corner(aligned_box.BottomLeftCeil);
    Eigen::RowVector3d BottomLeftFloor = aligned_box.corner(aligned_box.BottomLeftFloor);
    Eigen::RowVector3d TopRightCeil = aligned_box.corner(aligned_box.TopRightCeil);
    Eigen::RowVector3d TopRightFloor = aligned_box.corner(aligned_box.TopRightFloor);
    Eigen::RowVector3d TopLeftCeil = aligned_box.corner(aligned_box.TopLeftCeil);
    Eigen::RowVector3d TopLeftFloor = aligned_box.corner(aligned_box.TopLeftFloor);

    // Update the cube with the aligned box coordinates
    Eigen::MatrixXd V, VN, T;
    Eigen::MatrixXi F;

    V.resize(8, 3);
    F.resize(12, 3);
    V.row(0) = BottomLeftFloor;
    V.row(1) = BottomRightFloor;
    V.row(2) = TopLeftFloor;
    V.row(3) = TopRightFloor;
    V.row(4) = BottomLeftCeil;
    V.row(5) = BottomRightCeil;
    V.row(6) = TopLeftCeil;
    V.row(7) = TopRightCeil;
    F.row(0) = Eigen::Vector3i(0, 1, 3);
    F.row(1) = Eigen::Vector3i(3, 2, 0);
    F.row(2) = Eigen::Vector3i(4, 5, 7);
    F.row(3) = Eigen::Vector3i(7, 6, 4);
    F.row(4) = Eigen::Vector3i(0, 4, 6);
    F.row(5) = Eigen::Vector3i(6, 2, 0);
    F.row(6) = Eigen::Vector3i(5, 7, 3);
    F.row(7) = Eigen::Vector3i(7, 3, 1);
    F.row(8) = Eigen::Vector3i(2, 6, 7);
    F.row(9) = Eigen::Vector3i(7, 3, 2);
    F.row(10) = Eigen::Vector3i(4, 5, 1);
    F.row(11) = Eigen::Vector3i(1, 0, 4);

    igl::per_vertex_normals(V, F, VN);
    T = Eigen::MatrixXd::Zero(V.rows(), 2);

    auto mesh = cube_model->GetMeshList();
    mesh[0]->data.push_back({ V, F, VN, T });
    cube_model->SetMeshList(mesh);
    cube_model->meshIndex += 1;
}

bool BasicScene::CollisionCheck(igl::AABB<Eigen::MatrixXd, 3>* object_tree1, igl::AABB<Eigen::MatrixXd, 3>* object_tree2, int level)
{
    // Base cases
    if (object_tree1 == nullptr || object_tree2 == nullptr)
    {
        return false;
    }
    if (!BoxesIntersectionCheck(object_tree1->m_box, object_tree2->m_box)) 
    {
        return false;
    }


    // Simplification Support
    //if ((level > autoModels[0]->meshIndex) && (level <= max_support)) {
    //    level_down(0);
    //    level_down(1);
    //}

    // If the boxes intersect than creating each object's smallest bounding box
    if (object_tree1->is_leaf() && object_tree2->is_leaf()) {
        AlignedBoxTransformer(object_tree1->m_box, object1_hit_cube);
        AlignedBoxTransformer(object_tree2->m_box, object2_hit_cube);
        autoModel1->AddChild(object1_hit_cube);
        autoModel2->AddChild(object2_hit_cube);
        object1_hit_cube->showFaces = false;
        object2_hit_cube->showFaces = false;
        object1_hit_cube->showWireframe = true;
        object2_hit_cube->showWireframe = true;
        return true;
    }
    if (object_tree1->is_leaf() && !object_tree2->is_leaf()) {
        return CollisionCheck(object_tree1, object_tree2->m_right, level + 1) ||
            CollisionCheck(object_tree1, object_tree2->m_left, level + 1);
    }
    if (!object_tree1->is_leaf() && object_tree2->is_leaf()) {
        return CollisionCheck(object_tree1->m_right, object_tree2, level + 1) ||
            CollisionCheck(object_tree1->m_left, object_tree2, level + 1);
    }
    return CollisionCheck(object_tree1->m_left, object_tree2->m_left, level + 1) ||
        CollisionCheck(object_tree1->m_left, object_tree2->m_right, level + 1) ||
        CollisionCheck(object_tree1->m_right, object_tree2->m_left, level + 1) ||
        CollisionCheck(object_tree1->m_right, object_tree2->m_right, level + 1);
}

bool BasicScene::BoxesIntersectionCheck(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2)
{
    // Matrix A
    Eigen::Matrix3d A = autoModel1->GetRotation().cast<double>();
    Eigen::Vector3d A0 = A.col(0);
    Eigen::Vector3d A1 = A.col(1);
    Eigen::Vector3d A2 = A.col(2);

    // Matrix B
    Eigen::Matrix3d B = autoModel2->GetRotation().cast<double>();
    Eigen::Vector3d B0 = B.col(0);
    Eigen::Vector3d B1 = B.col(1);
    Eigen::Vector3d B2 = B.col(2);

    // Matrix C (Where: C=A^T*B)
    Eigen::Matrix3d C = A.transpose() * B;
    // Get the lengths of the sides of the bounding box
    Eigen::Vector3d a = aligned_box1.sizes();
    Eigen::Vector3d b = aligned_box2.sizes();
    a = a / 2;
    b = b / 2;

    // Matrix D
    Eigen::Vector4d CenterA = Eigen::Vector4d(aligned_box1.center()[0], aligned_box1.center()[1], aligned_box1.center()[2], 1);
    Eigen::Vector4d CenterB = Eigen::Vector4d(aligned_box2.center()[0], aligned_box2.center()[1], aligned_box2.center()[2], 1);
    Eigen::Vector4d D4d = autoModel2->GetTransform().cast<double>() * CenterB - autoModel1->GetTransform().cast<double>() * CenterA;
    Eigen::Vector3d D = D4d.head(3);

    float R0, R1, R;

    // Check the 15 conditions
    // Check A conditions
    // A0
    R0 = a(0);
    R1 = b(0) * abs(C.row(0)(0)) + b(1) * abs(C.row(0)(1)) + b(2) * abs(C.row(0)(2));
    R = abs(A0.transpose() * D);
    if (R0 + R1 < R) return false;
    // A1
    R0 = a(1);
    R1 = b(0) * abs(C.row(1)(0)) + b(1) * abs(C.row(1)(1)) + b(2) * abs(C.row(1)(2));
    R = abs(A1.transpose() * D);
    if (R0 + R1 < R) return false;
    // A2
    R0 = a(2);
    R1 = b(0) * abs(C.row(2)(0)) + b(1) * abs(C.row(2)(1)) + b(2) * abs(C.row(2)(2));
    R = abs(A2.transpose() * D);
    if (R0 + R1 < R) return false;
    
    // Check B conditions
    // B0
    R0 = a(0) * abs(C.row(0)(0)) + a(1) * abs(C.row(1)(0)) + a(2) * abs(C.row(2)(0));
    R1 = b(0);
    R = abs(B0.transpose() * D);
    if (R0 + R1 < R) return false;
    // B1
    R0 = a(0) * abs(C.row(0)(1)) + a(1) * abs(C.row(1)(1)) + a(2) * abs(C.row(2)(1));
    R1 = b(1);
    R = abs(B1.transpose() * D);
    if (R0 + R1 < R) return false;
    // B2
    R0 = a(0) * abs(C.row(0)(2)) + a(1) * abs(C.row(1)(2)) + a(2) * abs(C.row(2)(2));
    R1 = b(2);
    R = abs(B2.transpose() * D);
    if (R0 + R1 < R) return false;

    // Check A0 conditions
    // A0 X B0
    R0 = a(1) * abs(C.row(2)(0)) + a(2) * abs(C.row(1)(0));
    R1 = b(1) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(1));
    R = C.row(1)(0) * A2.transpose() * D;
    R -= C.row(2)(0) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A0 X B1
    R0 = a(1) * abs(C.row(2)(1)) + a(2) * abs(C.row(1)(1));
    R1 = b(0) * abs(C.row(0)(2)) + b(2) * abs(C.row(0)(0));
    R = C.row(1)(1) * A2.transpose() * D;
    R -= C.row(2)(1) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A0 X B2
    R0 = a(1) * abs(C.row(2)(2)) + a(2) * abs(C.row(1)(2));
    R1 = b(0) * abs(C.row(0)(1)) + b(1) * abs(C.row(0)(0));
    R = C.row(1)(2) * A2.transpose() * D;
    R -= C.row(2)(2) * A1.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    
    // Check A1 conditions
    // A1 X B0
    R0 = a(0) * abs(C.row(2)(0)) + a(2) * abs(C.row(0)(0));
    R1 = b(1) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(1));
    R = C.row(2)(0) * A0.transpose() * D;
    R -= C.row(0)(0) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A1 X B1
    R0 = a(0) * abs(C.row(2)(1)) + a(2) * abs(C.row(0)(1));
    R1 = b(0) * abs(C.row(1)(2)) + b(2) * abs(C.row(1)(0));
    R = C.row(2)(1) * A0.transpose() * D;
    R -= C.row(0)(1) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A1 X B2
    R0 = a(0) * abs(C.row(2)(2)) + a(2) * abs(C.row(0)(2));
    R1 = b(0) * abs(C.row(1)(1)) + b(1) * abs(C.row(1)(0));
    R = C.row(2)(2) * A0.transpose() * D;
    R -= C.row(0)(2) * A2.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;

    // Check A2 conditions
    // A2 X B0
    R0 = a(0) * abs(C.row(1)(0)) + a(1) * abs(C.row(0)(0));
    R1 = b(1) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(1));
    R = C.row(0)(0) * A1.transpose() * D;
    R -= C.row(1)(0) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A2 X B1
    R0 = a(0) * abs(C.row(1)(1)) + a(1) * abs(C.row(0)(1));
    R1 = b(0) * abs(C.row(2)(2)) + b(2) * abs(C.row(2)(0));
    R = C.row(0)(1) * A1.transpose() * D;
    R -= C.row(1)(1) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;
    // A2 X B2
    R0 = a(0) * abs(C.row(1)(2)) + a(1) * abs(C.row(0)(2));
    R1 = b(0) * abs(C.row(2)(1)) + b(1) * abs(C.row(2)(0));
    R = C.row(0)(2) * A1.transpose() * D;
    R -= C.row(1)(2) * A0.transpose() * D;
    R = abs(R);
    if (R0 + R1 < R) return false;

    // All the conditions are met
    return true;
}


// Simplification Support
void BasicScene::level_up(int object_index)
{
    indices[object_index]--;
    if (indices[object_index] < 0)
    {
        indices[object_index] = max(0, current_available_collapses[object_index] - 1);
    }
    autoModels[object_index]->meshIndex = indices[object_index];
}

void BasicScene::level_down(int object_index)
{
    indices[object_index]++;
    if (indices[object_index] >= current_available_collapses[object_index])
    {
        indices[object_index] = 0;
    }
    autoModels[object_index]->meshIndex = indices[object_index];
}

void BasicScene::level_reset(int object_index)
{
    indices[object_index] = 0;
    autoModels[object_index]->meshIndex = indices[object_index];
}

void BasicScene::set_mesh_data(int object_index)
{
    igl::per_vertex_normals(V[object_index], F[object_index], VN);
    T = Eigen::MatrixXd::Zero(V[object_index].rows(), 2);
    auto mesh = autoModels[object_index]->GetMeshList();
    mesh[0]->data.push_back({ V[object_index], F[object_index], VN, T });
    autoModels[object_index]->SetMeshList(mesh);
    autoModels[object_index]->meshIndex = indices[object_index];
}

void BasicScene::new_reset()
{
    int size = 2;
    E.resize(size);
    EMAP.resize(size);
    EF.resize(size);
    new_Q.resize(size);
    EI.resize(size);
    C.resize(size);
    Q_iter.resize(size);
    Q_matrix.resize(size);
    num_collapsed.resize(size);

    for (int object_index = 0; object_index < size; object_index++) {
        auto mesh = autoModels[object_index]->GetMeshList();
        for (int i = 1; i < current_available_collapses[object_index]; i++)
        {
            mesh[0]->data.pop_back();
        }
        autoModels[object_index]->SetMeshList(mesh);
        current_available_collapses[object_index] = 1;

        V[object_index] = OV[object_index];
        F[object_index] = OF[object_index];
        init_data(object_index);
        indices[object_index] = 0;
        autoModels[object_index]->meshIndex = indices[object_index];
    }
}

void BasicScene::init_data(int object_index)
{
    igl::edge_flaps(F[object_index], E[object_index], EMAP[object_index], EF[object_index], EI[object_index]); // Init data_structures
    C[object_index].resize(E[object_index].rows(), V[object_index].cols());
    Q_iter[object_index].resize(E[object_index].rows()); // Number of edges 
    Q_matrix_calculation(object_index);
    new_Q[object_index].clear();
    num_collapsed[object_index] = 0;

    // Caculate egdes cost
    for (int i = 0; i < E[object_index].rows(); i++)
    {
        edges_cost_calculation(i, object_index);
    }
}

void BasicScene::Q_matrix_calculation(int object_index)
{
    std::vector<std::vector<int>> VF;  // Vertex to faces
    std::vector<std::vector<int>> VFi; // Not in use
    int n = V[object_index].rows();
    Q_matrix[object_index].resize(n);
    igl::vertex_triangle_adjacency(n, F[object_index], VF, VFi);
    igl::per_face_normals(V[object_index], F[object_index], FN);

    for (int i = 0; i < n; i++)
    {
        // Initialize 
        Q_matrix[object_index][i] = Eigen::Matrix4d::Zero();

        // Caculate vertex Q matrix 
        for (int j = 0; j < VF[i].size(); j++)
        {
            // Get face normal
            Eigen::Vector3d normal = FN.row(VF[i][j]).normalized();

            // The equation is: ax+by+cz+d=0
            double a = normal[0];
            double b = normal[1];
            double c = normal[2];
            double d = V[object_index].row(i) * normal;
            d *= -1;

            // Kp = pp^T (s.t. p in planes)
            Eigen::Matrix4d Kp;
            Kp.row(0) = Eigen::Vector4d(a * a, a * b, a * c, a * d);
            Kp.row(1) = Eigen::Vector4d(a * b, b * b, b * c, b * d);
            Kp.row(2) = Eigen::Vector4d(a * c, b * c, c * c, c * d);
            Kp.row(3) = Eigen::Vector4d(a * d, b * d, c * d, d * d);
            Q_matrix[object_index][i] += Kp;
        }
    }
}

void BasicScene::edges_cost_calculation(int edge, int object_index)
{
    // Vertexes of the edge
    int v1 = E[object_index](edge, 0);
    int v2 = E[object_index](edge, 1);
    Eigen::Matrix4d Q_edge = Q_matrix[object_index][v1] + Q_matrix[object_index][v2];

    // We will use this to find v' position
    Eigen::Matrix4d Q_position = Q_edge;
    Q_position.row(3) = Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector4d v_position;
    double cost;
    bool isInversable;
    Q_position.computeInverseWithCheck(Q_position, isInversable);

    if (isInversable)
    {
        v_position = Q_position * (Eigen::Vector4d(0, 0, 0, 1));
        cost = v_position.transpose() * Q_edge * v_position;
    }
    else
    {
        // Find min error from v1, v2, (v1+v2)/2
        Eigen::Vector4d v1_position;
        v1_position << V[object_index].row(v1), 1;
        double cost1 = v1_position.transpose() * Q_edge * v1_position;

        Eigen::Vector4d v2_position;
        v2_position << V[object_index].row(v2), 1;
        double cost2 = v2_position.transpose() * Q_edge * v2_position;

        Eigen::Vector4d v1v2_position;
        v1v2_position << ((V[object_index].row(v1) + V[object_index].row(v2)) / 2), 1;
        double cost3 = v1v2_position.transpose() * Q_edge * v1v2_position;

        if (cost1 < cost2 && cost1 < cost3)
        {
            v_position = v1_position;
            cost = cost1;
        }
        else if (cost2 < cost1 && cost2 < cost3)
        {
            v_position = v2_position;
            cost = cost2;
        }
        else {
            v_position = v1v2_position;
            cost = cost3;
        }
    }
    Eigen::Vector3d new_position;
    new_position[0] = v_position[0];
    new_position[1] = v_position[1];
    new_position[2] = v_position[2];
    C[object_index].row(edge) = new_position;
    Q_iter[object_index][edge] = new_Q[object_index].insert(std::pair<double, int>(cost, edge)).first;
}

void BasicScene::new_simplification(int object_index)
{
    // If it isn't the last collapsed mesh, do nothing
    if (indices[object_index] != current_available_collapses[object_index] - 1)
    {
        return;
    }
    bool something_collapsed = false;

    // Collapse 10% of edges
    const int max_iter = std::ceil(0.1 * new_Q[object_index].size());
    for (int i = 0; i < max_iter; i++)
    {
        if (!new_collapse_edge(object_index))
        {
            break;
        }
        something_collapsed = true;
        num_collapsed[object_index]++;
    }
    if (something_collapsed)
    {
        current_available_collapses[object_index]++;
        indices[object_index]++;
        set_mesh_data(object_index);
    }
}

bool BasicScene::new_collapse_edge(int object_index)
{
    PriorityQueue& curr_Q = new_Q[object_index];
    std::vector<PriorityQueue::iterator>& curr_Q_iter = Q_iter[object_index];
    int e1, e2, f1, f2; // Will be used in the igl collapse_edge function
    if (curr_Q.empty())
    {
        // No edges to collapse
        return false;
    }
    std::pair<double, int> pair = *(curr_Q.begin());
    if (pair.first == std::numeric_limits<double>::infinity())
    {
        // Min cost edge is infinite cost
        return false;
    }
    curr_Q.erase(curr_Q.begin()); // Delete from the queue
    int e = pair.second; // The lowest cost edge in the queue

    // The 2 vertices of the edge
    int v1 = E[object_index].row(e)[0];
    int v2 = E[object_index].row(e)[1];
    curr_Q_iter[e] = curr_Q.end();

    // Get the list of faces around the end point the edge
    std::vector<int> N = igl::circulation(e, true, EMAP[object_index], EF[object_index], EI[object_index]);
    std::vector<int> Nd = igl::circulation(e, false, EMAP[object_index], EF[object_index], EI[object_index]);
    N.insert(N.begin(), Nd.begin(), Nd.end());

    // Collapse the edage
    bool is_collapsed = igl::collapse_edge(e, C[object_index].row(e), V[object_index], F[object_index],  E[object_index], EMAP[object_index], EF[object_index], EI[object_index], e1, e2, f1, f2);
    if (is_collapsed)
    {
        // Erase the two, other collapsed edges
        curr_Q.erase(curr_Q_iter[e1]);
        curr_Q_iter[e1] = curr_Q.end();
        curr_Q.erase(curr_Q_iter[e2]);
        curr_Q_iter[e2] = curr_Q.end();

        // Update the Q matrix for the 2 veterixes we collapsed 
        Q_matrix[object_index][v1] = Q_matrix[object_index][v1] + Q_matrix[object_index][v2];
        Q_matrix[object_index][v2] = Q_matrix[object_index][v1] + Q_matrix[object_index][v2];
        Eigen::VectorXd new_position;

        // Update local neighbors
        // Loop over original face neighbors
        for (auto n : N)
        {
            if (F[object_index](n, 0) != IGL_COLLAPSE_EDGE_NULL ||
                F[object_index](n, 1) != IGL_COLLAPSE_EDGE_NULL ||
                F[object_index](n, 2) != IGL_COLLAPSE_EDGE_NULL)
            {
                for (int v = 0; v < 3; v++)
                {
                    // Get edge id
                    const int ei = EMAP[object_index](v * F[object_index].rows() + n);
                    // Erase old entry
                    curr_Q.erase(curr_Q_iter[ei]);
                    // Compute cost and potential placement and place in queue
                    edges_cost_calculation(ei, object_index);
                    new_position = C[object_index].row(ei);
                }
            }
        }
        //cout << "Edge: " << e
        //    << ", Cost: " << pair.first
        //    << ", New Position: (" << new_position[0] << "," << new_position[1] << "," << new_position[2] << ")"
        //    << std::endl;
    }
    else
    {
        // Reinsert with infinite weight (the provided cost function must **not**
        // have given this un-collapsable edge inf cost already)
        pair.first = std::numeric_limits<double>::infinity();
        curr_Q_iter[e] = curr_Q.insert(pair).first;
    }
    return is_collapsed;
}