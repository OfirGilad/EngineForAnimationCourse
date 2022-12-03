#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include "igl/per_vertex_normals.h"

#include "AutoMorphingModel.h"

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
    object1 = Model::Create("sphere", sphereMesh, material);
    object2 = Model::Create("sphere", sphereMesh, material);

    object1->showWireframe = true;
    object2->showWireframe = true;

    auto morph_function = [](Model* model, cg3d::Visitor* visitor)
    {
        int current_index = model->meshIndex;
        return (model->GetMeshList())[0]->data.size() - 1;
    };
    autoModel1 = AutoMorphingModel::Create(*object1, morph_function);
    root->AddChild(autoModel1);
    autoModel1->Translate({ -1.5, 0, 0 });

    autoModel2 = AutoMorphingModel::Create(*object2, morph_function);
    root->AddChild(autoModel2);
    autoModel2->Translate({ 1.5, 0, 0 });

    Eigen::MatrixXi F1, F2;
    Eigen::MatrixXd V1, V2;

    auto mesh = autoModel1->GetMeshList();
    V1 = mesh[0]->data[0].vertices;
    F1 = mesh[0]->data[0].faces;
    object1Tree.init(V1, F1);

    mesh = autoModel2->GetMeshList();
    V2 = mesh[0]->data[0].vertices;
    F2 = mesh[0]->data[0].faces;
    object2Tree.init(V2, F2);


    /*data().set_mesh(V1, F1);
    DrawObjectBox(object1Tree.m_box);
    V1 = data().V;
    F1 = data().F;
    Eigen::MatrixXd VN, T;
    igl::per_vertex_normals(V1, F1, VN);
    T = Eigen::MatrixXd::Zero(V1.rows(), 2);
    mesh = autoModel1->GetMeshList();
    mesh[0]->data.push_back({ V1, F1, VN, T });
    autoModel1->SetMeshList(mesh);*/

    print_collision_status = true;
    object_velocity_x = 0.001;
    object_velocity_y = 0.0;

    object1_rotation_z = -0.001;
    object2_rotation_z = 0.001;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);

    

    autoModel1->Translate({ object_velocity_x, object_velocity_y, 0 });

    bool collision_check_result = CollisionCheck(&object1Tree, &object2Tree);
    if (collision_check_result) {
        autoModel1->Rotate(0.0, Axis::Z);
        autoModel2->Rotate(0.0, Axis::Z);
        
        object_velocity_x = 0.0;
        object_velocity_y = 0.0;

        object1_rotation_z = 0.0;
        object2_rotation_z = 0.0;

        if (print_collision_status) {
            print_collision_status = false;
            std::cout << "Collision Detected!" << std::endl;
        } 
    }
    else {
        autoModel1->Rotate(object1_rotation_z, Axis::Z);
        autoModel2->Rotate(object2_rotation_z, Axis::Z);
    }
    
}

void BasicScene::KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
        case GLFW_KEY_SPACE:
            autoModel1->Rotate(0.0, Axis::Z);
            autoModel2->Rotate(0.0, Axis::Z);

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
        }
    }
}

void BasicScene::DrawObjectBox(Eigen::AlignedBox<double, 3>& aligned_box, Eigen::RowVector3d color_vector) {
    Eigen::RowVector3d BottomRightCeil = aligned_box.corner(aligned_box.BottomRightCeil);
    Eigen::RowVector3d BottomRightFloor = aligned_box.corner(aligned_box.BottomRightFloor);
    Eigen::RowVector3d BottomLeftCeil = aligned_box.corner(aligned_box.BottomLeftCeil);
    Eigen::RowVector3d BottomLeftFloor = aligned_box.corner(aligned_box.BottomLeftFloor);
    Eigen::RowVector3d TopRightCeil = aligned_box.corner(aligned_box.TopRightCeil);
    Eigen::RowVector3d TopRightFloor = aligned_box.corner(aligned_box.TopRightFloor);
    Eigen::RowVector3d TopLeftCeil = aligned_box.corner(aligned_box.TopLeftCeil);
    Eigen::RowVector3d TopLeftFloor = aligned_box.corner(aligned_box.TopLeftFloor);

    data().add_edges(BottomLeftCeil, BottomRightCeil, color_vector);
    data().add_edges(BottomLeftCeil, BottomLeftFloor, color_vector);
    data().add_edges(BottomRightCeil, BottomRightFloor, color_vector);
    data().add_edges(BottomLeftFloor, BottomRightFloor, color_vector);
    data().add_edges(TopLeftCeil, TopRightCeil, color_vector);
    data().add_edges(TopRightCeil, TopRightFloor, color_vector);
    data().add_edges(TopLeftCeil, TopLeftFloor, color_vector);
    data().add_edges(TopLeftFloor, TopRightFloor, color_vector);
    data().add_edges(TopLeftCeil, BottomLeftCeil, color_vector);
    data().add_edges(TopRightFloor, BottomRightFloor, color_vector);
    data().add_edges(TopRightCeil, BottomRightCeil, color_vector);
    data().add_edges(TopLeftFloor, BottomLeftFloor, color_vector);
}

bool BasicScene::CollisionCheck(igl::AABB<Eigen::MatrixXd, 3>* object_tree1, igl::AABB<Eigen::MatrixXd, 3>* object_tree2) {
    //base cases
    if (object_tree1 == nullptr || object_tree2 == nullptr)
        return false;
    if (!BoxesIntersectionCheck(object_tree1->m_box, object_tree2->m_box)) {
        return false;
    }
    if (object_tree1->is_leaf() && object_tree2->is_leaf()) {
        Eigen::RowVector3d color_vector = Eigen::RowVector3d(255, 0, 0);
        //if the boxes intersect than draw the boxes
        //DrawObjectBox(aligned_box1->m_box, color_vector);
        //DrawObjectBox(aligned_box2->m_box, color_vector);
        return true;

    }
    if (object_tree1->is_leaf() && !object_tree2->is_leaf()) {

        return CollisionCheck(object_tree1, object_tree2->m_right) ||
            CollisionCheck(object_tree1, object_tree2->m_left);
    }
    if (!object_tree1->is_leaf() && object_tree2->is_leaf()) {
        return CollisionCheck(object_tree1->m_right, object_tree2) ||
            CollisionCheck(object_tree1->m_left, object_tree2);
    }

    return CollisionCheck(object_tree1->m_left, object_tree2->m_left) ||
        CollisionCheck(object_tree1->m_left, object_tree2->m_right) ||
        CollisionCheck(object_tree1->m_right, object_tree2->m_left) ||
        CollisionCheck(object_tree1->m_right, object_tree2->m_right);
}

bool BasicScene::BoxesIntersectionCheck(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2) {
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
    //Eigen::Vector4d D4d = data_list[1].MakeTransd().cast<double>() * CenterB - data_list[0].MakeTransd().cast<double>() * CenterA;
    //Eigen::Vector3d D = D4d.head(3);
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