#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"

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

 
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    
    auto material{ std::make_shared<Material>("material", program)}; // empty material
    auto material1{ std::make_shared<Material>("material", program1)}; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj")};
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj")};
    sphere1 = Model::Create( "sphere",sphereMesh, material);    
    cube = Model::Create( "cube", cubeMesh, material);
    
    //Axis
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);
    axis.push_back(Model::Create("axis",coordsys,material1));
    axis[0]->mode = 1;   
    axis[0]->Scale(4,Axis::XYZ);
    // axis[0]->lineWidth = 5;
    root->AddChild(axis[0]);
    float scaleFactor = 1; 
    cyls.push_back( Model::Create("cyl",cylMesh, material));
    cyls[0]->Scale(scaleFactor,Axis::X);
    cyls[0]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
    cyls[0]->RotateByDegree(90, Eigen::Vector3f(0,1,0));
    root->AddChild(cyls[0]);
   
    for(int i = 1; i < 3; i++)
    { 
        cyls.push_back( Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor,Axis::X);   
        cyls[i]->Translate(1.6f*scaleFactor,Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        cyls[i-1]->AddChild(cyls[i]);

        //Axis
        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(4, Axis::XYZ);
        cyls[i-1]->AddChild(axis[i]);
        axis[i]->Translate(0.8f* scaleFactor,Axis::Z);
    }
    cyls[0]->Translate({0,0,0.8f*scaleFactor});

    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
      return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };
    autoCube = AutoMorphingModel::Create(*cube, morphFunc);

  
    sphere1->showWireframe = true;
    autoCube->Translate({-6,0,0});
    autoCube->Scale(1.5f);
    sphere1->Translate({5,0,0});

    autoCube->showWireframe = true;
    camera->Translate(22, Axis::Z);
    root->AddChild(sphere1);
//    root->AddChild(cyl);
    root->AddChild(autoCube);
    // points = Eigen::MatrixXd::Ones(1,3);
    // edges = Eigen::MatrixXd::Ones(1,3);
    // colors = Eigen::MatrixXd::Ones(1,3);
    
    // cyl->AddOverlay({points,edges,colors},true);
    cube->mode =1   ; 
    auto mesh = cube->GetMeshList();

    //autoCube->AddOverlay(points,edges,colors);
    // mesh[0]->data.push_back({V,F,V,E});
    int num_collapsed;

  // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
   // igl::read_triangle_mesh("data/cube.off",V,F);
    igl::edge_flaps(F,E,EMAP,EF,EI);
    std::cout<< "vertices: \n" << V <<std::endl;
    std::cout<< "faces: \n" << F <<std::endl;
    
    std::cout<< "edges: \n" << E.transpose() <<std::endl;
    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
//    cyl->Rotate(0.001f, Axis::Y);
    cube->Rotate(0.1f, Axis::XYZ);

    IKCyclicCoordinateDecentMethod();
    IKFabrikMethod();
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        // if (pickedModel)
        //     debug("found ", pickedModel->isPickable ? "pickable" : "non-pickable", " model at pos ", x, ", ", y, ": ",
        //           pickedModel->name, ", depth: ", pickedModelDepth);
        // else
        //     debug("found nothing at pos ", x, ", ", y);

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        //pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
        //pickedToutAtPress = pickedModel->GetTout();


        // When one link of the arm is picked and being translated move all the arm
        // accordingly.The arm must not break!
        // Change ScrollCallback callback to translate the picked object away and to the
        // camera(perpendicular to camera plane).When no object is picked translate the
        // whole scene.
        if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
            cyls[0]->TranslateInSystem(system, {0, 0, -float(yoffset)});
            pickedToutAtPress = pickedModel->GetTout();
        }
        else {
            pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
            pickedToutAtPress = pickedModel->GetTout();
        }
    } else {
        camera->TranslateInSystem(system, {0, 0, -float(yoffset)});
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) {
                //pickedModel->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });


                // When one link of the arm is picked and being translated move all the arm
                // accordingly.The arm must not break!
                // Right mouse button will translate the whole scene or the picked object.
                if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
                    //pickedModel->TranslateInSystem(system * pickedModel->GetRotation(), { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                    cyls[0]->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                }
                else {
                    pickedModel->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                }
            }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                //pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                //pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);


                // Left mouse button will rotate objects or the scene in the same manner of the arrows
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);

                if (pickedModel == cyls[0]) {
                    pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
                }
                else {
                    pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::Z);
                } 
            }
        } else {
           // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff/10.0f, float( yAtPress - y) / moveCoeff/10.0f, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress =  x;
        yAtPress =  y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            //case GLFW_KEY_UP:
            //    cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::X);
            //    break;
            //case GLFW_KEY_DOWN:
            //    cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::X);
            //    break;
            //case GLFW_KEY_LEFT:
            //    cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::Y);
            //    break;
            //case GLFW_KEY_RIGHT:
            //    cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::Y);
            //    break;
            //case GLFW_KEY_W:
            //    camera->TranslateInSystem(system, {0, 0.1f, 0});
            //    break;
            //case GLFW_KEY_S:
            //    camera->TranslateInSystem(system, {0, -0.1f, 0});
            //    break;
            //case GLFW_KEY_A:
            //    camera->TranslateInSystem(system, {-0.1f, 0, 0});
            //    break;
            //case GLFW_KEY_D:
            //    camera->TranslateInSystem(system, {0.1f, 0, 0});
            //    break;
            case GLFW_KEY_B:
                camera->TranslateInSystem(system, {0, 0, 0.1f});
                break;
            case GLFW_KEY_F:
                camera->TranslateInSystem(system, {0, 0, -0.1f});
                break;
            case GLFW_KEY_1:
                if(pickedIndex > 0)
                  pickedIndex--;
                break;
            case GLFW_KEY_2:
                if(pickedIndex < cyls.size()-1)
                    pickedIndex++;
                break;
            case GLFW_KEY_3:
                if(tipIndex >= 0)
                {
                  if(tipIndex == cyls.size())
                    tipIndex--;
                  sphere1->Translate(GetSpherePos());
                  tipIndex--;
                }
                break;
            case GLFW_KEY_4:
                if(tipIndex < cyls.size())
                {
                    if(tipIndex < 0)
                      tipIndex++;
                    sphere1->Translate(GetSpherePos());
                    tipIndex++;
                }
                break;

            // New Keys
            case GLFW_KEY_SPACE: // IK solver
                Space_Callback();
                break;
            case GLFW_KEY_P: // prints rotation matrices
                P_Callback();
                break;
            case GLFW_KEY_T: // prints arms tip positions
                T_Callback();
                break;
            case GLFW_KEY_D: // prints destination position
                D_Callback();
                break;
            case GLFW_KEY_N: // pick the next link, or the first one in case the last link is picked
                N_Callback();
                break;
            case GLFW_KEY_RIGHT: // rotates picked link around the previous link Y axis
                Right_Callback();
                break;
            case GLFW_KEY_LEFT: // rotates picked link around the previous link Y axis
                Left_Callback();
                break;
            case GLFW_KEY_UP: // rotates picked link around the current X axis
                Up_Callback();
                break;
            case GLFW_KEY_DOWN: // rotates picked link around the current X axis
                Down_Callback();
                break;
            case GLFW_KEY_S: // switch IK modes
                S_Callback();
                break;
        }
    }
}

Eigen::Vector3f BasicScene::GetSpherePos()
{
      Eigen::Vector3f l = Eigen::Vector3f(1.6f,0,0);
      Eigen::Vector3f res;
      res = cyls[tipIndex]->GetRotation()*l;   
      return res;  
}

// New Functions
Eigen::Vector3f BasicScene::GetDestinationPosition()
{
    Eigen::Matrix4f destination_transform = sphere1->GetAggregatedTransform();
    Eigen::Vector3f destination_position = Eigen::Vector3f(destination_transform.col(3).x(), destination_transform.col(3).y(), destination_transform.col(3).z());

    return destination_position;
}

Eigen::Vector3f BasicScene::GetLinkTipPosition(int link_id)
{
    Eigen::Vector3f cyl_length = Eigen::Vector3f(0, 0, 0.8f);

    Eigen::Matrix4f arm_transform = cyls[link_id]->GetAggregatedTransform();
    Eigen::Vector3f arm_center = Eigen::Vector3f(arm_transform.col(3).x(), arm_transform.col(3).y(), arm_transform.col(3).z());
    Eigen::Vector3f arm_tip_position = arm_center + cyls[link_id]->GetRotation() * cyl_length;

    return arm_tip_position;
}

Eigen::Vector3f BasicScene::GetLinkSourcePosition(int link_id) {
    Eigen::Vector3f cyl_length = Eigen::Vector3f(0, 0, 0.8f);

    Eigen::Matrix4f arm_transform = cyls[link_id]->GetAggregatedTransform();
    Eigen::Vector3f arm_center = Eigen::Vector3f(arm_transform.col(3).x(), arm_transform.col(3).y(), arm_transform.col(3).z());
    Eigen::Vector3f arm_source_position = arm_center - cyls[link_id]->GetRotation() * cyl_length;

    return arm_source_position;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles (x and z are swapped).
// https://learnopencv.com/rotation-matrix-to-euler-angles/
Eigen::Vector3f BasicScene::RotationMatrixToEulerAngles(Eigen::Matrix3f R)
{
    float sy = sqrt(R.row(0).x() * R.row(0).x() + R.row(1).x() * R.row(1).x());

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.row(2).y(), R.row(2).z());
        y = atan2(-R.row(2).x(), sy);
        z = atan2(R.row(1).x(), R.row(0).x());
    }
    else
    {
        x = atan2(-R.row(1).z(), R.row(1).y());
        y = atan2(-R.row(2).x(), sy);
        z = 0;
    }
    return Eigen::Vector3f(x, y, z);
}

void BasicScene::IKCyclicCoordinateDecentMethod() {
    if (animate_CCD) {
        int first_link_id = 0;
        int last_link_id = 2;
        int num_of_links = 3;
        float link_length = 1.6f;

        Eigen::Vector3f D = GetDestinationPosition();
        Eigen::Vector3f first_link_position = GetLinkSourcePosition(first_link_id);

        if ((D - first_link_position).norm() > link_length * num_of_links) {
            std::cout << "cannot reach" << std::endl;
            animate_CCD = false;
            return;
        }

        int curr_link = last_link_id;

        while (curr_link != -1) {
            Eigen::Vector3f R = GetLinkSourcePosition(curr_link);
            Eigen::Vector3f E = GetLinkTipPosition(last_link_id);
            Eigen::Vector3f RD = D - R;
            Eigen::Vector3f RE = E - R;
            Eigen::Vector3f normal = RE.normalized().cross(RD.normalized()); //returns the plane normal
            float distance = (D - E).norm();

            if (distance < delta) {
                std::cout << "distance: " << distance << std::endl;
                //fix_rotate();
                animate_CCD = false;
                return;
            }

            //get dot product
            float dot = RD.normalized().dot(RE.normalized());

            //check that it is between -1 to 1
            if (dot > 1) dot = 1;
            if (dot < -1) dot = -1;

            float angle = (acosf(dot) * (180.f / 3.14f)) / 100.f;
            int parent_id = curr_link - 1;

            Eigen::Vector3f rotation_vector = cyls[curr_link]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;

            //the link has a parent
            //if (parent_id != -1) {
            //    cyls[curr_link]->RotateByDegree(angle, rotation_vector);
            // 
            //    E = GetLinkTipPosition(last_link_id); //get new position after rotation
            //    RE = E - R;
            //    Eigen::Vector3f R_parent = GetLinkSourcePosition(parent_id);
            //    RD = R_parent - R;
            // 
            //    //find angle between parent and link
            //    float constrain = 30;
            // 
            //    //get dot product
            //    dot = RD.normalized().dot(RE.normalized());  
            // 
            //    //check that it is between -1 to 1
            //    if (dot > 1) dot = 1;
            //    if (dot < -1) dot = -1;
            // 
            //    float parent_angle = (acos(dot) * (180.f / 3.14f));
            //    cyls[curr_link]->RotateByDegree(-angle, rotation_vector); //rotate back 
            // 
            //    if (parent_angle < constrain) { //fix angle
            //        angle = angle - (constrain - parent_angle);
            //    }
            //}

            cyls[curr_link]->RotateByDegree(angle, rotation_vector);
            curr_link = parent_id;
        }
    }
}

void BasicScene::fix_rotate() {
    int first_link_id = 0;
    int num_of_links = 3;
    auto system = camera->GetRotation().transpose() * GetRotation();

    Eigen::Vector3f Z(0, 0, 1);
    int curr_link = first_link_id;

    while (curr_link != num_of_links) {
        Eigen::Matrix3f R = cyls[curr_link]->GetRotation();
        Eigen::Vector3f ea = R.eulerAngles(2, 0, 2);//get the rotation angles
        float angleZ = ea[2];
        cyls[curr_link]->RotateByDegree(-angleZ, Z);

        curr_link = curr_link + 1;
        if (curr_link != num_of_links) {
            cyls[curr_link]->RotateInSystem(system * cyls[curr_link]->GetRotation(), angleZ, Axis::Z);
        }
    }
}

void BasicScene::IKFabrikMethod() {
    if (animate_Fabrik) {
        int first_link_id = 0;
        int last_link_id = 2;
        int num_of_links = 3;
        float link_length = 1.6f;

        std::vector<Eigen::Vector3f> p; //joint positions
        p.resize(num_of_links + 1);
        Eigen::Vector3f t = GetDestinationPosition();
        Eigen::Vector3f root = GetLinkSourcePosition(first_link_id);

        //Set disjoint positions
        //p1 is first disjoin
        int curr = last_link_id;
        while (curr != -1) {
            p[curr] = GetLinkSourcePosition(curr);
            curr = curr - 1;
        }
        p[last_link_id + 1] = GetLinkTipPosition(last_link_id);
        std::vector<double> riArr;
        std::vector<double> lambdaIArr;

        riArr.resize(num_of_links + 1);
        lambdaIArr.resize(num_of_links + 1);

        if ((t - root).norm() > link_length * num_of_links) {
            //1.5. The target is unreachable
            std::cout << "cannot reach" << std::endl;
            animate_Fabrik = false;
            return;
        }
        else
        {
            //1.14. The target is reachable; thus set as b the initial position of joint p0
            Eigen::Vector3f b = p[first_link_id];

            //1.16. Check wether the distance between the end effector Pn and the target t is greater then a tolerance
            Eigen::Vector3f endEffector = p[last_link_id + 1];

            float diffA = (endEffector - t).norm();
            if (diffA < delta) {
                std::cout << "distance: " << diffA << std::endl;
                animate_Fabrik = false;
                return;
            }
            while (diffA > delta) {
                //1.19. STAGE 1: FORWARD REACHING
                p[last_link_id + 1] = t;
                int parent = last_link_id;
                int child = last_link_id + 1;
                while (parent != -1) {
                    //1.23. Find the distance ri between the new joint position pi+1 and the joint pi
                    riArr[parent] = (p[child] - p[parent]).norm();
                    lambdaIArr[parent] = link_length / riArr[parent];
                    p[parent] = (1 - lambdaIArr[parent]) * p[child] + lambdaIArr[parent] * p[parent]; //1.27
                    child = parent;
                    parent = parent - 1;

                }
                //1.29. STAGE 2: BACKWORD REACHING

                //1.30. Set the root p0 its initial position
                p[first_link_id] = b;
                parent = first_link_id;
                child = first_link_id + 1;
                while (child != num_of_links) {
                    //1.33. Find the distance ri between the new joint position pi and the joint pi+1
                    riArr[parent] = (p[child] - p[parent]).norm();
                    lambdaIArr[parent] = link_length / riArr[parent];
                    p[child] = (1 - lambdaIArr[parent]) * p[parent] + lambdaIArr[parent] * p[child]; //1.27
                    parent = child;
                    child = child + 1;
                }

                riArr[last_link_id] = (p[last_link_id + 1] - p[last_link_id]).norm();
                lambdaIArr[last_link_id] = link_length / riArr[last_link_id];
                p[last_link_id + 1] = (1 - lambdaIArr[last_link_id]) * p[last_link_id] + lambdaIArr[last_link_id] * p[last_link_id + 1]; //1.27
                diffA = (p[last_link_id + 1] - t).norm();
            }
            //rotate
            int currLink = first_link_id;
            int target_id = first_link_id + 1;

            while (target_id != num_of_links) {
                IKSolverHelper(currLink, p[target_id]);
                currLink = target_id;
                target_id = target_id + 1;
            }
            IKSolverHelper(last_link_id, p[last_link_id + 1]);
            float distance = (t - GetLinkTipPosition(last_link_id)).norm();

            if (distance < delta) {
                //fix_rotate();
                animate_Fabrik = false;
                std::cout << "distance: " << distance << std::endl;
            }
        }
    }
}

void BasicScene::IKSolverHelper(int id, Eigen::Vector3f t) {
    Eigen::Vector3f r = GetLinkSourcePosition(id);
    Eigen::Vector3f e = GetLinkTipPosition(id);
    Eigen::Vector3f rd = t - r;
    Eigen::Vector3f re = e - r;
    Eigen::Vector3f normal = re.normalized().cross(rd.normalized());
    float dot = rd.normalized().dot(re.normalized());//get dot 

    if (dot > 1) dot = 1;
    if (dot < -1) dot = 1;
    float angle = (acos(dot) * (180.f / 3.14f)) / 100.f;

    Eigen::Vector3f rotationVec = cyls[id]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;
    int parent = id - 1;

    //bonus
    if (parent != -1) {
        cyls[id]->RotateByDegree(angle, rotationVec);
        e = GetLinkTipPosition(id); //get new position after rotation
        re = e - r;
        Eigen::Vector3f r_parent = GetLinkSourcePosition(parent);
        rd = r_parent - r;
        //find angle between parent and link
        float constarin = 30;
        float parentDot = rd.normalized().dot(re.normalized());//get dot

        if (parentDot > 1) parentDot = 1;
        if (parentDot < -1) parentDot = 1;

        float parentAngle = (acos(parentDot) * (180.f / 3.14f));
        cyls[id]->RotateByDegree(-angle, rotationVec); //rotate back
        if (parentAngle < constarin) {//fix angle
            angle = angle - (constarin - parentAngle);
        }
    }
    cyls[id]->RotateByDegree(angle, rotationVec);
}

// New Callback Functions
void BasicScene::Space_Callback()
{
    if (IK_mode == 0) {
        if (!animate_CCD) {
            animate_CCD = true;
        }
        else {
            animate_CCD = false;
        }
    }
    else {
        if (!animate_Fabrik) {
            animate_Fabrik = true;
        }
        else {
            animate_Fabrik = false;
        }
    }
}

void BasicScene::P_Callback()
{
    if (pickedModel == cyls[0]) {
        Eigen::Matrix3f arm1_rotation = pickedModel->GetRotation();

        std::cout << "Arm1 Rotation: " << std::endl
            << "(" << arm1_rotation.row(0).x() << "," << arm1_rotation.row(0).y() << "," << arm1_rotation.row(0).z() << ")" << std::endl
            << "(" << arm1_rotation.row(1).x() << "," << arm1_rotation.row(1).y() << "," << arm1_rotation.row(1).z() << ")" << std::endl
            << "(" << arm1_rotation.row(2).x() << "," << arm1_rotation.row(2).y() << "," << arm1_rotation.row(2).z() << ")" << std::endl;

        Eigen::Vector3f arm1_euler_angles = arm1_rotation.eulerAngles(2, 0, 2) * (180.f / 3.14f);

        std::cout << "Arm1 Euler Angles: "
            << "(" << arm1_euler_angles.x()
            << ", " << arm1_euler_angles.y()
            << ", " << arm1_euler_angles.z()
            << ")" << std::endl;
    }
    else if (pickedModel == cyls[1]) {
        Eigen::Matrix3f arm2_rotation = pickedModel->GetRotation();

        std::cout << "Arm2 Rotation: " << std::endl
            << "(" << arm2_rotation.row(0).x() << "," << arm2_rotation.row(0).y() << "," << arm2_rotation.row(0).z() << ")" << std::endl
            << "(" << arm2_rotation.row(1).x() << "," << arm2_rotation.row(1).y() << "," << arm2_rotation.row(1).z() << ")" << std::endl
            << "(" << arm2_rotation.row(2).x() << "," << arm2_rotation.row(2).y() << "," << arm2_rotation.row(2).z() << ")" << std::endl;

        Eigen::Vector3f arm2_euler_angles = arm2_rotation.eulerAngles(2, 0, 2) * (180.f / 3.14f);

        std::cout << "Arm2 Euler Angles: "
            << "(" << arm2_euler_angles.x()
            << ", " << arm2_euler_angles.y()
            << ", " << arm2_euler_angles.z()
            << ")" << std::endl;
    }
    else if (pickedModel == cyls[2]) {
        Eigen::Matrix3f arm3_rotation = pickedModel->GetRotation();

        std::cout << "Arm3 Rotation: " << std::endl
            << "(" << arm3_rotation.row(0).x() << "," << arm3_rotation.row(0).y() << "," << arm3_rotation.row(0).z() << ")" << std::endl
            << "(" << arm3_rotation.row(1).x() << "," << arm3_rotation.row(1).y() << "," << arm3_rotation.row(1).z() << ")" << std::endl
            << "(" << arm3_rotation.row(2).x() << "," << arm3_rotation.row(2).y() << "," << arm3_rotation.row(2).z() << ")" << std::endl;

        Eigen::Vector3f arm3_euler_angles = arm3_rotation.eulerAngles(2, 0, 2) * (180.f / 3.14f);

        std::cout << "Arm3 Euler Angles: "
            << "(" << arm3_euler_angles.x()
            << ", " << arm3_euler_angles.y()
            << ", " << arm3_euler_angles.z()
            << ")" << std::endl;
    }
    else {
        Eigen::Matrix3f scene_rotation = root->GetRotation();

        std::cout << "Scene Rotation: " << std::endl
            << "(" << scene_rotation.row(0).x() << "," << scene_rotation.row(0).y() << "," << scene_rotation.row(0).z() << ")" << std::endl
            << "(" << scene_rotation.row(1).x() << "," << scene_rotation.row(1).y() << "," << scene_rotation.row(1).z() << ")" << std::endl
            << "(" << scene_rotation.row(2).x() << "," << scene_rotation.row(2).y() << "," << scene_rotation.row(2).z() << ")" << std::endl;
    }
}

void BasicScene::T_Callback()
{
    Eigen::Vector3f arm1_tip_position = GetLinkTipPosition(0);
    Eigen::Vector3f arm2_tip_position = GetLinkTipPosition(1);
    Eigen::Vector3f arm3_tip_position = GetLinkTipPosition(2);

    std::cout << "Arm1 Tip Position: "
        << "(" << arm1_tip_position.x()
        << ", " << arm1_tip_position.y()
        << ", " << arm1_tip_position.z()
        << ")" << std::endl;

    std::cout << "Arm2 Tip Position: "
        << "(" << arm2_tip_position.x()
        << ", " << arm2_tip_position.y()
        << ", " << arm2_tip_position.z()
        << ")" << std::endl;

    std::cout << "Arm3 Tip Position: "
        << "(" << arm3_tip_position.x()
        << ", " << arm3_tip_position.y()
        << ", " << arm3_tip_position.z()
        << ")" << std::endl;
}

void BasicScene::D_Callback() 
{
    Eigen::Vector3f destination_position = GetDestinationPosition();

    std::cout << "Destination Position: "
        << "(" << destination_position.x()
        << ", " << destination_position.y()
        << ", " << destination_position.z()
        << ")" << std::endl;
}

void BasicScene::N_Callback()
{
    if (pickedModel == cyls[0]) {
        pickedModel = cyls[1];
    }
    else if (pickedModel == cyls[1]) {
        pickedModel = cyls[2];
    }
    // Last link or any other model
    else {
        pickedModel = cyls[0];
    }
}

void BasicScene::Right_Callback()
{
    auto system = camera->GetRotation().transpose();

    if (pickedModel == cyls[0]) {
        pickedModel->RotateInSystem(system, -0.1f, Axis::Y);
    }
    else if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
        pickedModel->RotateInSystem(system, -0.1f, Axis::Y);
    }
    else {
        camera->TranslateInSystem(system, { 0.1f, 0, 0 });
    }
}

void BasicScene::Left_Callback()
{
    auto system = camera->GetRotation().transpose();

    if (pickedModel == cyls[0]) {
        pickedModel->RotateInSystem(system, 0.1f, Axis::Y);
    }
    else if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
        pickedModel->RotateInSystem(system, 0.1f, Axis::Y);
    }
    else {
        camera->TranslateInSystem(system, { -0.1f, 0, 0 });
    }
}

void BasicScene::Up_Callback()
{
    auto system = camera->GetRotation().transpose();

    if (pickedModel == cyls[0]) {
        pickedModel->RotateInSystem(system, 0.1f, Axis::X);
    }
    else if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
        pickedModel->RotateInSystem(system, 0.1f, Axis::Z);
    }
    else {
        camera->TranslateInSystem(system, { 0, 0.1f, 0 });
    }
}

void BasicScene::Down_Callback()
{
    auto system = camera->GetRotation().transpose();

    if (pickedModel == cyls[0]) {
        pickedModel->RotateInSystem(system, -0.1f, Axis::X);
    }
    else if ((pickedModel == cyls[1]) || (pickedModel == cyls[2])) {
        pickedModel->RotateInSystem(system, -0.1f, Axis::Z);
    }
    else {
        camera->TranslateInSystem(system, { 0, -0.1f, 0 });
    }
}

void BasicScene::S_Callback()
{
    animate_CCD = false;
    animate_Fabrik = false;

    if (IK_mode == 1) {
        std::cout << "IK mode: Cyclic Coordinate Decent Method" << std::endl;
        IK_mode = 0;
    }
    else {
        std::cout << "IK mode: Fabrik Method" << std::endl;
        IK_mode = 1;
    }
}
