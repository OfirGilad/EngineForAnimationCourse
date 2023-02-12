#include "Libigl_404_Scene.h"

// #include "AutoMorphingModel.h"

const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);

using namespace cg3d;

void Libigl_404_Scene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) }; // empty material
    // SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto camelMesh{ IglLoader::MeshFromFiles("camel_igl","data/camel_b.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };

    sphere1 = Model::Create("sphere", sphereMesh, material);
    camel = Model::Create("camel", camelMesh, material);
    cube = Model::Create("cube", cubeMesh, material);
    sphere1->Scale(2);
    sphere1->showWireframe = true;
    sphere1->Translate({ -3,0,0 });
    camel->Translate({ 5,0,0 });
    camel->Scale(0.12f);
    camel->showWireframe = true;
    cube->showWireframe = true;
    camera->Translate(30, Axis::Z);
    root->AddChild(sphere1);
    root->AddChild(camel);
    root->AddChild(cube);

    auto mesh = sphere1->GetMeshList();
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi F, E, EF, EI;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C;
    int num_collapsed;

    // Function to reset original mesh and data structures
    V = mesh[0]->data[0].vertices;
    F = mesh[0]->data[0].faces;
    // igl::read_triangle_mesh("data/cube.off",V,F);
    igl::edge_flaps(F, E, EMAP, EF, EI);
    std::cout << "vertices: \n" << V << std::endl;
    std::cout << "faces: \n" << F << std::endl;

    std::cout << "edges: \n" << E.transpose() << std::endl;
    std::cout << "edges to faces: \n" << EF.transpose() << std::endl;
    std::cout << "faces to edges: \n " << EMAP.transpose() << std::endl;
    std::cout << "edges indices: \n" << EI.transpose() << std::endl;

    // New Code - Start

    using namespace std;
    using namespace Eigen;
    using namespace igl;
    

    const auto& pre_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool
    {
        using namespace Eigen;
        using namespace std;
        if (recompute)
        {
            // Find pose interval
            const int begin = (int)floor(anim_t) % poses.size();
            const int end = (int)(floor(anim_t) + 1) % poses.size();
            const double t = anim_t - floor(anim_t);

            // Interpolate pose and identity
            RotationList anim_pose(poses[begin].size());
            for (int e = 0;e < poses[begin].size();e++)
            {
                anim_pose[e] = poses[begin][e].slerp(t, poses[end][e]);
            }
            // Propagate relative rotations via FK to retrieve absolute transformations
            // vQ - rotations of joints
            // vT - translation of joints
            RotationList vQ;
            vector<Vector3d> vT;
            igl::forward_kinematics(C, BE, P, anim_pose, vQ, vT);
            const int dim = C.cols();
            MatrixXd T(BE.rows() * (dim + 1), dim);
            for (int e = 0;e < BE.rows();e++)
            {
                Affine3d a = Affine3d::Identity();
                a.translate(vT[e]);
                a.rotate(vQ[e]);
                T.block(e * (dim + 1), 0, dim + 1, dim) =
                    a.matrix().transpose().block(0, 0, dim + 1, dim);
            }
            // Compute deformation via LBS as matrix multiplication
            if (use_dqs)
            {
                igl::dqs(V, W, vQ, vT, U);
            }
            else
            {
                U = M * T;
            }

            // Also deform skeleton edges
            MatrixXd CT;
            MatrixXi BET;
            //move joints according to T, returns new position in CT and BET
            std::cout << "before" << std::endl;
            std::cout << BE << std::endl;
            igl::deform_skeleton(C, BE, T, CT, BET);
            std::cout << "after" << std::endl;
            std::cout << BE << std::endl;
            std::cout << BET << std::endl;
            viewer.data().set_vertices(U);
            viewer.data().set_edges(CT, BET, sea_green);
            viewer.data().compute_normals();
            if (viewer.core().is_animating)
            {
                anim_t += anim_t_dir;
            }
            else
            {
                recompute = false;
            }
        }
        return false;
    };

    const auto& key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod)->bool
    {
        recompute = true;
        switch (key)
        {
        case 'D':
        case 'd':
            use_dqs = !use_dqs;
            return true;
        case ' ':
            viewer.core().is_animating = !viewer.core().is_animating;
            return true;
        }
        return false;
    };

    igl::readOBJ("data/arm.obj", V, F);
    U = V;
    igl::readTGF("data/arm.tgf", C, BE);
    // retrieve parents for forward kinematics
    igl::directed_edge_parents(BE, P);
    RotationList rest_pose;
    igl::directed_edge_orientations(C, BE, rest_pose);
    poses.resize(4, RotationList(4, Quaterniond::Identity()));
    // poses[1] // twist
    const Quaterniond twist(AngleAxisd(igl::PI, Vector3d(1, 0, 0)));
    poses[1][2] = rest_pose[2] * twist * rest_pose[2].conjugate();
    const Quaterniond bend(AngleAxisd(-igl::PI * 0.7, Vector3d(0, 0, 1)));
    poses[3][2] = rest_pose[2] * bend * rest_pose[2].conjugate();

    igl::readDMAT("data/arm-weights.dmat", W);
    igl::lbs_matrix(V, W, M);

    // Plot the mesh with pseudocolors
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F);
    viewer.data().set_edges(C, BE, sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.core().trackball_angle.normalize();
    viewer.callback_pre_draw = pre_draw;
    viewer.callback_key_down = key_down;
    viewer.core().is_animating = false;
    viewer.core().camera_zoom = 2.5;
    viewer.core().animation_max_fps = 30.;
    cout << "Press [d] to toggle between LBS and DQS" << endl <<
        "Press [space] to toggle animation" << endl;
    viewer.launch();

    // New Code - End
}

void Libigl_404_Scene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    // cube->Rotate(0.01f, Axis::All);
    sphere1->Rotate(0.005f, Axis::Y);
    cube->Rotate(0.005f, Axis::Y);
    camel->Rotate(0.005f, Axis::Y);
}
