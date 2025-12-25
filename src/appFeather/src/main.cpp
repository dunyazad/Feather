#include <robin_hood.h>

#include <libFeather.h>

#include "GeometricProcessing/GeometricProcessingPipeline.h"

static inline std::string FormatWithCommas(size_t value)
{
    std::string numStr = std::to_string(value);
    int insertPosition = static_cast<int>(numStr.length()) - 3;
    while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
    return numStr;
}

namespace GPP = GeometricProcessingPipeline;

#include <Eigen/Core>

#define OPERATOR(operatorName) pipeline.AddOperator<GPP::operatorName>(#operatorName)
#define OPERATOR_PARAMETER(operatorName, parameter) pipeline.AddOperator<GPP::operatorName>(#operatorName, parameter);
#define EXECUTE_AND_VISUALIZE_RETURN() { pipeline.Execute(); pipeline.VisualizeLast(); return; }

std::string plyFilename = "D:\\Temp\\PLY\\Compound_G.ply";

void LoadPointCloudFromPLY(const std::string& filename, GPP::PointCloud& pointCloud)
{
    auto entity = Feather.CreateEntity("PointCloud");

    Feather.CreateEventCallback<KeyEvent>(entity, [](Entity entity, const KeyEvent& event) {
        auto renderable = Feather.GetComponent<Renderable>(entity);
        if (nullptr == renderable) return;

        if (0 == event.action)
        {
            if (GLFW_KEY_GRAVE_ACCENT == event.keyCode)
            {
                renderable->NextDrawingMode();
            }
            else if (GLFW_KEY_1 == event.keyCode)
            {
                renderable->SetActiveShaderIndex(0);
            }
            else if (GLFW_KEY_2 == event.keyCode)
            {
                renderable->SetActiveShaderIndex(1);
            }
        }
        });

    auto renderable = Feather.CreateComponent<Renderable>(entity);

    PLYFormat ply;
    ply.Deserialize(plyFilename);

    renderable->Initialize(Renderable::GeometryMode::Triangles);

    renderable->Initialize(Renderable::GeometryMode::Triangles);
    renderable->AddShader(Feather.CreateShader("Instancing", File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs")));
    renderable->AddShader(Feather.CreateShader("InstancingWithoutNormal", File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs")));
    renderable->SetActiveShaderIndex(1);

    auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere({ 0.0f, 0.0f, 0.0f }, 0.5f, 6, 6);
    renderable->AddIndices(indices);
    renderable->AddVertices(vertices);
    renderable->AddNormals(normals);
    renderable->AddColors(colors);
    renderable->AddUVs(uvs);

	pointCloud.Resize(ply.GetPoints().size() / 3);

    for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
    {
        auto px = ply.GetPoints()[3 * i];
        auto py = ply.GetPoints()[3 * i + 1];
        auto pz = ply.GetPoints()[3 * i + 2];

        auto nx = ply.GetNormals()[3 * i];
        auto ny = ply.GetNormals()[3 * i + 1];
        auto nz = ply.GetNormals()[3 * i + 2];

        renderable->AddInstanceNormal({ nx, ny, nz });

        if (ply.UseAlpha())
        {
            auto r = ply.GetColors()[4 * i];
            auto g = ply.GetColors()[4 * i + 1];
            auto b = ply.GetColors()[4 * i + 2];
            auto a = ply.GetColors()[4 * i + 3];

            renderable->AddInstanceColor({ r, g, b, a });

			pointCloud.colors[i] = Eigen::Vector3f(r, g, b);
        }
        else
        {
            auto r = ply.GetColors()[3 * i];
            auto g = ply.GetColors()[3 * i + 1];
            auto b = ply.GetColors()[3 * i + 2];

            renderable->AddInstanceColor({ r, g, b, 1.0f });

            pointCloud.colors[i] = Eigen::Vector3f(r, g, b);
        }

		pointCloud.positions[i] = Eigen::Vector3f(px, py, pz);
		pointCloud.normals[i] = Eigen::Vector3f(nx, ny, nz);
        if (false == ply.GetDeepLearningClasses().empty())
        {
            pointCloud.pointDeepLearningClassIDs[i] = ply.GetDeepLearningClasses()[i];
        }

        glm::mat4 tm = glm::identity<glm::mat4>();
        glm::mat4 rot = glm::mat4(1.0f);
        if (glm::length(glm::vec3(nx, ny, nz)) > 0.0001f)
        {
            glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), glm::vec3(nx, ny, nz)));
            float angle = acos(glm::dot(glm::normalize(glm::vec3(nx, ny, nz)), glm::vec3(0, 0, 1)));
            if (glm::length(axis) > 0.0001f)
                rot = glm::rotate(glm::mat4(1.0f), angle, axis);
        }

        tm = glm::translate(tm, glm::vec3(px, py, pz)) * rot * glm::scale(glm::mat4(1.0f), glm::vec3(0.1f));

        renderable->AddInstanceTransform(tm);
        renderable->IncreaseNumberOfInstances();
    }

	auto [mx, my, mz] = ply.GetAABBMin();
	auto [Mx, My, Mz] = ply.GetAABBMax();
    pointCloud.aabb = Eigen::AABB{ Eigen::Vector3f(mx, my, mz), Eigen::Vector3f(Mx, My, Mz) };
}

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Final Optimized" << std::endl;
    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

    GeometricProcessingPipeline::Pipeline pipeline;

    {
        auto appMain = Feather.CreateEntity("AppMain");
        Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
            if (GLFW_KEY_ESCAPE == event.keyCode) glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
            else if (GLFW_KEY_SPACE == event.keyCode && event.action == 0) Feather.GetImmediateModeRenderSystem()->ToggleEnable();
            else if (GLFW_KEY_BACKSPACE == event.keyCode && event.action == 0) VD::SetVisibilityAll(false);

#if 0
            else if (GLFW_KEY_F1 == event.keyCode && event.action == 0)
            {
                //VD::ToggleVisibility("Mesh");
                auto entity = Feather.GetEntityByName("Mesh");
                auto renderable = Feather.GetComponent<Renderable>(entity);
                if (renderable)
                {
                    renderable->ToggleVisible();
                }
            }
            else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("Blocks");
            else if (GLFW_KEY_F3 == event.keyCode && event.action == 0) VD::ToggleVisibility("Voxels");
            else if (GLFW_KEY_F4 == event.keyCode && event.action == 0) VD::ToggleVisibility("Points");
            else if (GLFW_KEY_F5 == event.keyCode && event.action == 0) VD::ToggleVisibility("DLCs");
            else if (GLFW_KEY_F6 == event.keyCode && event.action == 0) VD::ToggleVisibility("Curvature");
            else if (GLFW_KEY_F7 == event.keyCode && event.action == 0) VD::ToggleVisibility("Holes");
#endif // 0
            //else if (GLFW_KEY_F1 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridPoints");
            //else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridCells");
            else if (GLFW_KEY_F1 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_original");
            else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_sink");
            else if (GLFW_KEY_F3 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_source");
            else if (GLFW_KEY_F9 == event.keyCode && event.action == 0)
            {
                json j;

                std::ifstream in("camera_state.json");
                if (in.is_open())
                {
                    in >> j;
                    if (j.contains(plyFilename))
                    {
                        glm::vec3 eye, target, up;
                        eye.x = j[plyFilename]["eye"][0];
                        eye.y = j[plyFilename]["eye"][1];
                        eye.z = j[plyFilename]["eye"][2];

                        target.x = j[plyFilename]["target"][0];
                        target.y = j[plyFilename]["target"][1];
                        target.z = j[plyFilename]["target"][2];

                        up.x = j[plyFilename]["up"][0];
                        up.y = j[plyFilename]["up"][1];
                        up.z = j[plyFilename]["up"][2];

                        auto camEnt = Feather.GetEntityByName("Camera");
                        if (camEnt != entt::null)
                        {
                            auto cam = Feather.GetComponent<Camera>(camEnt);
                            auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(camEnt);
                            if (cam)
                            {
                                cam->SetEye(eye);
                                cam->SetTarget(target);
                                cam->SetUp(up);
                                cam->SetDirty(true);
                                if (manipulator)
                                {
                                    manipulator->SyncRadius();
                                }
                                std::cout << "[System] Camera state RESTORED from JSON." << std::endl;
                            }
                        }
                    }
                }
                else
                {
                    std::cout << "[System] No saved camera state JSON file found." << std::endl;
                }

                //std::ifstream in("camera_state.txt");
                //if (in.is_open())
                //{
                //    glm::vec3 eye, target, up;
                //    in >> eye.x >> eye.y >> eye.z;
                //    in >> target.x >> target.y >> target.z;
                //    in >> up.x >> up.y >> up.z;

                //    auto camEnt = Feather.GetEntityByName("Camera");
                //    if (camEnt != entt::null)
                //    {
                //        auto cam = Feather.GetComponent<Camera>(camEnt);
                //        auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(camEnt);

                //        if (cam)
                //        {
                //            cam->SetEye(eye);
                //            cam->SetTarget(target);
                //            cam->SetUp(up);
                //            cam->SetDirty(true);

                //            if (manipulator)
                //            {
                //                manipulator->SyncRadius();
                //            }

                //            std::cout << "[System] Camera state RESTORED." << std::endl;
                //        }
                //    }
                //}
                //else
                //{
                //    std::cout << "[System] No saved camera state file found." << std::endl;
                //}
            }
            else if (GLFW_KEY_F12 == event.keyCode && event.action == 0)
            {
                auto camEnt = Feather.GetEntityByName("Camera");
                if (camEnt != entt::null)
                {
                    auto cam = Feather.GetComponent<Camera>(camEnt);
                    if (cam)
                    {
                        std::ifstream in("camera_state.json");
                        if (in.is_open())
                        {
                            json j;
                            in >> j;

                            glm::vec3 eye = cam->GetEye();
                            glm::vec3 target = cam->GetTarget();
                            glm::vec3 up = cam->GetUp();

                            j[plyFilename] = {
                                { "eye",    { eye.x,    eye.y,    eye.z } },
                                { "target", { target.x, target.y, target.z } },
                                { "up",     { up.x,     up.y,     up.z } }
                            };

                            std::ofstream out("camera_state.json");
                            out << j.dump(4);
                        }
                    }
                }
            }
            });
    }
    {
        Entity cam = Feather.CreateEntity("Camera");
        auto pcam = Feather.CreateComponent<Camera>(cam);
        auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
        pcamMan->SetCamera(pcam);
        Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
            pcam->GetPerspectiveSettings().SetAspectRatio((f32)Feather.GetFeatherWindow()->GetWidth() / (f32)Feather.GetFeatherWindow()->GetHeight());
            });
        Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event); });
        Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event); });
        Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
            auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(entity);
            manipulator->OnMouseButton(event);

            if (event.button == GLFW_MOUSE_BUTTON_LEFT && event.action == 1)
            {
                auto window = Feather.GetFeatherWindow();
                GLFWwindow* nativeWin = window->GetGLFWwindow();

                int winW, winH, fbW, fbH;
                glfwGetWindowSize(nativeWin, &winW, &winH);
                glfwGetFramebufferSize(nativeWin, &fbW, &fbH);

                double mx, my;
                glfwGetCursorPos(nativeWin, &mx, &my);

                float pxRatio = (float)fbW / (float)winW;
                float pyRatio = (float)fbH / (float)winH;
                mx *= pxRatio;
                my *= pyRatio;

                float ndcX = (2.0f * (float)mx) / (float)fbW - 1.0f;
                float ndcY = 1.0f - (2.0f * (float)my) / (float)fbH;

                auto cameraComp = Feather.GetComponent<Camera>(cam);
                glm::mat4 view = cameraComp->GetViewMatrix();
                glm::mat4 proj = cameraComp->GetProjectionMatrix();
                glm::mat4 invVP = glm::inverse(proj * view);

                glm::vec4 screenPosNear(ndcX, ndcY, -1.0f, 1.0f);
                glm::vec4 screenPosFar(ndcX, ndcY, 1.0f, 1.0f);

                glm::vec4 worldPosNear = invVP * screenPosNear;
                glm::vec4 worldPosFar = invVP * screenPosFar;

                if (worldPosNear.w != 0.0f) worldPosNear /= worldPosNear.w;
                if (worldPosFar.w != 0.0f) worldPosFar /= worldPosFar.w;

                Eigen::Vector3f rayOrigin = Eigen::Vector3f(worldPosNear.x, worldPosNear.y, worldPosNear.z);
                auto delta = worldPosFar - worldPosNear;
                Eigen::Vector3f rayDir = (Eigen::Vector3f(delta.x, delta.y, delta.z)).normalized();

                Eigen::Ray ray{ rayOrigin, rayDir };

                auto result = pipeline.GetSparseGrid()->PickBruteForce(pipeline.GetCurrentPointCloud()->positions, ray, GeometricProcessingPipeline::Configuration::pointVisualizationRadius);

                if (result.hasHit)
                {
                    VD::Clear("PickedPoint");
                    VD::Clear("PickedCell");

                    auto p = pipeline.GetCurrentPointCloud()->positions[result.pointIndex];
                    //VD::AddSphere("PickedPoint", p, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));

                    if (Feather.IsKeyPressed(GLFW_KEY_LEFT_CONTROL) || Feather.IsKeyPressed(GLFW_KEY_RIGHT_CONTROL))
                    {
                        manipulator->SetCenter(glm::vec3(p.x(), p.y(), p.z()));
                    }

                    glm::vec3 cellMin =
                        glm::vec3(
                            pipeline.GetSparseGrid()->aabb.min.x(),
                            pipeline.GetSparseGrid()->aabb.min.y(),
                            pipeline.GetSparseGrid()->aabb.min.z())
                        + glm::vec3(
                            (float)result.gx * pipeline.GetSparseGrid()->cellSize,
                            (float)result.gy * pipeline.GetSparseGrid()->cellSize,
                            (float)result.gz * pipeline.GetSparseGrid()->cellSize);

                    glm::vec3 cellMax = cellMin + glm::vec3(pipeline.GetSparseGrid()->cellSize);
                    VD::AddWiredBox("PickedCell", { cellMin, cellMax }, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

                    alog("Hit! Idx:%d, Cell(%d,%d,%d), Dist:%.2f\n", result.pointIndex, result.gx, result.gy, result.gz, result.distance);

                    {
                        //auto op = pipeline.GetOperator(-1);
                        //auto operatorLocalPlaneFitting = std::dynamic_pointer_cast<GPP::OperatorLocalPlaneFitting>(op);
                        //if (operatorLocalPlaneFitting)
                        //{
                        //    auto fittedPoints = operatorLocalPlaneFitting->GetFiitedPoints();
                        //    if (result.pointIndex < fittedPoints.size())
                        //    {
                        //        auto fp = fittedPoints[result.pointIndex];
                        //        VD::AddSphere("FittedPoint", fp, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
                        //        VD::AddLine("FittedPointLine", p, fp, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
                        //    }
                        //}
                    }
                }
            }
            });
        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) { Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event); });
    }

    {
        auto entity = Feather.CreateEntity("TreeViewPanel");
        auto component = Feather.CreateComponent<TreeViewPanel>(entity);
    }

    Feather.AddOnInitializeCallback([&]()
        {
#if 0
            {
                GPP::PointCloud pointCloud;
                GPP::SparseGrid sparseGrid;

                LoadPointCloudFromPLY(plyFilename, pointCloud);

                TS(BuildingSparseGrid);
                sparseGrid.Build(pointCloud, GPP::Configuration::voxelSize);
                TE(BuildingSparseGrid);

                int minNeighborCount = 0;
                int maxNeighborCount = 0;
                std::vector<int> neighborCounts(pointCloud.numberOfElements, 0);

                TS(NeighborCount);
                {
                    std::vector<int> indices(pointCloud.numberOfElements);
                    std::iota(indices.begin(), indices.end(), 0);


                    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                        {
                            const Eigen::Vector3f& p = pointCloud.positions[i];

                            int neighborCount = 0;
                            int neighborOffset = 1;

                            auto cellIdx = sparseGrid.GetIndex(p);
                            for (int zo = -neighborOffset; zo <= neighborOffset; zo++)
                            {
                                for (int yo = -neighborOffset; yo <= neighborOffset; yo++)
                                {
                                    for (int xo = -neighborOffset; xo <= neighborOffset; xo++)
                                    {
                                        auto neighborKey = sparseGrid.GetKey(cellIdx.x() + xo, cellIdx.y() + yo, cellIdx.z() + zo);
                                        auto neighborIt = sparseGrid.voxelPointListHead.find(neighborKey);
                                        if (neighborIt != sparseGrid.voxelPointListHead.end())
                                        {
                                            int neighborHeadIndex = neighborIt->second;
                                            int currentIndex = neighborHeadIndex;
                                            while (currentIndex != -1)
                                            {
                                                neighborCount++;
                                                currentIndex = sparseGrid.nextPoint[currentIndex];
                                            }
                                        }
                                    }
                                }
                            }

                            neighborCounts[i] = neighborCount;
                        });

                    auto result = std::minmax_element(std::execution::par, neighborCounts.begin(), neighborCounts.end());

                    minNeighborCount = *result.first;
                    maxNeighborCount = *result.second;
                }
                TE(NeighborCount);

                TS(Visualize);
                {
                    auto entity = Feather.GetEntityByName("PointCloud");
                    auto renderable = Feather.GetComponent<Renderable>(entity);

                    for (size_t i = 0; i < pointCloud.numberOfElements; i++)
                    {
                        auto& color = pointCloud.colors[i];

                        auto nc = neighborCounts[i];

                        float t = 0.0f;
                        if (maxNeighborCount > minNeighborCount)
                            t = (float)(nc - minNeighborCount) / (float)(maxNeighborCount - minNeighborCount);
                        color = Eigen::Vector3f(1.0f - t, 0.0f, t);

                        renderable->SetInstanceColor(i, glm::vec4(color.x(), color.y(), color.z(), 1.0f));
                    }
                }
                TE(Visualize);
            }
#endif // 0


            std::thread([&]()
                {
                    OPERATOR(OperatorPointCloudLoader)->SetPLYFilename(plyFilename);

#pragma region Working
                    {
                        OPERATOR(OperatorStorePointCloud);

                        OPERATOR(OperatorCurvatureDeviation)->
							SetIgnoreOppositeNormals(true)->
                            SetNeighborSearchOffset(1)->
                            SetSearchRadiusMultiplier(0.3333333f)->
                            SetDeviationThreshold(0.07f);

                        OPERATOR(OperatorFilterMarked);

                        OPERATOR(OperatorClustering)->SetSearchRadiusMultiplier(0.35f);

                        OPERATOR(OperatorFilterLeaveLargestOnly);

                        //OPERATOR(OperatorPointCloudSaver)->SetPLYFilename("D:\\Temp\\PLY\\Compound_G_Filtered.ply");

                        OPERATOR(OperatorComparePointCloudUsingDistance);

                        EXECUTE_AND_VISUALIZE_RETURN();

                        OPERATOR(OperatorClustering)->SetSearchRadiusMultiplier(0.3f);

                        OPERATOR(OperatorFilterLeaveLargestOnly);


                        OPERATOR(OperatorComparePointCloudUsingDistance);

                        EXECUTE_AND_VISUALIZE_RETURN();
                    }
#pragma endregion

#pragma region Candidate
                    {
                        OPERATOR(OperatorStorePointCloud);

 /*                       OPERATOR(OperatorCurvatureDeviation)->
                            SetNeighborSearchOffset(1)->
                            SetSearchRadiusMultiplier(0.3333333f)->
                            SetDeviationThreshold(0.0125f);

                        EXECUTE_AND_VISUALIZE_RETURN();*/

                        for (size_t i = 0; i < 1; i++)
                        {
                            OPERATOR(OperatorCurvatureDeviation)->
                            SetNeighborSearchOffset(1)->
                            SetSearchRadiusMultiplier(0.3333333f)->
                            SetDeviationThreshold(0.0125f);

                            OPERATOR(OperatorLocalPlaneFitting)->
								SetNeighborSearchOffset(1)->
                                //SetSearchRadiusMultiplier(0.333333f)->
                                SetMarkedPointsOnly(true)->
                                SetUpdatePositions(true);

                            OPERATOR(OperatorClustering)->SetUseMarksForClustering(false);

                            OPERATOR(OperatorFilterLeaveLargestOnly);

							OPERATOR(OperatorPointCloudSaver)->SetPLYFilename("D:\\Temp\\PLY\\Compound_E_Filtered.ply");

                            OPERATOR(OperatorComparePointCloudUsingDistance);

                            //OPERATOR(OperatorSOR)->SetStdDevMultiplier(2.0f)->SetKNeighbors(15);

                            //OPERATOR(OperatorLocalPlaneFitting)->
                            //    SetNeighborSearchOffset(3)->
                            //    SetSearchRadiusMultiplier(5.0f)->
                            //    SetMarkedPointsOnly(true)->
                            //    SetUpdatePositions(true);

                            //OPERATOR(OperatorSOR)->SetStdDevMultiplier(2.0f)->SetKNeighbors(10);
                        }

                        //OPERATOR(OperatorCurvatureDeviation)->
                        //    SetNeighborSearchOffset(1)->
                        //    SetSearchRadiusMultiplier(0.3333333f)->
                        //    SetDeviationThreshold(0.0125f);

                        EXECUTE_AND_VISUALIZE_RETURN();
                    }
#pragma endregion


#pragma region Candidate
                    {
                        OPERATOR(OperatorCurvatureDeviation)->
                            SetNeighborSearchOffset(1)->
                            SetSearchRadiusMultiplier(0.3333333f)->
                            SetVisualizationSigma(5.0f);
					}
#pragma endregion

#pragma region Candidate
                    {
                        OPERATOR(OperatorStorePointCloud);

                        //OPERATOR(OperatorFilterETC);

                        OPERATOR(OperatorClustering);

                        OPERATOR(OperatorCurvatureEstimationAppliedNormal);

                        OPERATOR(OperatorNormalDivergence);

                        OPERATOR(OperatorShowMarks);

                        EXECUTE_AND_VISUALIZE_RETURN();
                    }
#pragma endregion

                    EXECUTE_AND_VISUALIZE_RETURN();
                }).detach();
        });

    Feather.Run();
    Feather.Terminate();
    return 0;
}
