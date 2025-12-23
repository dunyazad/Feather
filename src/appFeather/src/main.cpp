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

#define OPERATOR(operatorName) pipeline.AddOperator<GPP::operatorName>(#operatorName);
#define OPERATOR_PARAMETER(operatorName, parameter) pipeline.AddOperator<GPP::operatorName>(#operatorName, parameter);
#define EXECUTE_AND_VISUALIZE_RETURN() { pipeline.Execute(); pipeline.VisualizeLast(); return; }

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
                std::ifstream in("camera_state.txt");
                if (in.is_open())
                {
                    glm::vec3 eye, target, up;
                    in >> eye.x >> eye.y >> eye.z;
                    in >> target.x >> target.y >> target.z;
                    in >> up.x >> up.y >> up.z;

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

                            std::cout << "[System] Camera state RESTORED." << std::endl;
                        }
                    }
                }
                else
                {
                    std::cout << "[System] No saved camera state file found." << std::endl;
                }
            }
            else if (GLFW_KEY_F12 == event.keyCode && event.action == 0)
            {
                auto camEnt = Feather.GetEntityByName("Camera");
                if (camEnt != entt::null)
                {
                    auto cam = Feather.GetComponent<Camera>(camEnt);
                    if (cam)
                    {
                        std::ofstream out("camera_state.txt");
                        if (out.is_open())
                        {
                            glm::vec3 eye = cam->GetEye();
                            glm::vec3 target = cam->GetTarget();
                            glm::vec3 up = cam->GetUp();

                            // Eye, Target, Up 순서로 저장
                            out << eye.x << " " << eye.y << " " << eye.z << std::endl;
                            out << target.x << " " << target.y << " " << target.z << std::endl;
                            out << up.x << " " << up.y << " " << up.z << std::endl;

                            std::cout << "[System] Camera state SAVED to 'camera_state.txt'" << std::endl;
                            std::cout << "  Eye: " << eye.x << ", " << eye.y << ", " << eye.z << std::endl;
                        }
                        else
                        {
                            std::cout << "[Error] Failed to open file for saving." << std::endl;
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
                    VD::AddSphere("PickedPoint", p, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));

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
            std::thread([&]()
                {
                    // OperatorPointCloudLoader
                    {
                        GPP::GeometricProcessingOperatorParameter parameter;
                        parameter.SetParameter<std::string>("plyFilename", "D:\\Temp\\PLY\\Compound_C.ply");
                        OPERATOR_PARAMETER(OperatorPointCloudLoader, parameter);
                    }

                    OPERATOR(OperatorStorePointCloud);

                    OPERATOR(OperatorClustering);

                    OPERATOR(OperatorFilterLeaveLargestOnly);

                    OPERATOR(OperatorMeanShift);

                    OPERATOR(OperatorMeshGeneration);

                    EXECUTE_AND_VISUALIZE_RETURN();

                    OPERATOR(OperatorNormalDeviation);

                    OPERATOR(OperatorClustering);

                    //OPERATOR(OperatorNormalDivergence);

                    //OPERATOR(OperatorMeanShift);

                    OPERATOR(OperatorClustering);


                    OPERATOR(OperatorCurvatureEstimationAppliedNormal);

                    OPERATOR(OperatorNormalDivergence);

                    //OPERATOR(OperatorFilterUnmarked);

                    OPERATOR(OperatorClustering);

					EXECUTE_AND_VISUALIZE_RETURN();

                    //OPERATOR(OperatorFilterETC);

                    {
                        GPP::GeometricProcessingOperatorParameter parameter;
                        parameter.needToRebuildSpatialPartitioning = true;
                        OPERATOR_PARAMETER(OperatorNormalDivergence, parameter);
                    }

                    OPERATOR(OperatorPointCloudDensity);

                    EXECUTE_AND_VISUALIZE_RETURN();

                    {
                        GPP::GeometricProcessingOperatorParameter parameter;
                        parameter.SetParameter<std::string>("targetMarkName", "OperatorNormalDivergence");
                        OPERATOR_PARAMETER(OperatorExpandMarks, parameter);
                    }// EXECUTE_AND_VISUALIZE_RETURN();

                    EXECUTE_AND_VISUALIZE_RETURN();

                    //OPERATOR(OperatorCurvatureEstimation);

                    //OPERATOR(OperatorPointCloudDensity);

                    EXECUTE_AND_VISUALIZE_RETURN();

                }).detach();
        });

    Feather.Run();
    Feather.Terminate();
    return 0;
}
