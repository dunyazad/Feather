#include <robin_hood.h>

#include <libFeather.h>

#include "GeometricProcessingPipeline.hpp"

static inline std::string FormatWithCommas(size_t value)
{
    std::string numStr = std::to_string(value);
    int insertPosition = static_cast<int>(numStr.length()) - 3;
    while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
    return numStr;
}

using VD = VisualDebugging;

namespace GPP = GeometricProcessingPipeline;

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Final Optimized" << std::endl;
    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

    GeometricProcessingPipeline::Pipeline pipeline;
    GeometricProcessingPipeline::PointCloud pc;

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
                VD::Clear("PickedPoint");
                VD::Clear("PickedCell");

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

                glm::vec3 rayOrigin = glm::vec3(worldPosNear);
                glm::vec3 rayDir = glm::normalize(glm::vec3(worldPosFar - worldPosNear));

                Ray ray{ rayOrigin, rayDir };

                auto result = pipeline.GetSparseGrid()->Pick(pc.positions, ray, GeometricProcessingPipeline::Configuration::pointVisualizationRadius);

                if (result.hasHit)
                {
                    glm::vec3 p = pc.positions[result.pointIndex];
                    VD::AddSphere("PickedPoint", p, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

                    if (Feather.IsKeyPressed(GLFW_KEY_LEFT_CONTROL) || Feather.IsKeyPressed(GLFW_KEY_RIGHT_CONTROL))
                    {
                        manipulator->SetCenter(p);
                    }

                    glm::vec3 cellMin = pipeline.GetSparseGrid()->aabb.min + glm::vec3(
                        (float)result.gx * pipeline.GetSparseGrid()->cellSize,
                        (float)result.gy * pipeline.GetSparseGrid()->cellSize,
                        (float)result.gz * pipeline.GetSparseGrid()->cellSize
                    );
                    glm::vec3 cellMax = cellMin + glm::vec3(pipeline.GetSparseGrid()->cellSize);
                    VD::AddWiredBox("PickedCell", { cellMin, cellMax }, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

                    alog("Hit! Idx:%d, Cell(%d,%d,%d), Dist:%.2f\n", result.pointIndex, result.gx, result.gy, result.gz, result.distance);
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
            {
                TS(PLYLoading);
                PLYFormat ply;
                if (!ply.Deserialize("D:\\Debug\\PLY\\Compound_A.ply"))
                {
                    printf("Failed to load PLY.\n");
                    return;
                }
                pc.FromPLY(ply);
                TE(PLYLoading);
            }

            pipeline.BuildSparseGrid(pc);

            pipeline.AddOperator<GPP::OperatorFilterETC>("OperatorFilterETC", true);

            {
                auto operatorCurvatureEstimation = pipeline.AddOperator<GPP::OperatorCurvatureEstimation>("OperatorCurvatureEstimation", false);
                operatorCurvatureEstimation->SetNeighborSearchOffset(3);
                operatorCurvatureEstimation->SetSearchRadiusScale(5.0f);
                operatorCurvatureEstimation->SetVisualizationScale(5.0f);
            }

            {
                auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
                operatorClustering->SetUseMarksForClustering(true);
            }

            {
                auto operatorClusterBorderFinding = pipeline.AddOperator<GPP::OperatorClusterBorderFinding>("OperatorClusterBorderFinding", false);
            }

            {
                auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
                operatorClustering->SetUseMarksForClustering(true);
            }

            {
                struct FilterFunctor
                {
                    GPP::OperatorCustomFilter<FilterFunctor>* filter = nullptr;

                    bool operator()(GPP::PointCloud& pointCloud, size_t index)
                    {
                        if (0 < pointCloud.pointClusterIDs[index])
                        {
                            pointCloud.colors[index] = { 1.0f, 0.0f, 0.0f };
                            return false;
                        }
                        else
                        {
                            return true;
                        }
                    }
                };

                FilterFunctor filterFunctor;
                //pipeline.BuildSparseGrid(pc);
                auto operatorCustomFilter = pipeline.AddOperator<GPP::OperatorCustomFilter<FilterFunctor>>("OperatorCustomFilter", true);
            }

            {
				auto operatorMeshGeneration = pipeline.AddOperator<GPP::OperatorMeshGeneration>("OperatorMeshGeneration", false);
                operatorMeshGeneration->SetMeshVoxelSize(0.3f);

				//operatorMeshGeneration->ExportPLY("D:\\Debug\\PLY\\Compound_A_MeshGeneration_Output.ply");
            }

            {
				auto operatorPointCloudLoader = pipeline.AddOperator<GPP::OperatorPointCloudLoader>("OperatorPointCloudLoader", true);
                operatorPointCloudLoader->SetPLYFilename("D:\\Debug\\PLY\\Compound_A.ply");
            }

            {
				//std::vector<GPP::Triangle> triangles;
    //            PLYFormat ply;
				//ply.Deserialize("D:\\Debug\\PLY\\Compound_A_MeshGeneration_Output.ply");

    //            for (size_t i = 0; i < ply.GetTriangleIndices().size() / 3; i++)
    //            {
				//	auto i0 = ply.GetTriangleIndices()[i * 3 + 0];
				//	auto i1 = ply.GetTriangleIndices()[i * 3 + 1];
				//	auto i2 = ply.GetTriangleIndices()[i * 3 + 2];

				//	auto v0 = glm::vec3(ply.GetPoints()[i0 * 3 + 0], ply.GetPoints()[i0 * 3 + 1], ply.GetPoints()[i0 * 3 + 2]);
				//	auto v1 = glm::vec3(ply.GetPoints()[i1 * 3 + 0], ply.GetPoints()[i1 * 3 + 1], ply.GetPoints()[i1 * 3 + 2]);
				//	auto v2 = glm::vec3(ply.GetPoints()[i2 * 3 + 0], ply.GetPoints()[i2 * 3 + 1], ply.GetPoints()[i2 * 3 + 2]);

				//	triangles.push_back({ v0, v1, v2 });
    //            }

				auto operatorMeshDistanceFilter = pipeline.AddOperator<GPP::OperatorMeshDistanceFilter>("OperatorMeshDistanceFilter", false);
				//operatorMeshDistanceFilter->SetReferenceMesh(triangles);
                operatorMeshDistanceFilter->SetThresholdMultiplier(3.0f);
            }

            {
				auto operatorNormalDivergence = pipeline.AddOperator<GPP::OperatorCurvatureDivergence>("OperatorCurvatureDivergence", false);
            }

            {
                struct FilterFunctor
                {
                    GPP::OperatorCustomFilter<FilterFunctor>* filter = nullptr;

                    bool operator()(GPP::PointCloud& pointCloud, size_t index)
                    {
                        if (0 < pointCloud.pointClusterIDs[index])
                        {
                            pointCloud.colors[index] = { 1.0f, 0.0f, 0.0f };
                            return false;
                        }
                        else
                        {
                            return true;
                        }
                    }
                };
                FilterFunctor filterFunctor;

				auto operatorCustomFilter = pipeline.AddOperator<GPP::OperatorCustomFilter<FilterFunctor>>("OperatorCustomFilter", false);
            }

            {
                auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
                operatorClustering->SetUseMarksForClustering(true);
            }

            //{
            //    auto operatorCurvatureEstimation = pipeline.AddOperator<GPP::OperatorCurvatureEstimation>("OperatorCurvatureEstimation", false);
            //    operatorCurvatureEstimation->SetNeighborSearchOffset(3);
            //    operatorCurvatureEstimation->SetSearchRadiusScale(5.0f);
            //    operatorCurvatureEstimation->SetVisualizationScale(5.0f);
            //}

            //{
            //    auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
            //    operatorClustering->SetUseMarksForClustering(true);
            //}

       //     {
       //         struct FilterFunctor
       //         {
       //             GPP::OperatorCustomFilter<FilterFunctor>* filter = nullptr;

       //             bool operator()(GPP::PointCloud& pointCloud, size_t index)
       //             {
       //                 
       //                 //if (0 < pointCloud.pointClusterIDs[index])
       //                 //{
       //                 //    pointCloud.colors[index] = { 1.0f, 0.0f, 0.0f };
       //                 //    return true;
       //                 //}

       //                 if(1 == pointCloud.marks[index])
							//return false;

       //                 return true;
       //             }
       //         };

       //         FilterFunctor filterFunctor;
       //         //pipeline.BuildSparseGrid(pc);
       //         auto operatorCustomFilter = pipeline.AddOperator<GPP::OperatorCustomFilter<FilterFunctor>>("OperatorCustomFilter", true);
       //     }

       //     {
       //         auto operatorMeshGeneration = pipeline.AddOperator<GPP::OperatorMeshGeneration>("OperatorMeshGeneration", false);
       //         operatorMeshGeneration->SetMeshVoxelSize(0.15f);

       //         operatorMeshGeneration->ExportPLY("D:\\Debug\\PLY\\Compound_A_Result.ply");
       //     }

			pipeline.Execute(pc);

            pipeline.VisualizeLast();
            //operatorCustomFilter->Visualize();

#if 0
            pipeline.BuildSparseGrid(pc);

            //pipeline.AddOperator<GPP::OperatorFilterETC>("OperatorFilterETC", true);

            auto operatorCurvatureEstimation = pipeline.AddOperator<GPP::OperatorCurvatureEstimation>("OperatorCurvatureEstimation", false);
            operatorCurvatureEstimation->SetNeighborSearchOffset(3);
            operatorCurvatureEstimation->SetSearchRadiusScale(5.0f);
            operatorCurvatureEstimation->SetVisualizationScale(5.0f);

            auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
            operatorClustering->SetUseMarksForClustering(true);

            auto operatorClusterBorderFinding = pipeline.AddOperator<GPP::OperatorClusterBorderFinding>("OperatorClusterBorderFinding", false);

            pipeline.Execute(pc);

            //pipeline.VisualizeAll();
            //operatorCurvatureEstimation->Visualize();
            operatorClustering->Visualize();
            operatorClusterBorderFinding->Visualize();
#endif // 0


            return;
#if 0
            //pipeline.AddOperator<GPP::OperatorFilterETC>("OperatorFilterETC", true);
//pipeline.AddOperator<GPP::OperatorCurvatureDivergence>("OperatorCurvatureDivergence", false);
            auto operatorCurvatureEstimation = pipeline.AddOperator<GPP::OperatorCurvatureEstimation>("OperatorCurvatureEstimation", false);
            operatorCurvatureEstimation->SetNeighborSearchOffset(3);
            operatorCurvatureEstimation->SetSearchRadiusScale(5.0f);
            operatorCurvatureEstimation->SetVisualizationScale(5.0f);

            auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
            operatorClustering->SetUseMarksForClustering(true);

            pipeline.Execute(pc);

            //pipeline.VisualizeAll();
            //operatorCurvatureEstimation->Visualize();
            operatorClustering->Visualize();

            /*GPP::OperatorClustering operatorClustering;
            {
                TS(Clustering);
                operatorClustering.Process(pc, &sgrid);
                TE(Clustering);

                TS(Visualizing);
                operatorClustering.Visualize();
                TE(Visualizing);
            }*/

            /*GPP::OperatorCurvatureEstimation operatorCurvatureEstimation;
            {
                TS(Clustering);
                operatorCurvatureEstimation.Process(pc, &sgrid);
                TE(Clustering);

                TS(Visualizing);
                operatorCurvatureEstimation.Visualize();
                TE(Visualizing);
            }*/

            /*GPP::OperatorClusteringComplex operatorClusteringComplex;
            {
                TS(ClusteringComplex);
                operatorClusteringComplex.Process(pc, &sgrid);
                TE(ClusteringComplex);

                TS(VisualizingComplex);
                operatorClusteringComplex.Visualize();
                TE(VisualizingComplex);
            }*/
            return;
#endif // 0
        });

    Feather.Run();
    Feather.Terminate();
    return 0;
}
