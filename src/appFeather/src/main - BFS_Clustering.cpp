#include <libFeather.h>
#include <queue>
#include <unordered_map>

using VD = VisualDebugging;
using namespace libRxTx;

struct VoxelData {
    bool occupied = false;
    glm::vec3 position = glm::vec3(0.0f);
};

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Fast Clustering" << std::endl;

    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);

    auto w = Feather.GetFeatherWindow();

#pragma region AppMain
    {
        auto appMain = Feather.CreateEntity("AppMain");
        Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
            if (GLFW_KEY_ESCAPE == event.keyCode)
            {
                glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
            }
            else if (GLFW_KEY_SPACE == event.keyCode)
            {
                if (event.action == 0)
                {
                    Feather.GetImmediateModeRenderSystem()->ToggleEnable();
                }
            }
            });
    }
#pragma endregion

#pragma region Camera
    {
        Entity cam = Feather.CreateEntity("Camera");
        auto pcam = Feather.CreateComponent<PerspectiveCamera>(cam);
        auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
        pcamMan->SetCamera(pcam);

        Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
            auto window = Feather.GetFeatherWindow();
            float aspect = (float)window->GetWidth() / (float)window->GetHeight();
            pcam->SetAspectRatio(aspect);
            });

        Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event);
            });
        Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event);
            });
        Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMouseButton(event);
            });
        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) {
            Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
            });
    }
#pragma endregion

    Feather.AddOnInitializeCallback([&]() {

        auto contrastingColors = Color::GetContrastingColors(1000);

        PLYFormat ply;
        ply.Deserialize("D:\\Debug\\PLY\\inputA.ply");

        size_t pointCount = ply.GetPoints().size() / 3;

        // compute AABB min
        auto [minx, miny, minz] = ply.GetAABBMin();

        float voxelSize = 0.1f;

        glm::vec3 gridOrigin;
        gridOrigin.x = std::floor(minx / voxelSize) * voxelSize;
        gridOrigin.y = std::floor(miny / voxelSize) * voxelSize;
        gridOrigin.z = std::floor(minz / voxelSize) * voxelSize;

        // voxel map
        std::unordered_map<uint64_t, VoxelData> volume;
        volume.reserve(pointCount);

        // voxelize
        for (size_t i = 0; i < pointCount; i++)
        {
            glm::vec3 p(
                ply.GetPoints()[i * 3 + 0],
                ply.GetPoints()[i * 3 + 1],
                ply.GetPoints()[i * 3 + 2]
            );

            uint64_t key = Morton3D::Encode(
                Morton3D::PositionToIndex(p, gridOrigin, voxelSize)
            );

            auto& v = volume[key];
            v.occupied = true;
            v.position = p;
        }

        printf("[Voxel] Total voxels: %zu\n", volume.size());

        // 26 neighbor offsets
        glm::ivec3 neighbor26[26];
        {
            int c = 0;
            for (int z = -1; z <= 1; z++)
                for (int y = -1; y <= 1; y++)
                    for (int x = -1; x <= 1; x++)
                    {
                        if (x == 0 && y == 0 && z == 0) continue;
                        neighbor26[c++] = glm::ivec3(x, y, z);
                    }
        }

        // BFS clustering
        std::unordered_map<uint64_t, int> clusterId;
        clusterId.reserve(volume.size());

        int cid = 0;

        TS(BFS_Clustering);
        for (auto& [key, voxel] : volume)
        {
            if (!voxel.occupied) continue;
            if (clusterId.count(key)) continue;

            std::queue<uint64_t> q;
            q.push(key);
            clusterId[key] = cid;

            while (!q.empty())
            {
                uint64_t k = q.front();
                q.pop();

                glm::ivec3 base = Morton3D::KeyToIndex(k);

                for (auto& off : neighbor26)
                {
                    glm::ivec3 nb = base + off;
                    uint64_t nk = Morton3D::Encode(nb);

                    auto it = volume.find(nk);
                    if (it == volume.end()) continue;
                    if (!it->second.occupied) continue;
                    if (clusterId.count(nk)) continue;

                    clusterId[nk] = cid;
                    q.push(nk);
                }
            }

            cid++;
        }
        TE(BFS_Clustering);

        printf("[Cluster] cluster count = %d\n", cid);

        // visualize clusters
        for (auto& [key, voxel] : volume)
        {
            if (!voxel.occupied) continue;

            int id = clusterId[key];
            auto col = contrastingColors[id % contrastingColors.size()];

            VD::AddWiredBox("cluster", voxel.position, glm::vec3(voxelSize), col);
        }

#pragma region Status Panel
        {
            auto gui = Feather.GetRegistry().create();
            auto& statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);

            Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event) {
                auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
                component.mouseX = event.xpos;
                component.mouseY = event.ypos;
                });
        }
#pragma endregion
        });

    Feather.Run();
    Feather.Terminate();

    return 0;
}
