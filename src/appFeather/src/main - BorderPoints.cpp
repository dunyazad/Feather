#include <libFeather.h>
#include <queue>
#include <algorithm> // for std::sort, std::nth_element

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;

struct VoxelData {
    bool occupied = false;
    bool isEdge = false;
    int neighborCount = 0;
    glm::vec3 position = glm::vec3(0.0f);
    glm::vec3 normal = glm::vec3(0.0f);
    glm::vec3 color = glm::vec3(0.0f);
};

struct HoleInfo {
    std::vector<uint64_t> borderVoxels;
    glm::vec3 centroid = glm::vec3(0.0f);
    float area = 0.0f;
};

// --- KD-Tree Implementation Start ---
struct KDPoint {
    glm::vec3 position;
    size_t originalIndex;
};

struct KDNode {
    KDPoint point;
    KDNode* left = nullptr;
    KDNode* right = nullptr;
    int axis;

    KDNode(const KDPoint& pt, int ax) : point(pt), axis(ax), left(nullptr), right(nullptr) {}
};

class SimpleKDTree {
public:
    SimpleKDTree() : root(nullptr) {}
    ~SimpleKDTree() { Clear(root); }

    void Build(const std::vector<glm::vec3>& rawPoints) {
        Clear(root);
        points.clear();
        points.reserve(rawPoints.size());

        for (size_t i = 0; i < rawPoints.size(); ++i) {
            points.push_back({ rawPoints[i], i });
        }

        root = BuildRecursive(points.begin(), points.end(), 0);
        printf("[KDTree] Built with %zu points.\n", points.size());
    }

    void Search(const glm::vec3& query, int k, std::vector<size_t>& outIndices, std::vector<float>& outDistSq) {
        if (!root || k <= 0) return;

        // Max-heap to store the k nearest neighbors found so far.
        // Pair: <Squared Distance, Original Index>
        std::priority_queue<std::pair<float, size_t>> pq;

        SearchRecursive(root, query, k, pq);

        // Extract results from PQ
        outIndices.resize(pq.size());
        outDistSq.resize(pq.size());

        // PQ pops the largest element first, so we fill from back to front
        for (int i = static_cast<int>(pq.size()) - 1; i >= 0; --i) {
            outIndices[i] = pq.top().second;
            outDistSq[i] = pq.top().first;
            pq.pop();
        }
    }

    void SearchRadius(const glm::vec3& query, float radius, std::vector<size_t>& outIndices) {
        if (!root || radius <= 0.0f) return;

        outIndices.clear();
        float radiusSq = radius * radius;
        SearchRadiusRecursive(root, query, radiusSq, outIndices);
    }

private:
    KDNode* root;
    std::vector<KDPoint> points;

    void Clear(KDNode* node) {
        if (!node) return;
        Clear(node->left);
        Clear(node->right);
        delete node;
    }

    KDNode* BuildRecursive(std::vector<KDPoint>::iterator start, std::vector<KDPoint>::iterator end, int depth) {
        if (start >= end) return nullptr;

        int axis = depth % 3;
        size_t len = std::distance(start, end);
        auto mid = start + len / 2;

        std::nth_element(start, mid, end, [axis](const KDPoint& a, const KDPoint& b) {
            return a.position[axis] < b.position[axis];
            });

        KDNode* node = new KDNode(*mid, axis);
        node->left = BuildRecursive(start, mid, depth + 1);
        node->right = BuildRecursive(mid + 1, end, depth + 1);

        return node;
    }

    void SearchRecursive(KDNode* node, const glm::vec3& query, int k, std::priority_queue<std::pair<float, size_t>>& pq) {
        if (!node) return;

        // Calculate squared distance between query and current node
        float dx = query.x - node->point.position.x;
        float dy = query.y - node->point.position.y;
        float dz = query.z - node->point.position.z;
        float distSq = dx * dx + dy * dy + dz * dz;

        // Update priority queue
        if (pq.size() < k) {
            pq.push({ distSq, node->point.originalIndex });
        }
        else if (distSq < pq.top().first) {
            pq.pop();
            pq.push({ distSq, node->point.originalIndex });
        }

        // Determine which side to traverse first
        float diff = query[node->axis] - node->point.position[node->axis];
        KDNode* nearNode = diff < 0 ? node->left : node->right;
        KDNode* farNode = diff < 0 ? node->right : node->left;

        // Visit near side
        SearchRecursive(nearNode, query, k, pq);

        // Visit far side only if necessary
        // If we haven't found k items yet, OR if the plane distance is within our current search radius
        if (pq.size() < k || (diff * diff) < pq.top().first) {
            SearchRecursive(farNode, query, k, pq);
        }
    }

    void SearchRadiusRecursive(KDNode* node, const glm::vec3& query, float radiusSq, std::vector<size_t>& outIndices) {
        if (!node) return;

        // Calculate squared distance
        float dx = query.x - node->point.position.x;
        float dy = query.y - node->point.position.y;
        float dz = query.z - node->point.position.z;
        float distSq = dx * dx + dy * dy + dz * dz;

        // Collect point if within radius
        if (distSq <= radiusSq) {
            outIndices.push_back(node->point.originalIndex);
        }

        // Determine traversal order
        float diff = query[node->axis] - node->point.position[node->axis];
        float diffSq = diff * diff;

        KDNode* nearNode = diff < 0 ? node->left : node->right;
        KDNode* farNode = diff < 0 ? node->right : node->left;

        // Always search the near side
        SearchRadiusRecursive(nearNode, query, radiusSq, outIndices);

        // Search the far side only if the plane intersects the search radius
        if (diffSq <= radiusSq) {
            SearchRadiusRecursive(farNode, query, radiusSq, outIndices);
        }
    }
};
// --- KD-Tree Implementation End ---

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Hole Detection" << std::endl;

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
            auto aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
            pcam->SetAspectRatio(aspectRatio);
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
        auto contrastingColors = Color::GetContrastingColors(7);

        {
            PLYFormat ply;
            ply.Deserialize("D:\\Debug\\PLY\\inputA.ply");
            auto [minx, miny, minz] = ply.GetAABBMin();

            // --- 1. Generate KD-Tree ---
            // Convert flat float vector to vec3 vector for the KDTree
            std::vector<glm::vec3> pointCloud;
            size_t pointCount = ply.GetPoints().size() / 3;
            pointCloud.reserve(pointCount);

            for (size_t i = 0; i < pointCount; ++i)
            {
                float x = ply.GetPoints()[i * 3 + 0];
                float y = ply.GetPoints()[i * 3 + 1];
                float z = ply.GetPoints()[i * 3 + 2];

                if (-10.0f < x && x < 10.0f &&
                    -10.0f < y && y < 10.0f &&
                    -10.0f < z && z < 10.0f)
                {
                    pointCloud.emplace_back(x, y, z);
                }
            }

            TS(build_KDTree);
            SimpleKDTree kdtree;
            kdtree.Build(pointCloud);
            TE(build_KDTree);

			std::vector<size_t> numberOfNeighbors(pointCount);

            TS(KNN);
            for (auto& p : pointCloud)
            {
                std::vector<size_t> indices;
				kdtree.SearchRadius(p, 0.2f, indices);

                numberOfNeighbors.push_back(indices.size());
            }
            TE(KNN);

            for (auto& p : pointCloud)
            {
                std::vector<size_t> indices;
                kdtree.SearchRadius(p, 0.3f, indices);

                numberOfNeighbors.push_back(indices.size());

                size_t threshold = 25;
                if (threshold > indices.size())
                {
                    VD::AddSphere("points", p, 0.05f, Color::red());
                }
                else
                {
                    VD::AddSphere("points", p, 0.05f, Color::white());
                }
            }


#if 0
            float voxelSize = 0.1f;
            float blockSize = voxelSize * VpB;

            glm::vec3 gridOrigin;
            gridOrigin.x = std::floor(minx / blockSize) * blockSize;
            gridOrigin.y = std::floor(miny / blockSize) * blockSize;
            gridOrigin.z = std::floor(minz / blockSize) * blockSize;

            // Use a temp struct to accumulate data for averaging
            struct VoxelAccumulator {
                bool occupied = false;
                int pointCount = 0;
                glm::vec3 sumPosition = glm::vec3(0.0f);
                glm::vec3 sumNormal = glm::vec3(0.0f);
                glm::vec3 sumColor = glm::vec3(0.0f);
            };

            std::unordered_map<uint64_t, VoxelAccumulator> tempVolume;

            // 1. Voxelize & Accumulate
            for (size_t i = 0; i < pointCount; i++)
            {
                auto x = ply.GetPoints()[i * 3 + 0];
                auto y = ply.GetPoints()[i * 3 + 1];
                auto z = ply.GetPoints()[i * 3 + 2];

                auto nx = ply.GetNormals()[i * 3 + 0];
                auto ny = ply.GetNormals()[i * 3 + 1];
                auto nz = ply.GetNormals()[i * 3 + 2];

                auto cx = ply.GetColors()[i * 3 + 0];
                auto cy = ply.GetColors()[i * 3 + 1];
                auto cz = ply.GetColors()[i * 3 + 2];

                auto index = Morton3D::PositionToIndex({ x, y, z }, gridOrigin, voxelSize);
                auto key = Morton3D::Encode(index);

                auto& acc = tempVolume[key];
                acc.occupied = true;
                acc.pointCount++;
                acc.sumPosition += glm::vec3(x, y, z);
                acc.sumNormal += glm::vec3(nx, ny, nz);
                acc.sumColor += glm::vec3(cx, cy, cz);
            }

            std::unordered_map<uint64_t, VoxelData> volume;

            // 2. Average Data
            for (auto& [key, acc] : tempVolume)
            {
                VoxelData voxel;
                voxel.occupied = true;

                if (acc.pointCount > 0)
                {
                    float invCount = 1.0f / acc.pointCount;
                    voxel.position = acc.sumPosition * invCount;
                    voxel.normal = glm::normalize(acc.sumNormal * invCount);
                    voxel.color = acc.sumColor * invCount;
                }
                else
                {
                    auto index = Morton3D::KeyToIndex(key);
                    voxel.position = Morton3D::IndexToPosition({ index.x, index.y, index.z }, gridOrigin, voxelSize);
                }

                volume[key] = voxel;
            }

            printf("[Mesh] Total voxels: %zu\n", volume.size());

            // 26 Neighbors Offsets
            glm::ivec3 neighborOffsets[26];
            int count = 0;
            for (int z = -1; z <= 1; z++) {
                for (int y = -1; y <= 1; y++) {
                    for (int x = -1; x <= 1; x++) {
                        if (x == 0 && y == 0 && z == 0) continue;
                        neighborOffsets[count++] = glm::ivec3(x, y, z);
                    }
                }
            }

            // ----------------------------------------------------------
            // [Pass 1] Basic neighbor count and 1st pass edge detection
            // ----------------------------------------------------------
            for (auto& [key, voxel] : volume)
            {
                if (!voxel.occupied) continue;

                auto index = Morton3D::KeyToIndex(key);
                int count = 0;

                for (const auto& offset : neighborOffsets)
                {
                    auto neighborKey = Morton3D::Encode(index + offset);
                    if (volume.find(neighborKey) != volume.end()) {
                        count++;
                    }
                }
                voxel.neighborCount = count;

                // 1st Criteria: If neighbors <= 7, consider as edge
                if (voxel.neighborCount <= 7) {
                    voxel.isEdge = true;
                }
                else {
                    voxel.isEdge = false;
                }
            }

            // ----------------------------------------------------------
            // [Pass 2] Gap Filling
            // ----------------------------------------------------------
            std::vector<uint64_t> gapsToFill;

            for (auto& [key, voxel] : volume)
            {
                if (voxel.isEdge) continue;

                auto index = Morton3D::KeyToIndex(key);
                int edgeNeighbors = 0;

                for (const auto& offset : neighborOffsets)
                {
                    auto neighborKey = Morton3D::Encode(index + offset);
                    auto it = volume.find(neighborKey);
                    if (it != volume.end() && it->second.isEdge) {
                        edgeNeighbors++;
                    }
                }

                if (edgeNeighbors >= 2) {
                    gapsToFill.push_back(key);
                }
            }

            for (auto key : gapsToFill) {
                volume[key].isEdge = true;
            }

            printf("[Mesh] Edge voxels refined. Filled gaps: %zu\n", gapsToFill.size());

            // ----------------------------------------------------------
            // [Visualization]
            // ----------------------------------------------------------
            for (auto& [key, voxel] : volume)
            {
                if (!voxel.occupied) continue;

                if (voxel.isEdge)
                {
                    // Noise removal: do not draw isolated points
                    if (voxel.neighborCount <= 1) {
                        // VD::AddWiredBox("noise", voxel.position, glm::vec3(voxelSize), glm::vec3(0.5f)); 
                    }
                    else {
                        VD::AddWiredBox("voxel_edge", voxel.position, glm::vec3(voxelSize), Color::red());
                    }
                }
                else
                {
                    VD::AddWiredBox("voxel_inner", voxel.position, glm::vec3(voxelSize), Color::white());
                }
            }
#endif // 0

        }

#pragma region Status Panel
        {
            auto gui = Feather.GetRegistry().create();
            auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);

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
