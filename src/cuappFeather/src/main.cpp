#include <iostream>

#include "main.cuh"

#include <iostream>
using namespace std;

#include <libFeather.h>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

int main(int argc, char** argv)
{
	TestCUDA();

	cout << "AppFeather" << endl;

	Feather.Initialize(1920, 1080);

	GLuint VAO, VBO;

	auto w = Feather.GetFeatherWindow();

	struct Point
	{
		MiniMath::V3 position;
		MiniMath::V3 normal;
		MiniMath::V3 color;
	};
	ALPFormat<Point> alp;

	Feather.AddOnInitializeCallback([&]() {
		{
			auto appMain = Feather.CreateInstance<Entity>("AppMain");
			auto appMainEventReceiver = Feather.CreateInstance<ComponentBase>();
			appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_ESCAPE == event.keyEvent.keyCode)
				{
					glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
				}
				});
		}
		{
			auto camera = Feather.CreateInstance<Entity>("Camera");
			auto perspectiveCamera = Feather.CreateInstance<PerspectiveCamera>();

			//auto cameraManipulator = Feather.CreateInstance<CameraManipulatorOrbit>();
			//cameraManipulator->SetCamera(perspectiveCamera);

			auto cameraManipulator = Feather.CreateInstance<CameraManipulatorTrackball>();
			cameraManipulator->SetCamera(perspectiveCamera);
		}

		{
			auto gui = Feather.CreateInstance<Entity>("GUI");
			auto statusPanel = Feather.CreateInstance<StatusPanel>();
		}

		//#define RENDER_TRIANGLE
#ifdef RENDER_TRIANGLE
		{
			auto entity = Feather.CreateInstance<Entity>("Triangle");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			renderable->AddVertex({ -1.0f, -1.0f, 0.0f });
			renderable->AddVertex({ 1.0f, -1.0f, 0.0f });
			renderable->AddVertex({ 0.0f, 1.0f, 0.0f });

			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });

			renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
			renderable->AddColor({ 0.0f, 1.0f, 0.0f, 1.0f });
			renderable->AddColor({ 0.0f, 0.0f, 1.0f, 1.0f });

			renderable->AddIndex(0);
			renderable->AddIndex(1);
			renderable->AddIndex(2);
		}
#endif // RENDER_TRIANGLE

		//#define RENDER_VOXELS_BOX
#ifdef RENDER_VOXELS_BOX
		{
			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "one");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			int xCount = 100;
			int yCount = 100;
			int zCount = 100;
			int tCount = xCount * yCount * zCount;

			for (int i = 0; i < tCount; i++)
			{
				int z = i / (xCount * yCount);
				int y = (i % (xCount * yCount)) / xCount;
				int x = (i % (xCount * yCount)) % xCount;

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);
		}
#endif // RENDER_VOXELS_BOX

		//#define RENDER_VOXELS_SPHERE
#ifdef RENDER_VOXELS_SPHERE
		{
			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere("zero", 0.25f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			int xCount = 100;
			int yCount = 100;
			int zCount = 100;
			int tCount = xCount * yCount * zCount;

			for (int i = 0; i < tCount; i++)
			{
				int z = i / (xCount * yCount);
				int y = (i % (xCount * yCount)) / xCount;
				int x = (i % (xCount * yCount)) % xCount;

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);

			renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_M == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->NextDrawingMode();
				}
				});
		}
#endif // RENDER_VOXELS_BOX

		//#define LOAD_PLY
#ifdef LOAD_PLY
		{
			auto entity = Feather.CreateInstance<Entity>("Teeth");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs"));

			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Points);
			renderable->SetShader(shader);

			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth.ply");
			ply.SwapAxisYZ();

			if (false == ply.GetPoints().empty())
				renderable->AddVertices((MiniMath::V3*)ply.GetPoints().data(), ply.GetPoints().size() / 3);
			if (false == ply.GetNormals().empty())
				renderable->AddNormals((MiniMath::V3*)ply.GetNormals().data(), ply.GetNormals().size() / 3);
			if (false == ply.GetColors().empty())
			{
				if (ply.UseAlpha())
				{
					renderable->AddColors((MiniMath::V4*)ply.GetColors().data(), ply.GetColors().size() / 4);
				}
				else
				{
					renderable->AddColors((MiniMath::V3*)ply.GetColors().data(), ply.GetColors().size() / 3);
				}
			}

			auto cameraManipulator = Feather.GetFirstInstance<CameraManipulatorTrackball>();
			auto camera = cameraManipulator->SetCamera();
			auto [x, y, z] = ply.GetAABBCenter();
			camera->SetEye({ x,y,z + cameraManipulator->GetRadius() });
			camera->SetTarget({ x,y,z });
		}
#endif // LOAD_PLY

		/*
		{
			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth.ply");
			ply.SwapAxisYZ();

			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere("zero", 0.05f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			ui32 tCount = ply.GetPoints().size() / 3;

			for (int i = 0; i < tCount; i++)
			{
				auto x = ply.GetPoints()[i * 3];
				auto y = ply.GetPoints()[i * 3 + 1];
				auto z = ply.GetPoints()[i * 3 + 2];

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);

			renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_M == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->NextDrawingMode();
				}
				});
		}
		*/

#pragma region Load PLY and Convert to ALP format
		{
			auto t = Time::Now();

			if (false == alp.Deserialize("../../res/3D/Compound_Partial.alp"))
			{
				PLYFormat ply;
				ply.Deserialize("../../res/3D/Compound_Partial.ply");
				//ply.SwapAxisYZ();

				vector<Point> points;
				for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
				{
					auto px = ply.GetPoints()[i * 3];
					auto py = ply.GetPoints()[i * 3 + 1];
					auto pz = ply.GetPoints()[i * 3 + 2];

					auto nx = ply.GetNormals()[i * 3];
					auto ny = ply.GetNormals()[i * 3 + 1];
					auto nz = ply.GetNormals()[i * 3 + 2];

					if (false == ply.GetColors().empty())
					{
						auto cx = ply.GetColors()[i * 3];
						auto cy = ply.GetColors()[i * 3 + 1];
						auto cz = ply.GetColors()[i * 3 + 2];

						points.push_back({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
					}
					else
					{
						points.push_back({ {px, py, pz}, {nx, ny, nz}, {1.0f, 1.0f, 1.0f} });
					}

				}

				alog("PLY %d points loaded\n", points.size());

				alp.AddPoints(points);
				alp.Serialize("../../res/3D/Compound_Partial.alp");
			}

			t = Time::End(t, "Loading Teeth");

			auto entity = Feather.CreateInstance<Entity>("Box");
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			{
				auto shader = Feather.CreateInstance<Shader>();
				shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
				renderable->AddShader(shader);
			}
			{
				auto shader = Feather.CreateInstance<Shader>();
				shader->Initialize(File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs"));
				renderable->AddShader(shader);
				renderable->SetActiveShaderIndex(1);
			}

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere("zero", 0.05f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			//PLYFormat tempPLY;

			vector<float3> host_points;

			for (auto& p : alp.GetPoints())
			{
				auto r = p.color.x;
				auto g = p.color.y;
				auto b = p.color.z;
				auto a = 1.f;

				renderable->AddInstanceColor(MiniMath::V4(r, g, b, a));
				renderable->AddInstanceNormal(p.normal);

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 1.5f;
				model.m[1][1] = 1.5f;
				model.m[2][2] = 1.5f;
				model = MiniMath::translate(model, p.position);
				renderable->AddInstanceTransform(model);

				//tempPLY.AddPoint(p.position.x, p.position.y, p.position.z);
				//tempPLY.AddNormal(p.normal.x, p.normal.y, p.normal.z);
				//tempPLY.AddColor(p.color.x, p.color.y, p.color.z);

				host_points.push_back(make_float3(p.position.x, p.position.y, p.position.z));
			}
			//tempPLY.Serialize("../../res/3D/Teeth_temp.ply");

			alog("ALP %d points loaded\n", alp.GetPoints().size());

			renderable->EnableInstancing(alp.GetPoints().size());

			renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_M == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->NextDrawingMode();
				}
				else if (GLFW_KEY_1 == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->SetActiveShaderIndex(0);
				}
				else if (GLFW_KEY_2 == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->SetActiveShaderIndex(1);
				}
				else if (GLFW_KEY_R == event.keyEvent.keyCode)
				{
					auto cameraManipulator = Feather.GetFirstInstance<CameraManipulatorTrackball>();
					auto camera = cameraManipulator->SetCamera();
					auto [x, y, z] = alp.GetAABBCenter();
					camera->SetEye({ x,y,z + cameraManipulator->GetRadius() });
					camera->SetTarget({ x,y,z });
				}
				});
			
			t = Time::End(t, "Upload to GPU");

			auto hashToFloat = [](uint32_t seed) -> float {
				seed ^= seed >> 13;
				seed *= 0x5bd1e995;
				seed ^= seed >> 15;
				return (seed & 0xFFFFFF) / static_cast<float>(0xFFFFFF);
			};

			auto [x, y, z] = alp.GetAABBCenter();
			auto pointIndices = cuMain(host_points, make_float3(x,y,z));
			for (size_t i = 0; i < pointIndices.size(); i++)
			{
				auto index = pointIndices[i];
				if (index != -1)
				{
					float r = hashToFloat(index * 3 + 0);
					float g = hashToFloat(index * 3 + 1);
					float b = hashToFloat(index * 3 + 2);

					//if (index == 0)
					//{
					//	renderable->SetInstanceColor(i, MiniMath::V4(1.0f, 0.0f, 0.0f, 1.0f));
					//}
					//else
					//{
					renderable->SetInstanceColor(i, MiniMath::V4(r, g, b, 1.0f));
					//}
				}
			}

			/*
			{ // AABB
				auto m = alp.GetAABBMin();
				float x = get<0>(m);
				float y = get<1>(m);
				float z = get<2>(m);
				auto M = alp.GetAABBMax();
				float X = get<0>(M);
				float Y = get<1>(M);
				float Z = get<2>(M);

				auto entity = Feather.CreateInstance<Entity>("AABB");
				auto shader = Feather.CreateInstance<Shader>();
				shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
				auto renderable = Feather.CreateInstance<Renderable>();
				renderable->Initialize(Renderable::GeometryMode::Lines);
				renderable->AddShader(shader);

				renderable->AddVertex({ x, y, z }); renderable->AddColor({1.0f, 0.0f, 0.0f, 1.0f});
				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });


				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });


				renderable->AddVertex({ x, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
			}
			*/

			{ // Cache Area
				auto [cx, cy, cz] = alp.GetAABBCenter();

				float x = cx + (-10.0f);
				float y = cy + (-15.0f);
				float z = cz + (-20.0f);
				float X = cx + (10.0f);
				float Y = cy + (15.0f);
				float Z = cz + (20.0f);

				auto entity = Feather.CreateInstance<Entity>("AABB");
				auto shader = Feather.CreateInstance<Shader>();
				shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
				auto renderable = Feather.CreateInstance<Renderable>();
				renderable->Initialize(Renderable::GeometryMode::Lines);
				renderable->AddShader(shader);

				renderable->AddVertex({ x, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });


				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });

				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });


				renderable->AddVertex({ x, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ X, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
				renderable->AddVertex({ x, Y, Z }); renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
			}
		}
#pragma endregion
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
