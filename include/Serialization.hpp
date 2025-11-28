#pragma once

#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#define FLT_VALID(x) ((x) < 3.402823466e+36F)

inline int safe_stoi(const std::string& input)
{
	if (input.empty())
	{
		return INT_MAX;
	}
	else
	{
		return stoi(input);
	}
}

inline float safe_stof(const std::string& input)
{
	if (input.empty())
	{
		return FLT_MAX;
	}
	else
	{
		return stof(input);
	}
}

inline std::vector<std::string> split(const std::string& input, const std::string& delimiters, bool includeEmptyString = false)
{
	std::vector<std::string> result;
	std::string piece;
	for (auto c : input)
	{
		bool contains = false;
		for (auto d : delimiters)
		{
			if (d == c)
			{
				contains = true;
				break;
			}
		}

		if (contains == false)
		{
			piece += c;
		}
		else
		{
			if (includeEmptyString || piece.empty() == false)
			{
				result.push_back(piece);
				piece.clear();
			}
		}
	}
	if (piece.empty() == false)
	{
		result.push_back(piece);
	}

	return result;
}

inline void ParseOneLine(
	const std::string& line,
	std::vector<float>& vertices,
	std::vector<float>& uvs,
	std::vector<float>& vertex_normals,
	std::vector<float>& vertex_colors,
	std::vector<uint32_t>& faces,
	float scaleX, float scaleY, float scaleZ)
{
	if (line.empty())
		return;

	auto words = split(line, " \t");

	if (words[0] == "v")
	{
		float x = safe_stof(words[1]) * scaleX;
		float y = safe_stof(words[2]) * scaleY;
		float z = safe_stof(words[3]) * scaleZ;
		vertices.push_back(x);
		vertices.push_back(y);
		vertices.push_back(z);

		if (words.size() > 3)
		{
			float r = safe_stof(words[4]);
			float g = safe_stof(words[5]);
			float b = safe_stof(words[6]);
			vertex_colors.push_back(r);
			vertex_colors.push_back(g);
			vertex_colors.push_back(b);
		}

	}
	else if (words[0] == "vt")
	{
		float u = safe_stof(words[1]);
		float v = safe_stof(words[2]);
		uvs.push_back(u);
		uvs.push_back(v);
	}
	else if (words[0] == "vn")
	{
		float x = safe_stof(words[1]);
		float y = safe_stof(words[2]);
		float z = safe_stof(words[3]);
		vertex_normals.push_back(x);
		vertex_normals.push_back(y);
		vertex_normals.push_back(z);
	}
	else if (words[0] == "f")
	{
		if (words.size() == 4)
		{
			auto fe0 = split(words[1], "/", true);
			auto fe1 = split(words[2], "/", true);
			auto fe2 = split(words[3], "/", true);

			if (fe0.size() == 1 && fe1.size() == 1 && fe2.size() == 1) {
				faces.push_back(safe_stoi(fe0[0]));
				//faces.push_back(INT_MAX);
				//faces.push_back(INT_MAX);
				faces.push_back(safe_stoi(fe1[0]));
				//faces.push_back(INT_MAX);
				//faces.push_back(INT_MAX);
				faces.push_back(safe_stoi(fe2[0]));
				//faces.push_back(INT_MAX);
				//faces.push_back(INT_MAX);
			}
			else {
				faces.push_back(safe_stoi(fe0[0]));
				//faces.push_back(safe_stoi(fe0[1]));
				//faces.push_back(safe_stoi(fe0[2]));
				faces.push_back(safe_stoi(fe1[0]));
				//faces.push_back(safe_stoi(fe1[1]));
				//faces.push_back(safe_stoi(fe1[2]));
				faces.push_back(safe_stoi(fe2[0]));
				//faces.push_back(safe_stoi(fe2[1]));
				//faces.push_back(safe_stoi(fe2[2]));
			}
		}
	}
}

class HSerializable
{
public:
	virtual bool Serialize(const std::string& filename) = 0;
	virtual bool Deserialize(const std::string& filename) = 0;

	virtual inline void AddPoint(float x, float y, float z)
	{
		points.push_back(x);
		points.push_back(y);
		points.push_back(z);

		if (FLT_VALID(x) && FLT_VALID(y) && FLT_VALID(z))
		{
			aabbMinX = x < aabbMinX ? x : aabbMinX;
			aabbMinY = y < aabbMinY ? y : aabbMinY;
			aabbMinZ = z < aabbMinZ ? z : aabbMinZ;

			aabbMaxX = x > aabbMaxX ? x : aabbMaxX;
			aabbMaxY = y > aabbMaxY ? y : aabbMaxY;
			aabbMaxZ = z > aabbMaxZ ? z : aabbMaxZ;
		}
	}

	virtual inline void AddPoint(float x, float y, float z, float w)
	{
		points.push_back(x);
		points.push_back(y);
		points.push_back(z);
		points.push_back(w);

		if (FLT_VALID(x) && FLT_VALID(y) && FLT_VALID(z))
		{
			aabbMinX = x < aabbMinX ? x : aabbMinX;
			aabbMinY = y < aabbMinY ? y : aabbMinY;
			aabbMinZ = z < aabbMinZ ? z : aabbMinZ;

			aabbMaxX = x > aabbMaxX ? x : aabbMaxX;
			aabbMaxY = y > aabbMaxY ? y : aabbMaxY;
			aabbMaxZ = z > aabbMaxZ ? z : aabbMaxZ;
		}
	}

	virtual inline void AddPointFloat3(const float* point)
	{
		points.push_back(point[0]);
		points.push_back(point[1]);
		points.push_back(point[2]);

		if (FLT_VALID(point[0]) && FLT_VALID(point[1]) && FLT_VALID(point[2]))
		{
			aabbMinX = point[0] < aabbMinX ? point[0] : aabbMinX;
			aabbMinY = point[1] < aabbMinY ? point[1] : aabbMinY;
			aabbMinZ = point[2] < aabbMinZ ? point[2] : aabbMinZ;

			aabbMaxX = point[0] > aabbMaxX ? point[0] : aabbMaxX;
			aabbMaxY = point[1] > aabbMaxY ? point[1] : aabbMaxY;
			aabbMaxZ = point[2] > aabbMaxZ ? point[2] : aabbMaxZ;
		}
	}

	virtual inline void AddPointFloat4(const float* point)
	{
		points.push_back(point[0]);
		points.push_back(point[1]);
		points.push_back(point[2]);
		points.push_back(point[3]);

		if (FLT_VALID(point[0]) && FLT_VALID(point[1]) && FLT_VALID(point[2]))
		{
			aabbMinX = point[0] < aabbMinX ? point[0] : aabbMinX;
			aabbMinY = point[1] < aabbMinY ? point[1] : aabbMinY;
			aabbMinZ = point[2] < aabbMinZ ? point[2] : aabbMinZ;

			aabbMaxX = point[0] > aabbMaxX ? point[0] : aabbMaxX;
			aabbMaxY = point[1] > aabbMaxY ? point[1] : aabbMaxY;
			aabbMaxZ = point[2] > aabbMaxZ ? point[2] : aabbMaxZ;
		}
	}

	virtual inline void SwapAxisYZ() = 0;

	inline const std::vector<float>& GetPoints() const { return points; }
	inline std::vector<float>& GetPoints() { return points; }

	inline std::tuple<float, float, float> GetAABBMin() { return std::make_tuple(aabbMinX, aabbMinY, aabbMinZ); }
	inline std::tuple<float, float, float> GetAABBMax() { return std::make_tuple(aabbMaxX, aabbMaxY, aabbMaxZ); }
	inline std::tuple<float, float, float> GetAABBCenter()
	{
		return std::make_tuple(
			(aabbMinX + aabbMaxX) * 0.5f,
			(aabbMinY + aabbMaxY) * 0.5f,
			(aabbMinZ + aabbMaxZ) * 0.5f);
	}

protected:
	std::vector<float> points;

	float aabbMinX = FLT_MAX;
	float aabbMinY = FLT_MAX;
	float aabbMinZ = FLT_MAX;

	float aabbMaxX = -FLT_MAX;
	float aabbMaxY = -FLT_MAX;
	float aabbMaxZ = -FLT_MAX;
};

class XYZFormat : public HSerializable
{
public:
	virtual bool Serialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "wb");
		if (0 != err)
		{
			printf("[Serialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		fprintf(fp, "%d\n", (int)points.size() / 3);
		for (size_t i = 0; i < points.size() / 3; i++)
		{
			fprintf(fp, "%.6f %.6f %.6f\n", points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2]);
		}

		fclose(fp);

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "rb");
		if (0 != err)
		{
			printf("[Deserialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		int size = 0;
		fscanf(fp, "%d\n", &size);

		for (size_t i = 0; i < size; i++)
		{
			float x, y, z;
			fscanf(fp, "%f %f %f\n", &x, &y, &z);

			points.push_back(x);
			points.push_back(y);
			points.push_back(z);
		}

		fclose(fp);

		return true;
	}
};

class OFFFormat : public HSerializable
{
public:
	virtual bool Serialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "wb");
		if (0 != err)
		{
			printf("[Serialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		fprintf(fp, "OFF\n");
		auto pointCount = 0 < points.size() ? points.size() / 3 : 0;
		auto indexCount = 0 < indices.size() ? indices.size() / 3 : 0;
		if (0 == indexCount && 0 < colors.size())
		{
			indexCount = colors.size() / 3;
		}
		fprintf(fp, "%llu %llu %llu\n", pointCount, indexCount, (size_t)0);

		for (size_t i = 0; i < points.size() / 3; i++)
		{
			auto x = points[3 * i + 0];
			auto y = points[3 * i + 1];
			auto z = points[3 * i + 2];

			fprintf(fp, "%4.6f %4.6f %4.6f\n", x, y, z);

			if (0 == i % 10000)
			{
				auto percent = ((double)i / (double)(points.size() / 3)) * 100.0;
				printf("[%llu / %llu] %f percent\n", i, points.size() / 3, percent);
			}
		}

#pragma region To remove float precision error
		//for (size_t i = 0; i < points.size() / 3; i++)
		//{
		//	auto fx = floorf((points[3 * i + 0]) * 100.0f + 0.001f);
		//	auto fy = floorf((points[3 * i + 1]) * 100.0f + 0.001f);
		//	auto fz = floorf((points[3 * i + 2]) * 100.0f + 0.001f);

		//	int x = (int)fx;
		//	int y = (int)fy;
		//	int z = (int)fz;

		//	//printf("%7d %7d %7d\n", x, y, z);
		//	fprintf(fp, "%4.6f %4.6f %4.6f\n", points[3 * i + 0], points[3 * i + 1], points[3 * i + 2]);
		//	fprintf(fp, "%7d %7d %7d\n", x, y, z);

		//	if (0 == i % 10000)
		//	{
		//		auto percent = ((double)i / (double)(points.size() / 3)) * 100.0;
		//		printf("[%llu / %llu] %f percent\n", i, points.size() / 3, percent);
		//	}
		//}
#pragma endregion


		for (size_t i = 0; i < indices.size() / 3; i++)
		{
			auto i0 = indices[i * 3 + 0];
			auto i1 = indices[i * 3 + 1];
			auto i2 = indices[i * 3 + 2];

			if (colors.empty())
			{
				fprintf(fp, "3 %7d %7d %7d 255 255 255\n", i0, i1, i2);
			}
			else
			{
				auto red = unsigned char(colors[i0 * 3 + 0] * 255);
				auto green = unsigned char(colors[i0 * 3 + 1] * 255);
				auto blue = unsigned char(colors[i0 * 3 + 2] * 255);

				fprintf(fp, "3 %7d %7d %7d %3d %3d %3d\n", i0, i1, i2, red, green, blue);
			}
		}

		if (0 == indices.size() && 0 < colors.size())
		{
			for (size_t i = 0; i < colors.size() / 3; i++)
			{
				auto red = unsigned char(colors[i * 3 + 0] * 255);
				auto green = unsigned char(colors[i * 3 + 1] * 255);
				auto blue = unsigned char(colors[i * 3 + 2] * 255);

				fprintf(fp, "1 %3d %3d %3d\n", red, green, blue);
			}
		}

		fclose(fp);

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "rb");
		if (0 != err)
		{
			printf("[Deserialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		char buffer[1024];
		memset(buffer, 0, 1024);
		auto line = fgets(buffer, 1024, fp);
		if (0 != strcmp(line, "OFF\n"))
			return false;

		line = fgets(buffer, 1024, fp);
		while ('#' == line[0])
		{
			line = fgets(buffer, 1024, fp);
		}

		size_t vertexCount = 0;
		size_t triangleCount = 0;
		sscanf_s(line, "%llu %llu", &vertexCount, &triangleCount);

		printf("vertexCount : %llu, triangleCount : %llu\n", vertexCount, triangleCount);

		for (size_t i = 0; i <= vertexCount; i++)
		{
			line = fgets(buffer, 1024, fp);
			if (nullptr != line)
			{
				if ('#' == line[0])
				{
					i--;
					continue;
				}
				else
				{
					float x, y, z;
					sscanf_s(line, "%f %f %f\n", &x, &y, &z);

					AddPoint(x, y, z);
				}
			}
		}

		for (size_t i = 0; i < triangleCount; i++)
		{
			line = fgets(buffer, 1024, fp);
			if ('#' == line[0])
			{
				i--;
				continue;
			}
			else
			{
				size_t count, i0, i1, i2;
				sscanf_s(line, "%llu %llu %llu %llu\n", &count, &i0, &i1, &i2);

				AddIndex((unsigned int)i0);
				AddIndex((unsigned int)i1);
				AddIndex((unsigned int)i2);
			}
		}

		//return false;

		while (nullptr != line)
		{
			printf("%s", line);

			line = fgets(buffer, 1024, fp);
		}

		fclose(fp);

		return true;
	}

	inline const std::vector<unsigned int>& GetIndices() const { return indices; }
	inline const std::vector<float>& GetColors() const { return colors; }

	virtual inline void AddIndex(unsigned int index) { indices.push_back(index); }

	virtual inline void AddColor(float r, float g, float b)
	{
		colors.push_back(r);
		colors.push_back(g);
		colors.push_back(b);
	}

	virtual inline void SetColor(size_t index, float color) { if (index < colors.size() - 1) colors[index] = color; }

protected:
	std::vector<unsigned int> indices;
	std::vector<float> colors;
};

class CustomMeshFormat : public HSerializable
{
public:
	virtual bool Serialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "wb");
		if (0 != err)
		{
			printf("[Serialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		if (0 < points.size())
		{
			fprintf_s(fp, "%llu\n", points.size() / 3);
			for (size_t i = 0; i < points.size() / 3; i++)
			{
				fprintf(fp, "%f, %f, %f\n", points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2]);
			}
		}
		else
		{
			fprintf_s(fp, "%llu\n", (size_t)0);
		}

		if (0 < normals.size())
		{
			fprintf_s(fp, "%llu\n", normals.size() / 3);
			for (size_t i = 0; i < normals.size() / 3; i++)
			{
				fprintf(fp, "%f, %f, %f\n", normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]);
			}
		}
		else
		{
			fprintf_s(fp, "%llu\n", (size_t)0);
		}

		if (0 < colors.size())
		{
			fprintf_s(fp, "%llu\n", colors.size() / 3);
			for (size_t i = 0; i < colors.size() / 3; i++)
			{
				fprintf(fp, "%f, %f, %f\n", colors[i * 3 + 0], colors[i * 3 + 1], colors[i * 3 + 2]);
			}
		}
		else
		{
			fprintf_s(fp, "%llu\n", (size_t)0);
		}

		if (0 < indices.size())
		{
			fprintf_s(fp, "%llu\n", indices.size() / 3);
			for (size_t i = 0; i < indices.size() / 3; i++)
			{
				fprintf(fp, "%d, %d, %d\n", indices[i * 3 + 0], indices[i * 3 + 1], indices[i * 3 + 2]);
			}
		}
		else
		{
			fprintf_s(fp, "%llu\n", (size_t)0);
		}

		fclose(fp);

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "rb");
		if (0 != err)
		{
			printf("[Deserialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		char buffer[1024];
		while (nullptr != fgets(buffer, sizeof(buffer), fp))
		{
			size_t nop = 0;
			sscanf(buffer, "%llu\n", &nop);
			for (size_t i = 0; i < nop; i++)
			{
				fgets(buffer, sizeof(buffer), fp);
				if (nullptr == buffer) break;

				float x, y, z;
				sscanf(buffer, "%f, %f, %f\n", &x, &y, &z);
				AddPoint(x, y, z);
			}

			size_t non = 0;
			fgets(buffer, sizeof(buffer), fp);
			if (nullptr == buffer) break;
			sscanf(buffer, "%llu\n", &non);
			for (size_t i = 0; i < non; i++)
			{
				fgets(buffer, sizeof(buffer), fp);
				if (nullptr == buffer) break;

				float x, y, z;
				sscanf(buffer, "%f, %f, %f\n", &x, &y, &z);
				AddNormal(x, y, z);
			}

			size_t noc = 0;
			fgets(buffer, sizeof(buffer), fp);
			if (nullptr == buffer) break;
			sscanf(buffer, "%llu\n", &noc);
			for (size_t i = 0; i < noc; i++)
			{
				fgets(buffer, sizeof(buffer), fp);
				if (nullptr == buffer) break;

				float r, g, b;
				sscanf(buffer, "%f, %f, %f\n", &r, &g, &b);
				AddColor(r, g, b);
			}

			size_t noi = 0;
			fgets(buffer, sizeof(buffer), fp);
			if (nullptr == buffer) break;
			sscanf(buffer, "%llu\n", &noi);
			for (size_t i = 0; i < noc; i++)
			{
				fgets(buffer, sizeof(buffer), fp);
				if (nullptr == buffer) break;

				size_t i0, i1, i2;
				sscanf(buffer, "%llu, %llu, %llu\n", &i0, &i1, &i2);

				AddIndex((unsigned int)i0);
				AddIndex((unsigned int)i1);
				AddIndex((unsigned int)i2);
			}
		}

		return true;
	}

	inline const std::vector<unsigned int>& GetIndices() const { return indices; }
	inline const std::vector<float>& GetColors() const { return colors; }

	virtual inline void AddNormal(float x, float y, float z)
	{
		normals.push_back(x);
		normals.push_back(y);
		normals.push_back(z);
	}

	virtual inline void AddNormalFloat3(const float* normal)
	{
		normals.push_back(normal[0]);
		normals.push_back(normal[1]);
		normals.push_back(normal[2]);
	}

	virtual inline void AddIndex(unsigned int index) { indices.push_back(index); }

	virtual inline void AddColor(float r, float g, float b)
	{
		colors.push_back(r);
		colors.push_back(g);
		colors.push_back(b);
	}

	virtual inline void SetColor(size_t index, float color) { if (index < colors.size() - 1) colors[index] = color; }

protected:
	std::vector<float> normals;
	std::vector<unsigned int> indices;
	std::vector<float> colors;
};

class OBJFormat : public HSerializable
{
public:
	virtual bool Serialize(const std::string& filename)
	{
		std::ofstream ofs(filename);
		std::stringstream ss;
		ss.precision(6);

		ss << "# cuTSDF::ResourceIO::OBJ" << std::endl;
		for (size_t i = 0; i < points.size() / 3; i++)
		{
			auto x = points[3 * i + 0];
			auto y = points[3 * i + 1];
			auto z = points[3 * i + 2];

			if (colors.size() == 0)
			{
				ss << "v " << x << " " << y << " " << z << std::endl;
			}
			else if (colors.size() == points.size())
			{
				auto r = colors[3 * i + 0];
				auto g = colors[3 * i + 1];
				auto b = colors[3 * i + 2];

				ss << "v " << x << " " << y << " " << z << " " << r << " " << g << " " << b << std::endl;
			}

			if (normals.size() == points.size())
			{
				auto x = normals[3 * i + 0];
				auto y = normals[3 * i + 1];
				auto z = normals[3 * i + 2];

				ss << "vn " << x << " " << y << " " << z << std::endl;

				//printf("%f %f %f\n", x, y, z);
			}
		}

		for (size_t i = 0; i < uvs.size() / 2; i++)
		{
			auto u = uvs[2 * i + 0];
			auto v = uvs[2 * i + 1];

			ss << "vt " << u << " " << v << std::endl;
		}

		//for (size_t i = 0; i < normals.size() / 3; i++)
		//{
		//	auto x = normals[3 * i + 0];
		//	auto y = normals[3 * i + 1];
		//	auto z = normals[3 * i + 2];

		//	ss << "vn " << x << " " << y << " " << z << std::endl;
		//}

		bool has_uv = uvs.size() != 0;
		bool has_vn = normals.size() != 0;

		auto nof = indices.size() / 3;
		if (nof == 0)
		{
			//for (size_t i = 0; i < points.size(); i++)
			//{
			//	if (has_uv && has_vn)
			//	{
			//		ss << "f "
			//			<< i + 1 << "/" << i + 1 << "/" << i + 1 << " "
			//			<< i + 1 << "/" << i + 1 << "/" << i + 1 << " "
			//			<< i + 1 << "/" << i + 1 << "/" << i + 1 << std::endl;
			//	}
			//	else if (has_uv)
			//	{
			//		ss << "f "
			//			<< i + 1 << "/" << i + 1 << " "
			//			<< i + 1 << "/" << i + 1 << " "
			//			<< i + 1 << "/" << i + 1 << std::endl;
			//	}
			//	else if (has_vn)
			//	{
			//		ss << "f "
			//			<< i + 1 << "//" << i + 1 << " "
			//			<< i + 1 << "//" << i + 1 << " "
			//			<< i + 1 << "//" << i + 1 << std::endl;
			//	}
			//	else
			//	{
			//		ss << "f " << i + 1 << " " << i + 1 << " " << i + 1 << std::endl;
			//	}

			//	if (0 == i % 10000)
			//	{
			//		auto percent = ((double)i / (double)(points.size())) * 100.0;
			//		printf("[%llu / %llu] %f percent\n", i, points.size(), percent);
			//	}
			//}
		}
		else
		{
			for (size_t i = 0; i < nof; i++)
			{
				//uint32_t face[3] = { (uint32_t)i * 3 + 1, (uint32_t)i * 3 + 2, (uint32_t)i * 3 + 3 };
				uint32_t face[3] = { indices[i * 3 + 0],indices[i * 3 + 1],indices[i * 3 + 2] };

				if (has_uv && has_vn)
				{
					ss << "f "
						<< face[0] << "/" << face[0] << "/" << face[0] << " "
						<< face[1] << "/" << face[1] << "/" << face[1] << " "
						<< face[2] << "/" << face[2] << "/" << face[2] << std::endl;
				}
				else if (has_uv)
				{
					ss << "f "
						<< face[0] << "/" << face[0] << " "
						<< face[1] << "/" << face[1] << " "
						<< face[2] << "/" << face[2] << std::endl;
				}
				else if (has_vn)
				{
					ss << "f "
						<< face[0] << "//" << face[0] << " "
						<< face[1] << "//" << face[1] << " "
						<< face[2] << "//" << face[2] << std::endl;
				}
				else
				{
					ss << "f " << face[0] << " " << face[1] << " " << face[2] << std::endl;
				}

				if (0 == i % 10000)
				{
					auto percent = ((double)i / (double)(nof)) * 100.0;
					printf("[%llu / %llu] %f percent\n", i, nof, percent);
				}
			}
		}

		ofs << ss.rdbuf();
		ofs.close();

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		std::ifstream ifs(filename);
		if (false == ifs.is_open())
		{
			printf("filename : %s is not open\n", filename.c_str());
			return false;
		}

		std::stringstream buffer;
		buffer << ifs.rdbuf();

		std::string line;
		while (buffer.good())
		{
			getline(buffer, line);
			ParseOneLine(line, points, uvs, normals, colors, indices, 1.0f, 1.0f, 1.0f);
		}

		return true;
	}

	inline const std::vector<float>& GetNormals() const { return normals; }
	inline const std::vector<unsigned int>& GetIndices() const { return indices; }
	inline const std::vector<float>& GetColors() const { return colors; }

	virtual inline void AddUV(float u, float v)
	{
		uvs.push_back(u);
		uvs.push_back(v);
	}

	virtual inline void AddUVFloat2(const float* uv)
	{
		uvs.push_back(uv[0]);
		uvs.push_back(uv[1]);
	}

	virtual inline void AddNormal(float x, float y, float z)
	{
		normals.push_back(x);
		normals.push_back(y);
		normals.push_back(z);
	}

	virtual inline void AddNormalFloat3(const float* normal)
	{
		normals.push_back(normal[0]);
		normals.push_back(normal[1]);
		normals.push_back(normal[2]);
	}

	virtual inline void AddIndex(unsigned int index) { indices.push_back(index); }

	virtual inline void AddColor(float r, float g, float b)
	{
		colors.push_back(r);
		colors.push_back(g);
		colors.push_back(b);
	}

	virtual inline void AddColorFloat3(const float* color)
	{
		colors.push_back(color[0]);
		colors.push_back(color[1]);
		colors.push_back(color[2]);
	}

	virtual inline void SetColor(size_t index, float color) { if (index < colors.size() - 1) colors[index] = color; }

protected:
	std::vector<float> uvs;
	std::vector<float> normals;
	std::vector<unsigned int> indices;
	std::vector<float> colors;
};

class PLYFormat : public HSerializable
{
public:
	virtual bool Serialize(const std::string& filename)
	{
		std::ofstream ofs(filename);
		std::stringstream ss;
		ss.precision(6);

		ss << "ply" << std::endl;
		ss << "format ascii 1.0" << std::endl;
		ss << "element vertex " << points.size() / 3 << std::endl;
		ss << "property float x" << std::endl;
		ss << "property float y" << std::endl;
		ss << "property float z" << std::endl;

		if (normals.size() == points.size())
		{
			ss << "property float nx" << std::endl;
			ss << "property float ny" << std::endl;
			ss << "property float nz" << std::endl;
		}
		if (colors.size() == points.size() || colors.size() / 4 == points.size() / 3)
		{
			ss << "property uchar red" << std::endl;
			ss << "property uchar green" << std::endl;
			ss << "property uchar blue" << std::endl;
			if (useAlpha)
			{
				ss << "property uchar alpha" << std::endl;
			}
		}
		if (uvs.size() == points.size())
		{
			ss << "property float u" << std::endl;
			ss << "property float v" << std::endl;
		}

		if (lineIndices.size() > 0)
		{
			ss << "element edge " << lineIndices.size() / 2 << std::endl;
			ss << "property int vertex1" << std::endl;
			ss << "property int vertex2" << std::endl;
		}

		if (triangleIndices.size() > 0)
		{
			ss << "element face " << triangleIndices.size() / 3 << std::endl;
			ss << "property list uchar int vertex_indices" << std::endl;
		}

		ss << "end_header" << std::endl;

		for (size_t i = 0; i < points.size() / 3; i++)
		{
			auto x = points[3 * i + 0];
			auto y = points[3 * i + 1];
			auto z = points[3 * i + 2];

			ss << x << " " << y << " " << z << " ";

			if (normals.size() == points.size())
			{
				auto nx = normals[3 * i + 0];
				auto ny = normals[3 * i + 1];
				auto nz = normals[3 * i + 2];

				ss << nx << " " << ny << " " << nz << " ";
			}

			if (false == useAlpha)
			{
				if (colors.size() == points.size())
				{
					auto red = (unsigned char)(colors[3 * i + 0] * 255.0f);
					auto green = (unsigned char)(colors[3 * i + 1] * 255.0f);
					auto blue = (unsigned char)(colors[3 * i + 2] * 255.0f);

					ss << (int)red << " " << (int)green << " " << (int)blue << " ";
				}
			}
			else
			{
				if (colors.size() / 4 == points.size() / 3)
				{
					auto red = (unsigned char)(colors[4 * i + 0] * 255.0f);
					auto green = (unsigned char)(colors[4 * i + 1] * 255.0f);
					auto blue = (unsigned char)(colors[4 * i + 2] * 255.0f);
					auto alpha = (unsigned char)(colors[4 * i + 3] * 255.0f);

					ss << (int)red << " " << (int)green << " " << (int)blue << " " << (int)alpha << " ";
				}
			}

			if (uvs.size() == points.size())
			{
				auto u = uvs[3 * i + 0];
				auto v = uvs[3 * i + 1];

				ss << u << " " << v << " ";
			}

			ss << std::endl;

			if (0 == i % 10000 && i != 0)
			{
				auto percent = ((double)i / (double)(points.size() / 3)) * 100.0;
				printf("[Vertices %llu / %llu] %f percent\n", i, points.size(), percent);
			}
		}

		if (lineIndices.size() > 0)
		{
			for (size_t i = 0; i < lineIndices.size(); i += 2)
			{
				auto i0 = lineIndices[i + 0];
				auto i1 = lineIndices[i + 1];

				//ss << i0 << " " << i1 << std::endl;
				ss << "2 " << i0 << " " << i1 << std::endl;

				if (0 == i % 10000 && i != 0)
				{
					auto percent = ((double)i / (double)(lineIndices.size())) * 100.0;
					printf("[Lines %llu / %llu] %f percent\n", i / 2, lineIndices.size() / 2, percent);
				}
			}
		}

		if (triangleIndices.size() > 0)
		{
			for (size_t i = 0; i < triangleIndices.size() / 3; i++)
			{
				auto i0 = triangleIndices[3 * i + 0];
				auto i1 = triangleIndices[3 * i + 1];
				auto i2 = triangleIndices[3 * i + 2];

				ss << "3 " << i0 << " " << i1 << " " << i2 << std::endl;

				if (0 == i % 10000 && i != 0)
				{
					auto percent = ((double)i / (double)(triangleIndices.size() / 3)) * 100.0;
					printf("[Triangles %llu / %llu] %f percent\n", i, triangleIndices.size() / 3, percent);
				}
			}
		}

		ofs << ss.rdbuf();
		ofs.close();

		printf("\"%s\" saved.\n", filename.c_str());

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		std::ifstream ifs(filename);
		if (false == ifs.is_open())
		{
			printf("filename : %s is not open\n", filename.c_str());
			return false;
		}

		std::stringstream buffer;
		buffer << ifs.rdbuf();

		std::string line;
		std::vector<std::string> elementNames;
		std::vector<size_t> elementCounts;
		std::vector<bool> listTypeInfo;
		std::vector<std::vector<std::string>> elementPropertyTypes;
		std::vector<std::vector<std::string>> elementPropertyNames;

		if (buffer.good())
		{
			getline(buffer, line);
			if (false == ("ply" == line || "PLY" == line))
			{
				printf("Not a ply file");
				return false;
			}
		}
		while (buffer.good())
		{
			getline(buffer, line);
			std::stringstream ss(line);
			auto words = split(line, " \t");
			if (words[0] == "format")
			{

			}
			else if (words[0] == "element")
			{
				elementNames.push_back(words[1]);
				elementCounts.push_back(atoi(words[2].c_str()));
			}
			else if (words[0] == "property")
			{
				auto index = elementNames.size() - 1;
				if (elementPropertyTypes.size() <= index)
				{
					elementPropertyTypes.push_back(std::vector<std::string>());
					elementPropertyNames.push_back(std::vector<std::string>());

					listTypeInfo.push_back(false);
				}
				if ("list" != words[1])
				{
					elementPropertyTypes[index].push_back(words[1]);
					elementPropertyNames[index].push_back(words[2]);
					if ("a" == words[2] || "alpha" == words[2])
					{
						useAlpha = true;
					}
				}
				else
				{
					if (words[1] == "list")
					{
						listTypeInfo[index] = true;

						elementPropertyTypes[index].push_back(words[2]);
						elementPropertyTypes[index].push_back(words[3]);

						elementPropertyNames[index].push_back(words[4]);
					}
				}
			}
			else if (words[0] == "end_header")
			{
				break;
			}
		}

		for (size_t i = 0; i < elementNames.size(); i++)
		{
			float x, y, z, nx, ny, nz;
			unsigned char red, green, blue, alpha;
			bool isListype = listTypeInfo[i];
			if (isListype == false)
			{
				for (size_t j = 0; j < elementCounts[i]; j++)
				{
					getline(buffer, line);
					auto words = split(line, " \t");

					for (size_t k = 0; k < words.size(); k++)
					{
						if (elementPropertyNames[i][k] == "x")
						{
							x = (float)atof(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "y")
						{
							y = (float)atof(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "z")
						{
							z = (float)atof(words[k].c_str());

							AddPoint(x, y, z);
						}

						else if (elementPropertyNames[i][k] == "nx")
						{
							nx = (float)atof(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "ny")
						{
							ny = (float)atof(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "nz")
						{
							nz = (float)atof(words[k].c_str());

							AddNormal(nx, ny, nz);
						}

						else if (elementPropertyNames[i][k] == "red")
						{
							red = (unsigned char)atoi(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "green")
						{
							green = (unsigned char)atoi(words[k].c_str());
						}
						else if (elementPropertyNames[i][k] == "blue")
						{
							blue = (unsigned char)atoi(words[k].c_str());
							if (false == useAlpha)
							{
								AddColor((float)red / 255.0f, (float)green / 255.0f, (float)blue / 255.0f);
							}
						}
						else if (elementPropertyNames[i][k] == "alpha")
						{
							alpha = (unsigned char)atoi(words[k].c_str());

							AddColor((float)red / 255.0f, (float)green / 255.0f, (float)blue / 255.0f, (float)alpha / 255.0f);
						}
						else if (elementPropertyNames[i][k] == "label")
						{
							int label = (int)atoi(words[k].c_str());
							labels.push_back(label);
						}
						else if (elementPropertyNames[i][k] == "deepLearningClass")
						{
							int deepLearningClass = (int)atoi(words[k].c_str());
							deepLearningClasses.push_back(deepLearningClass);
						}
					}
				}
			}
			else
			{
				for (size_t j = 0; j < elementCounts[i]; j++)
				{
					getline(buffer, line);
					auto words = split(line, " \t");

					auto noi = atoi(words[0].c_str());
					for (size_t k = 1; k <= noi; k++)
					{
						auto vi = atoi(words[k].c_str());

						AddTriangleIndex(vi);
					}
				}
			}
		}

		printf("\"%s\" loaded.\n", filename.c_str());

		return true;
	}

	//virtual bool SerializeAsync(const std::string& filename)
	//{
	//	async(launch::async, [&, filename]() {
	//		Serialize(filename);
	//		});
	//	return true;
	//}

	inline std::vector<float>& GetNormals() { return normals; }
	inline const std::vector<float>& GetNormals() const { return normals; }
	inline const std::vector<unsigned int>& GetLineIndices() const { return lineIndices; }
	inline const std::vector<unsigned int>& GetTriangleIndices() const { return triangleIndices; }
	inline std::vector<float>& GetColors() { return colors; }
	inline const std::vector<float>& GetColors() const { return colors; }
	inline std::vector<int>& GetLabels() { return labels; }
	inline const std::vector<int>& GetLabels() const { return labels; }
	inline std::vector<int>& GetDeepLearningClasses() { return deepLearningClasses; }
	inline const std::vector<int>& GetDeepLearningClasses() const { return deepLearningClasses; }
	inline bool UseAlpha() const { return useAlpha; }

	virtual inline void AddUV(float u, float v)
	{
		uvs.push_back(u);
		uvs.push_back(v);
	}

	virtual inline void AddUVFloat2(const float* uv)
	{
		uvs.push_back(uv[0]);
		uvs.push_back(uv[1]);
	}

	virtual inline void AddNormal(float x, float y, float z)
	{
		normals.push_back(x);
		normals.push_back(y);
		normals.push_back(z);
	}

	virtual inline void AddNormalFloat3(const float* normal)
	{
		normals.push_back(normal[0]);
		normals.push_back(normal[1]);
		normals.push_back(normal[2]);
	}

	virtual inline void AddLineIndex(unsigned int index) { lineIndices.push_back(index); }

	virtual inline void AddTriangleIndex(unsigned int index) { triangleIndices.push_back(index); }

	virtual inline void AddFace(unsigned int i0, unsigned int i1, unsigned int i2)
	{
		triangleIndices.push_back(i0);
		triangleIndices.push_back(i1);
		triangleIndices.push_back(i2);
	}

	virtual inline void AddColor(float r, float g, float b)
	{
		colors.push_back(r);
		colors.push_back(g);
		colors.push_back(b);
	}

	virtual inline void AddColorFloat3(const float* color)
	{
		colors.push_back(color[0]);
		colors.push_back(color[1]);
		colors.push_back(color[2]);
	}

	virtual inline void AddColor(float r, float g, float b, float a)
	{
		colors.push_back(r);
		colors.push_back(g);
		colors.push_back(b);
		colors.push_back(a);
		useAlpha = true;
	}

	virtual inline void AddColorFloat4(const float* color)
	{
		colors.push_back(color[0]);
		colors.push_back(color[1]);
		colors.push_back(color[2]);
		colors.push_back(color[3]);
		useAlpha = true;
	}

	virtual inline void SetColor(size_t index, float color) { if (index < colors.size() - 1) colors[index] = color; }

	virtual void AddCube(
		float centerX, float centerY, float centerZ,
		float normalX, float normalY, float normalZ,
		float r, float g, float b, float a, float scale)
	{
		float half = scale * 0.5f;

		float verts[8][3] = {
			{centerX - half, centerY - half, centerZ - half}, // 0
			{centerX + half, centerY - half, centerZ - half}, // 1
			{centerX + half, centerY + half, centerZ - half}, // 2
			{centerX - half, centerY + half, centerZ - half}, // 3
			{centerX - half, centerY - half, centerZ + half}, // 4
			{centerX + half, centerY - half, centerZ + half}, // 5
			{centerX + half, centerY + half, centerZ + half}, // 6
			{centerX - half, centerY + half, centerZ + half}  // 7
		};

		unsigned int baseIdx = static_cast<unsigned int>(points.size() / 3);

		// 정점, 노말, 컬러 추가
		for (int i = 0; i < 8; ++i)
		{
			AddPoint(verts[i][0], verts[i][1], verts[i][2]);
			AddNormal(normalX, normalY, normalZ);
			if (useAlpha)
				AddColor(r, g, b, a);
			else
				AddColor(r, g, b);
		}

		// 12개 삼각형을 인덱스 배열로 정의
		static const unsigned int cube_tris[12][3] =
		{
			{0, 1, 2}, {0, 2, 3}, // -Z
			{4, 6, 5}, {4, 7, 6}, // +Z
			{0, 4, 5}, {0, 5, 1}, // -Y
			{2, 6, 7}, {2, 7, 3}, // +Y
			{0, 3, 7}, {0, 7, 4}, // -X
			{1, 5, 6}, {1, 6, 2}  // +X
		};

		// AddFace (index는 baseIdx 기준 offset)
		for (int i = 0; i < 12; ++i)
		{
			AddFace(
				baseIdx + cube_tris[i][0],
				baseIdx + cube_tris[i][1],
				baseIdx + cube_tris[i][2]
			);
		}
	}

	virtual void AddData(
		float* positions,
		float* normals,
		float* colors,
		unsigned int numberOfPoints,
		bool useAlpha)
	{
		this->useAlpha = useAlpha;

		auto lastPointSize = this->points.size();
		this->points.resize(lastPointSize + numberOfPoints * 3);
		memcpy(this->points.data() + lastPointSize, positions, sizeof(float) * numberOfPoints * 3);

		auto lastNormalSize = this->normals.size();
		this->normals.resize(lastNormalSize + numberOfPoints * 3);
		memcpy(this->normals.data() + lastNormalSize, positions, sizeof(float) * numberOfPoints * 3);

		if (useAlpha)
		{
			auto lastColorSize = this->colors.size();
			this->colors.resize(lastColorSize + numberOfPoints * 3);
			memcpy(this->colors.data() + lastColorSize, positions, sizeof(float) * numberOfPoints * 3);
		}
		else
		{
			auto lastColorSize = this->colors.size();
			this->colors.resize(lastColorSize + numberOfPoints * 4);
			memcpy(this->colors.data() + lastColorSize, positions, sizeof(float) * numberOfPoints * 4);
		}
	}

	virtual inline void SwapAxisYZ()
	{
		if (false == points.empty())
		{
			for (size_t i = 0; i < points.size() / 3; i++)
			{
				auto temp = points[i * 3 + 1];
				points[i * 3 + 1] = points[i * 3 + 2];
				points[i * 3 + 2] = temp;
			}
		}
		if (false == normals.empty())
		{
			for (size_t i = 0; i < normals.size() / 3; i++)
			{
				auto temp = normals[i * 3 + 1];
				normals[i * 3 + 1] = normals[i * 3 + 2];
				normals[i * 3 + 2] = temp;
			}
		}

		{
			float temp = aabbMinY;
			aabbMinY = aabbMinZ;
			aabbMinZ = temp;
		}

		{
			float temp = aabbMaxY;
			aabbMaxY = aabbMaxZ;
			aabbMaxZ = temp;
		}
	}

	void FilterWithinAABB(
		float minX, float minY, float minZ,
		float maxX, float maxY, float maxZ)
	{
		std::vector<float> filteredPoints;
		std::vector<float> filteredNormals;
		std::vector<float> filteredColors;
		std::vector<float> filteredUVs;
		std::vector<int> filteredLabels;
		std::vector<int> filteredDeepLearningClasses;

		bool hasNormals = (normals.size() == points.size());
		bool hasColorsRGB = (colors.size() == points.size());
		bool hasColorsRGBA = (!hasColorsRGB && colors.size() / 4 == points.size() / 3);
		bool hasUVs = (uvs.size() / 2 == points.size() / 3);
		bool hasLabels = (labels.size() == points.size() / 3);
		bool hasDLClasses = (deepLearningClasses.size() == points.size() / 3);

		size_t numPoints = points.size() / 3;

		for (size_t i = 0; i < numPoints; i++)
		{
			auto x = points[i * 3 + 0];
			auto y = points[i * 3 + 1];
			auto z = points[i * 3 + 2];

			if (x >= minX && x <= maxX &&
				y >= minY && y <= maxY &&
				z >= minZ && z <= maxZ)
			{
				filteredPoints.push_back(x);
				filteredPoints.push_back(y);
				filteredPoints.push_back(z);

				if (hasNormals)
				{
					filteredNormals.push_back(normals[i * 3 + 0]);
					filteredNormals.push_back(normals[i * 3 + 1]);
					filteredNormals.push_back(normals[i * 3 + 2]);
				}

				if (hasColorsRGB)
				{
					filteredColors.push_back(colors[i * 3 + 0]);
					filteredColors.push_back(colors[i * 3 + 1]);
					filteredColors.push_back(colors[i * 3 + 2]);
				}
				else if (hasColorsRGBA)
				{
					filteredColors.push_back(colors[i * 4 + 0]);
					filteredColors.push_back(colors[i * 4 + 1]);
					filteredColors.push_back(colors[i * 4 + 2]);
					filteredColors.push_back(colors[i * 4 + 3]);
				}

				if (hasUVs)
				{
					filteredUVs.push_back(uvs[i * 2 + 0]);
					filteredUVs.push_back(uvs[i * 2 + 1]);
				}

				if (hasLabels)
				{
					filteredLabels.push_back(labels[i]);
				}

				if (hasDLClasses)
				{
					filteredDeepLearningClasses.push_back(deepLearningClasses[i]);
				}
			}
		}

		if (hasNormals) normals = filteredNormals;
		if (hasColorsRGB || hasColorsRGBA) colors = filteredColors;
		if (hasUVs) uvs = filteredUVs;
		if (hasLabels) labels = filteredLabels;
		if (hasDLClasses) deepLearningClasses = filteredDeepLearningClasses;

		points = filteredPoints;
	}

protected:
	std::vector<float> uvs;
	std::vector<float> normals;
	std::vector<unsigned int> lineIndices;
	std::vector<unsigned int> triangleIndices;
	std::vector<float> colors;
	std::vector<int> labels;
	std::vector<int> deepLearningClasses;
	bool useAlpha = false;
};

// Asynchronous Loadable PointCloud
template<typename Point>
class ALPFormat
{
public:
	bool Serialize(const std::string& filename)
	{
		std::ofstream ofs(filename, std::ios::out | std::ios::binary);
		if (false == ofs.is_open())
		{
			printf("filename : %s is not open\n", filename.c_str());
			return false;
		}

		unsigned long writtenSize = 0;

		unsigned long nop = points.size();
		ofs.write((char*)&nop, sizeof(unsigned long));
		writtenSize += sizeof(unsigned long);

		unsigned int pointSize = sizeof(Point);
		ofs.write((char*)&pointSize, sizeof(unsigned int));
		writtenSize += sizeof(unsigned int);

		for (auto& p : points)
		{
			ofs.write((char*)&p, pointSize);
		}

		writtenSize += points.size() * pointSize;

		ofs.close();
		
		printf("%s written : %d bytes\n", filename.c_str(), writtenSize);

		return true;
	}

	bool Deserialize(const std::string& filename)
	{
		std::ifstream ifs(filename, std::ios::in | std::ios::binary);
		if (false == ifs.is_open())
		{
			printf("filename : %s is not open\n", filename.c_str());
			return false;
		}

		unsigned long readSize = 0;

		unsigned long nop = 0;
		ifs.read((char*)&nop, sizeof(unsigned long));
		readSize += sizeof(unsigned long);

		unsigned int pointSize = 0;
		ifs.read((char*)&pointSize, sizeof(unsigned int));
		readSize += sizeof(unsigned int);

		for (size_t i = 0; i < nop; i++)
		{
			Point p;
			ifs.read((char*)&p, pointSize);
			points.push_back(p);

			aabbMinX = p.position.x < aabbMinX ? p.position.x : aabbMinX;
			aabbMinY = p.position.y < aabbMinY ? p.position.y : aabbMinY;
			aabbMinZ = p.position.z < aabbMinZ ? p.position.z : aabbMinZ;

			aabbMaxX = p.position.x > aabbMaxX ? p.position.x : aabbMaxX;
			aabbMaxY = p.position.y > aabbMaxY ? p.position.y : aabbMaxY;
			aabbMaxZ = p.position.z > aabbMaxZ ? p.position.z : aabbMaxZ;
		}
		ifs.close();

		readSize += pointSize * nop;
		printf("%s read : %d bytes\n", filename.c_str(), readSize);

		return true;
	}

	void AddPoint(const Point& point)
	{
		std::lock_guard<std::mutex> lock(points_mutex);
		points.push_back(point);

		aabbMinX = point.position.x < aabbMinX ? point.position.x : aabbMinX;
		aabbMinY = point.position.y < aabbMinY ? point.position.y : aabbMinY;
		aabbMinZ = point.position.z < aabbMinZ ? point.position.z : aabbMinZ;

		aabbMaxX = point.position.x > aabbMaxX ? point.position.x : aabbMaxX;
		aabbMaxY = point.position.y > aabbMaxY ? point.position.y : aabbMaxY;
		aabbMaxZ = point.position.z > aabbMaxZ ? point.position.z : aabbMaxZ;
	}

	void AddPoints(const std::vector<Point>& inputPoints)
	{
		std::lock_guard<std::mutex> lock(points_mutex);
		points.insert(points.end(), inputPoints.begin(), inputPoints.end());

		for (auto& point : points)
		{
			aabbMinX = point.position.x < aabbMinX ? point.position.x : aabbMinX;
			aabbMinY = point.position.y < aabbMinY ? point.position.y : aabbMinY;
			aabbMinZ = point.position.z < aabbMinZ ? point.position.z : aabbMinZ;

			aabbMaxX = point.position.x > aabbMaxX ? point.position.x : aabbMaxX;
			aabbMaxY = point.position.y > aabbMaxY ? point.position.y : aabbMaxY;
			aabbMaxZ = point.position.z > aabbMaxZ ? point.position.z : aabbMaxZ;
		}
	}

	const std::vector<Point>& GetPoints() const
	{
		std::lock_guard<std::mutex> lock(points_mutex);
		return points;
	}

	void FromPLY(const PLYFormat& ply)
	{
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
				if (ply.UseAlpha())
				{
					auto cx = ply.GetColors()[i * 4];
					auto cy = ply.GetColors()[i * 4 + 1];
					auto cz = ply.GetColors()[i * 4 + 2];
					auto ca = ply.GetColors()[i * 4 + 3];

					AddPoint({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
				}
				else
				{
					auto cx = ply.GetColors()[i * 3];
					auto cy = ply.GetColors()[i * 3 + 1];
					auto cz = ply.GetColors()[i * 3 + 2];

					AddPoint({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
				}
			}
			else
			{
				AddPoint({ {px, py, pz}, {nx, ny, nz}, {1.0f, 1.0f, 1.0f} });
			}
		}
		alog("PLY %llu points loaded\n", points.size());
	}

	inline std::tuple<float, float, float> GetAABBMin() { return std::make_tuple(aabbMinX, aabbMinY, aabbMinZ); }
	inline std::tuple<float, float, float> GetAABBMax() { return std::make_tuple(aabbMaxX, aabbMaxY, aabbMaxZ); }
	inline std::tuple<float, float, float> GetAABBCenter()
	{
		return std::make_tuple(
			(aabbMinX + aabbMaxX) * 0.5f,
			(aabbMinY + aabbMaxY) * 0.5f,
			(aabbMinZ + aabbMaxZ) * 0.5f);
	}

protected:
	mutable std::mutex points_mutex;
	std::vector<Point> points;

	float aabbMinX = FLT_MAX;
	float aabbMinY = FLT_MAX;
	float aabbMinZ = FLT_MAX;

	float aabbMaxX = -FLT_MAX;
	float aabbMaxY = -FLT_MAX;
	float aabbMaxZ = -FLT_MAX;
};

class CSVFormat : public HSerializable
{
public:

	virtual bool Serialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "wb");
		if (0 != err)
		{
			printf("[Serialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		fprintf(fp, "%d\n", (int)points.size() / 3);
		fprintf(fp, "X, Y, Z\n");

		for (size_t i = 0; i < points.size() / 3; i++)
		{
			fprintf(fp, "%.6f, %.6f, %.6f\n", points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2]);
		}

		fclose(fp);

		return true;
	}

	virtual bool Deserialize(const std::string& filename)
	{
		FILE* fp = nullptr;
		auto err = fopen_s(&fp, filename.c_str(), "rb");
		if (0 != err)
		{
			printf("[Deserialize] File \"%s\" open failed.", filename.c_str());
			return false;
		}

		char buffer[1024];
		float x, y, z;

		fgets(buffer, sizeof(buffer), fp);
		memset(buffer, 0, sizeof(char) * 1024);

		while (fgets(buffer, sizeof(buffer), fp)) {
			if (sscanf(buffer, "%f,%f,%f", &x, &y, &z) == 3) {
				//printf("x = %.2f, y = %.2f, z = %.2f\n", x, y, z);
				points.push_back(x);
				points.push_back(y);
				points.push_back(z);
			}
			else {
				fprintf(stderr, "Failed to parse line: %s", buffer);
			}
		}

		fclose(fp);

		return true;
	}

	virtual inline void SwapAxisYZ()
	{
		if (false == points.empty())
		{
			for (size_t i = 0; i < points.size() / 3; i++)
			{
				auto temp = points[i * 3 + 1];
				points[i * 3 + 1] = points[i * 3 + 2];
				points[i * 3 + 2] = temp;
			}
		}
		//if (false == normals.empty())
		//{
		//	for (size_t i = 0; i < normals.size() / 3; i++)
		//	{
		//		auto temp = normals[i * 3 + 1];
		//		normals[i * 3 + 1] = normals[i * 3 + 2];
		//		normals[i * 3 + 2] = temp;
		//	}
		//}

		{
			float temp = aabbMinY;
			aabbMinY = aabbMinZ;
			aabbMinZ = temp;
		}

		{
			float temp = aabbMaxY;
			aabbMaxY = aabbMaxZ;
			aabbMaxZ = temp;
		}
	}
};