#pragma once

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <TypeDefinitions.h>

namespace MiniMath
{
	const f32 PI = 3.14159265359f;

#define i32NAN std::numeric_limits<i32>::quiet_NaN()
#define RealNAN std::numeric_limits<f32>::quiet_NaN()
#define i32Infinity std::numeric_limits<i32>::max()

#define DEG2RAD (MiniMath::PI / 180.0f)
#define RAD2DEG (180.0f / MiniMath::PI)

	const f32 Epsilon = 0.000001f;

	f32 clamp(f32 f, f32 minf, f32 maxf);

	f32 Trimax(f32 a, f32 b, f32 c);

	f32 Trimin(f32 a, f32 b, f32 c);

	struct V2
	{
		f32 x = 0.0f;
		f32 y = 0.0f;

		V2();
		V2(i32, i32);
		V2(f32, f32);
		V2(double, double);
		V2(f32*);
		V2(const char*);
		V2(const f2&);
	};

	struct V3
	{
		f32 x = 0.0f;
		f32 y = 0.0f;
		f32 z = 0.0f;

		V3();
		V3(i32, i32, i32);
		V3(f32, f32, f32);
		V3(double, double, double);
		V3(f32*);
		V3(const char* c);
		V3(const f3&);

		const V3& operator += (f32);
		const V3& operator -= (f32);
		const V3& operator *= (f32);
		const V3& operator /= (f32);

		const V3& operator += (const V3&);
		const V3& operator -= (const V3&);

		f32 operator[](i32);
	};

	inline std::ostream& operator<<(std::ostream& os, const V3& v) { return os << "(" << v.x << ", " << v.y << ", " << v.z << ")"; }

	V3 operator - (const V3&);
	V3 operator + (const V3&, f32);
	V3 operator + (f32, const V3&);
	V3 operator - (const V3&, f32);
	V3 operator - (f32, const V3&);
	V3 operator * (const V3&, f32);
	V3 operator * (f32, const V3&);
	V3 operator / (const V3&, f32);
	V3 operator / (f32, const V3&);

	V3 operator + (const V3&, const V3&);
	V3 operator - (const V3&, const V3&);

	f32 magnitude(const V3&);
	V3 normalize(const V3&);
	f32 dot(const V3&, const V3&);
	V3 cross(const V3&, const V3&);
	f32 distance(const V3&, const V3&);
	f32 angle(const V3&, const V3&);

	V3 calculateMean(const std::vector<V3>& vectors);
	void centerData(std::vector<V3>& vectors, const V3& mean);
	void calculateCovarianceMatrix(const std::vector<V3>& vectors, V3& eigenvalues, V3& eigenvector1, V3& eigenvector2, V3& eigenvector3);

	struct V4
	{
		f32 x = 0.0f;
		f32 y = 0.0f;
		f32 z = 0.0f;
		f32 w = 1.0f; // Homogeneous coordinate (default to 1 for positions)

		V4();
		V4(f32 scalar);
		V4(f32 x, f32 y, f32 z, f32 w = 1.0f);
		V4(f32* array);
		V4(const char* str);
		V4(const f4&);

		const V4& operator+=(const V4& other);
		const V4& operator-=(const V4& other);
		const V4& operator*=(f32 scalar);
		const V4& operator/=(f32 scalar);

		f32 operator[](i32 index) const;
		f32& operator[](i32 index);

		f32 magnitude() const;
		V4 normalize() const;
		f32 dot(const V4& other) const;

		static V4 zero();
		static V4 one();
		static V4 unitX();
		static V4 unitY();
		static V4 unitZ();
		static V4 unitW();
	};

	// Overloaded Operators
	V4 operator+(const V4& a, const V4& b);
	V4 operator-(const V4& a, const V4& b);
	V4 operator*(const V4& v, f32 scalar);
	V4 operator*(f32 scalar, const V4& v);
	V4 operator/(const V4& v, f32 scalar);
	std::ostream& operator<<(std::ostream& os, const V4& v);

	struct Quaternion
	{
		f32 w = 1.0f;
		f32 x = 0.0f;
		f32 y = 0.0f;
		f32 z = 0.0f;

		Quaternion(f32, f32, f32, f32);
		Quaternion(f32, const V3&);

		static Quaternion identity();
		static Quaternion zero();
	};

	Quaternion operator*(const Quaternion& q1, const Quaternion& q2);
	Quaternion conjugate(const Quaternion q);

	V3 rotate(const V3&, const Quaternion&);

	Quaternion rotation(const V3& v0, const V3& v1);

	struct M3
	{
		f32 m[3][3];

		M3();
		M3(f32 diagonal);
		M3(f32 m00, f32 m01, f32 m02,
			f32 m10, f32 m11, f32 m12,
			f32 m20, f32 m21, f32 m22);

		M3 operator+(const M3& other) const;
		M3 operator-(const M3& other) const;
		M3 operator*(f32 scalar) const;
		M3 operator*(const M3& other) const;
		V3 operator*(const V3& vec) const;

		M3 transpose() const;
		f32 determinant() const;
		M3 inverse() const;

		f32 at(i32 row, i32 column);

		static M3 identity();
		static M3 zero();
	};

	struct M4
	{
		f32 m[4][4];

		M4();
		M4(f32 diagonal);
		M4(f32 m00, f32 m01, f32 m02, f32 m03,
			f32 m10, f32 m11, f32 m12, f32 m13,
			f32 m20, f32 m21, f32 m22, f32 m23,
			f32 m30, f32 m31, f32 m32, f32 m33);

		M4 operator+(const M4& other) const;
		M4 operator-(const M4& other) const;
		M4 operator*(f32 scalar) const;
		M4 operator*(const M4& other) const;
		V4 operator*(const V4& vec) const;
		
		V3 transformPoi32(const V3& poi32) const;
		V3 transformVector(const V3& vector) const;

		M4 transpose() const;
		f32 determinant() const;
		M4 inverse() const;

		f32& at(i32 row, i32 column);
		const f32& at(i32 row, i32 column) const;

		static M4 identity();
		static M4 zero();
	};

	M4 translate(const M4&, const V3&);

	struct AABB
	{
		V3 center = { 0.0,  0.0,  0.0 };
		V3 extents = { 0.0,  0.0,  0.0 };
		V3 xyz = { FLT_MAX,  FLT_MAX,  FLT_MAX };
		V3 xyZ = { FLT_MAX,  FLT_MAX, -FLT_MAX };
		V3 xYz = { FLT_MAX, -FLT_MAX,  FLT_MAX };
		V3 xYZ = { FLT_MAX, -FLT_MAX, -FLT_MAX };
		V3 Xyz = { -FLT_MAX,  FLT_MAX,  FLT_MAX };
		V3 XyZ = { -FLT_MAX,  FLT_MAX, -FLT_MAX };
		V3 XYz = { -FLT_MAX, -FLT_MAX,  FLT_MAX };
		V3 XYZ = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

		AABB() {}
		AABB(const V3& min, const V3& max) { SetMixMax(min, max); }
		AABB(const std::vector<V3>& poi32s)
		{
			for (auto& p : poi32s)
			{
				Expand(p);
			}
		}

		inline const V3& GetMinPoi32() const { return xyz; }
		inline const V3& GetMaxPoi32() const { return XYZ; }
		inline const V3& GetCenter() const { return center; }
		inline const V3& GetExtents() const { return extents; }

		inline const V3& Getxyz() const { return xyz; }
		inline const V3& GetxyZ() const { return xyZ; }
		inline const V3& GetxYz() const { return xYz; }
		inline const V3& GetxYZ() const { return xYZ; }
		inline const V3& GetXyz() const { return Xyz; }
		inline const V3& GetXyZ() const { return XyZ; }
		inline const V3& GetXYz() const { return XYz; }
		inline const V3& GetXYZ() const { return XYZ; }

		inline void Setxyz(const V3& xyz) { this->xyz = xyz; }
		inline void SetxyZ(const V3& xyZ) { this->xyZ = xyZ; }
		inline void SetxYz(const V3& xYz) { this->xYz = xYz; }
		inline void SetxYZ(const V3& xYZ) { this->xYZ = xYZ; }
		inline void SetXyz(const V3& Xyz) { this->Xyz = Xyz; }
		inline void SetXyZ(const V3& XyZ) { this->XyZ = XyZ; }
		inline void SetXYz(const V3& XYz) { this->XYz = XYz; }
		inline void SetXYZ(const V3& XYZ) { this->XYZ = XYZ; }

		inline void SetMixMax(const V3& minPoi32, const V3& maxPoi32)
		{
			xyz = minPoi32;
			XYZ = maxPoi32;
			update();
		}

		inline void Expand(f32 scale)
		{
			auto length = magnitude(XYZ - center);
			auto dir = normalize(XYZ - center);
			Expand(XYZ + dir * scale);
			Expand(xyz + -dir * scale);
		}

		inline void Expand(f32 x, f32 y, f32 z)
		{
			if (x < xyz.x) { xyz.x = x; }
			if (y < xyz.y) { xyz.y = y; }
			if (z < xyz.z) { xyz.z = z; }

			if (x > XYZ.x) { XYZ.x = x; }
			if (y > XYZ.y) { XYZ.y = y; }
			if (z > XYZ.z) { XYZ.z = z; }

			update();
		}

		inline void Expand(const V3& p)
		{
			if (p.x < xyz.x) { xyz.x = p.x; }
			if (p.y < xyz.y) { xyz.y = p.y; }
			if (p.z < xyz.z) { xyz.z = p.z; }

			if (p.x > XYZ.x) { XYZ.x = p.x; }
			if (p.y > XYZ.y) { XYZ.y = p.y; }
			if (p.z > XYZ.z) { XYZ.z = p.z; }

			update();
		}

		inline void Expand(const std::vector<V3>& poi32s)
		{
			for (auto& p : poi32s)
			{
				Expand(p);
			}
		}

		inline void ExpandZ(f32 distance)
		{
			xyz.z -= distance;
			XYZ.z += distance;
		}

		inline f32 GetXLength() const { return XYZ.x - xyz.x; }
		inline f32 GetYLength() const { return XYZ.y - xyz.y; }
		inline f32 GetZLength() const { return XYZ.z - xyz.z; }

		inline bool Contains(const V3& p) const
		{
			return (xyz.x <= p.x && p.x <= XYZ.x) &&
				(xyz.y <= p.y && p.y <= XYZ.y) &&
				(xyz.z <= p.z && p.z <= XYZ.z);
		}

		inline bool Intersects(const AABB& other) const
		{
			if ((xyz.x > other.XYZ.x) && (xyz.y > other.XYZ.y) && (xyz.z > other.XYZ.z)) { return false; }
			if ((XYZ.x < other.xyz.x) && (XYZ.y < other.xyz.y) && (XYZ.z < other.xyz.z)) { return false; }

			if (Contains(other.xyz)) return true;
			if (Contains(other.xyZ)) return true;
			if (Contains(other.xYz)) return true;
			if (Contains(other.xYZ)) return true;
			if (Contains(other.Xyz)) return true;
			if (Contains(other.XyZ)) return true;
			if (Contains(other.XYz)) return true;
			if (Contains(other.XYZ)) return true;

			if (other.Contains(xyz)) return true;
			if (other.Contains(xyZ)) return true;
			if (other.Contains(xYz)) return true;
			if (other.Contains(xYZ)) return true;
			if (other.Contains(Xyz)) return true;
			if (other.Contains(XyZ)) return true;
			if (other.Contains(XYz)) return true;
			if (other.Contains(XYZ)) return true;

			return false;
		}

		inline void update()
		{
			xYz.x = xyz.x; xYz.y = XYZ.y; xYz.z = xyz.z;
			xyZ.x = xyz.x; xyZ.y = xyz.y; xyZ.z = XYZ.z;
			xYZ.x = xyz.x; xYZ.y = XYZ.y; xYZ.z = XYZ.z;
			Xyz.x = XYZ.x; Xyz.y = xyz.y; Xyz.z = xyz.z;
			XYz.x = XYZ.x; XYz.y = XYZ.y; XYz.z = xyz.z;
			XyZ.x = XYZ.x; XyZ.y = xyz.y; XyZ.z = XYZ.z;

			center.x = (xyz.x + XYZ.x) * f32(0.5);
			center.y = (xyz.y + XYZ.y) * f32(0.5);
			center.z = (xyz.z + XYZ.z) * f32(0.5);
			extents.x = XYZ.x - center.x;
			extents.y = XYZ.y - center.y;
			extents.z = XYZ.z - center.z;
		}

		bool IntersectsTriangle(const V3& tp0, const V3& tp1, const V3& tp2);
	};

	V3 LinePlaneIntersection(const V3& l0, const V3& l1, const V3& planePosition, const V3& planeNormal);
}
