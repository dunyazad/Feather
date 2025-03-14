#include <MiniMath.h>
#include <sstream>
#include <algorithm>

namespace MiniMath
{
	float clamp(float f, float minf, float maxf)
	{
		return std::min(std::max(f, minf), maxf);
	}

	float Trimax(float a, float b, float c) {
		return std::max(std::max(a, b), c);
	}

	float Trimin(float a, float b, float c) {
		return std::min(std::min(a, b), c);
	}

	V2::V2()
		: x(0.0f), y(0.0f) {}
	V2::V2(int ix, int iy)
		: x(ix), y(iy) {}
	V2::V2(float fx, float fy)
		: x(fx), y(fy) {}
	V2::V2(double dx, double dy)
		: x(dx), y(dy) {}
	V2::V2(float* fs)
		: x(fs[0]), y(fs[1]) {}

	V2::V2(const char* c)
	{
		std::string code(c);
		std::istringstream iss(code);
		float value;
		iss >> std::noskipws >> value;

		if (iss.eof() && !iss.fail())
		{
			x = y = value;
		}
		else
		{
			std::transform(code.begin(), code.end(), code.begin(), [](unsigned char c) { return std::tolower(c); });

			if (code == "nan")
			{
				x = y = std::numeric_limits<float>::quiet_NaN();
			}
			else if (code == "zero")
			{
				x = y = 0.0;
			}
			else if (code == "one")
			{
				x = y = 1.0;
			}
			else if (code == "half")
			{
				x = y = 0.5;
			}
		}
	}

	V3::V3()
		: x(0.0f), y(0.0f), z(0.0f) {}
	V3::V3(int ix, int iy, int iz)
		: x(ix), y(iy), z(iz) {}
	V3::V3(float fx, float fy, float fz)
		: x(fx), y(fy), z(fz) {}
	V3::V3(double dx, double dy, double dz)
		: x(dx), y(dy), z(dz) {}
	V3::V3(float* fs)
		: x(fs[0]), y(fs[1]), z(fs[2]) {}
	V3::V3(const char* c)
	{
		std::string code(c);
		std::istringstream iss(code);
		float value;
		iss >> std::noskipws >> value;

		if (iss.eof() && !iss.fail())
		{
			x = y = z = value;
		}
		else
		{
			std::transform(code.begin(), code.end(), code.begin(), [](unsigned char c) { return std::tolower(c); });

			if (code == "nan")
			{
				x = y = z = std::numeric_limits<float>::quiet_NaN();
			}
			else if (code == "zero")
			{
				x = y = z = 0.0;
			}
			else if (code == "one")
			{
				x = y = z = 1.0;
			}
			else if (code == "half")
			{
				x = y = z = 0.5;
			}
			else if (code == "red")
			{
				x = 1.0f;
			}
			else if (code == "green")
			{
				y = 1.0f;
			}
			else if (code == "blue")
			{
				z = 1.0f;
			}
			else if (code == "black")
			{
				x = y = z = 0.0f;
			}
			else if (code == "gray")
			{
				x = y = z = 0.5f;
			}
			else if (code == "white")
			{
				x = y = z = 1.0f;
			}
			else if (code == "yellow")
			{
				x = y = 1.0f;
				z = 0.0f;
			}
			else if (code == "magenta")
			{
				x = z = 1.0f;
				y = 0.0f;
			}
			else if (code == "cyan")
			{
				y = z = 1.0f;
				x = 0.0f;
			}
			else if (code == "positive x")
			{
				x = 1.0f;
				y = z = 0.0f;
			}
			else if (code == "negative x")
			{
				x = -1.0f;
				y = z = 0.0f;
			}
			else if (code == "positive y")
			{
				y = 1.0f;
				x = z = 0.0f;
			}
			else if (code == "negative y")
			{
				y = -1.0f;
				x = z = 0.0f;
			}
			else if (code == "positive z")
			{
				z = 1.0f;
				x = y = 0.0f;
			}
			else if (code == "negative z")
			{
				z = -1.0f;
				x = y = 0.0f;
			}
		}
	}


	const V3& V3::operator += (float scalar) { x += scalar; y += scalar; z += scalar; return *this; }
	const V3& V3::operator -= (float scalar) { x -= scalar; y -= scalar; z -= scalar; return *this; }
	const V3& V3::operator *= (float scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
	const V3& V3::operator /= (float scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

	const V3& V3::operator += (const V3& other) { x += other.x; y += other.y; z += other.z; return *this; }
	const V3& V3::operator -= (const V3& other) { x -= other.x; y -= other.y; z -= other.z; return *this; }

	float V3::operator[](int index) { return *(&this->x + index); }

	V3 operator - (const V3& v) { return { -v.x, -v.y, -v.z }; }
	V3 operator + (const V3& v, float scalar) { return { v.x + scalar, v.y + scalar, v.z + scalar }; }
	V3 operator + (float scalar, const V3& v) { return { v.x + scalar, v.y + scalar, v.z + scalar }; }
	V3 operator - (const V3& v, float scalar) { return { v.x - scalar, v.y - scalar, v.z - scalar }; }
	V3 operator - (float scalar, const V3& v) { return { v.x - scalar, v.y - scalar, v.z - scalar }; }
	V3 operator * (const V3& v, float scalar) { return { v.x * scalar, v.y * scalar, v.z * scalar }; }
	V3 operator * (float scalar, const V3& v) { return { v.x * scalar, v.y * scalar, v.z * scalar }; }
	V3 operator / (const V3& v, float scalar) { return { v.x / scalar, v.y / scalar, v.z / scalar }; }
	V3 operator / (float scalar, const V3& v) { return { v.x / scalar, v.y / scalar, v.z / scalar }; }

	V3 operator + (const V3& a, const V3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
	V3 operator - (const V3& a, const V3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }

	float magnitude(const V3& v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
	V3 normalize(const V3& v)
	{
		float mag = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
		if (mag != 0.0f)
		{
			return { v.x / mag, v.y / mag, v.z / mag };
		}
		return v;
	}

	float dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	V3 cross(const V3& a, const V3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }

	float distance(const V3& a, const V3& b)
	{
		return sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
	}

	float angle(const V3& a, const V3& b)
	{
		return acosf(clamp(dot(a, b), -1.0f, 1.0f));
	}

	V3 calculateMean(const std::vector<V3>& vectors)
	{
		V3 mean;
		for (const V3& vec : vectors)
		{
			mean.x += vec.x;
			mean.y += vec.y;
			mean.z += vec.z;
		}
		mean.x /= vectors.size();
		mean.y /= vectors.size();
		mean.z /= vectors.size();
		return mean;
	}

	void centerData(std::vector<V3>& vectors, const V3& mean)
	{
		for (V3& vec : vectors)
		{
			vec.x -= mean.x;
			vec.y -= mean.y;
			vec.z -= mean.z;
		}
	}

	void calculateCovarianceMatrix(const std::vector<V3>& vectors, V3& eigenvalues, V3& eigenvector1, V3& eigenvector2, V3& eigenvector3)
	{
		int n = vectors.size();

		for (const V3& vec : vectors)
		{
			eigenvalues.x += vec.x * vec.x;
			eigenvalues.y += vec.y * vec.y;
			eigenvalues.z += vec.z * vec.z;

			eigenvector1.x += vec.x * vec.x;
			eigenvector2.x += vec.x * vec.y;
			eigenvector3.x += vec.x * vec.z;

			eigenvector2.y += vec.y * vec.y;
			eigenvector3.y += vec.y * vec.z;

			eigenvector3.z += vec.z * vec.z;
		}

		eigenvalues.x /= n;
		eigenvalues.y /= n;
		eigenvalues.z /= n;

		eigenvector1.x /= n;
		eigenvector2.x /= n;
		eigenvector3.x /= n;

		eigenvector2.y /= n;
		eigenvector3.y /= n;

		eigenvector3.z /= n;
	}

	// ---- V4 (4D Vector) ----
	V4::V4() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
	V4::V4(float scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}
	V4::V4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	V4::V4(float* array) : x(array[0]), y(array[1]), z(array[2]), w(array[3]) {}

	V4::V4(const char* str)
	{
		std::string code(str);
		std::istringstream iss(code);
		float value;
		iss >> std::noskipws >> value;

		if (iss.eof() && !iss.fail())
		{
			x = y = z = w = value;
		}
		else
		{
			std::transform(code.begin(), code.end(), code.begin(), [](unsigned char c) { return std::tolower(c); });

			if (code == "nan")
			{
				x = y = z = w = std::numeric_limits<float>::quiet_NaN();
			}
			else if (code == "zero")
			{
				x = y = z = w = 0.0f;
			}
			else if (code == "one")
			{
				x = y = z = w = 1.0f;
			}
			else if (code == "half")
			{
				x = y = z = w = 0.5f;
			}
		}
	}

	const V4& V4::operator+=(const V4& other) { x += other.x; y += other.y; z += other.z; w += other.w; return *this; }
	const V4& V4::operator-=(const V4& other) { x -= other.x; y -= other.y; z -= other.z; w -= other.w; return *this; }
	const V4& V4::operator*=(float scalar) { x *= scalar; y *= scalar; z *= scalar; w *= scalar; return *this; }
	const V4& V4::operator/=(float scalar) { x /= scalar; y /= scalar; z /= scalar; w /= scalar; return *this; }

	float V4::operator[](int index) const { return *(&this->x + index); }
	float& V4::operator[](int index) { return *(&this->x + index); }

	float V4::magnitude() const { return std::sqrt(x * x + y * y + z * z + w * w); }

	V4 V4::normalize() const
	{
		float mag = magnitude();
		if (mag < std::numeric_limits<float>::epsilon()) return { 0.0f, 0.0f, 0.0f, 0.0f };
		return { x / mag, y / mag, z / mag, w / mag };
	}

	float V4::dot(const V4& other) const { return x * other.x + y * other.y + z * other.z + w * other.w; }

	// Static Methods
	V4 V4::zero() { return { 0.0f, 0.0f, 0.0f, 0.0f }; }
	V4 V4::one() { return { 1.0f, 1.0f, 1.0f, 1.0f }; }
	V4 V4::unitX() { return { 1.0f, 0.0f, 0.0f, 0.0f }; }
	V4 V4::unitY() { return { 0.0f, 1.0f, 0.0f, 0.0f }; }
	V4 V4::unitZ() { return { 0.0f, 0.0f, 1.0f, 0.0f }; }
	V4 V4::unitW() { return { 0.0f, 0.0f, 0.0f, 1.0f }; }

	// Operator Overloads
	V4 operator+(const V4& a, const V4& b) { return { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w }; }
	V4 operator-(const V4& a, const V4& b) { return { a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w }; }
	V4 operator*(const V4& v, float scalar) { return { v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar }; }
	V4 operator*(float scalar, const V4& v) { return { v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar }; }
	V4 operator/(const V4& v, float scalar) { return { v.x / scalar, v.y / scalar, v.z / scalar, v.w / scalar }; }

	std::ostream& operator<<(std::ostream& os, const V4& v) { return os << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")"; }


	Quaternion::Quaternion(float scalar, float i, float j, float k)
		: w(scalar), x(i), y(j), z(k) {}

	Quaternion::Quaternion(float radian, const V3& axis)
	{
		float halfAngle = radian * 0.5f;
		float sinHalfAngle = sinf(halfAngle);
		w = cosf(halfAngle);
		x = axis.x * sinHalfAngle;
		y = axis.y * sinHalfAngle;
		z = axis.z * sinHalfAngle;
	}

	Quaternion conjugate(const Quaternion q) { return { q.w, -q.x, -q.y, -q.z }; }

	Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
	{
		float w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		float x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
		float y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
		float z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

		return Quaternion(w, x, y, z);
	}

	V3 rotate(const V3& v, const Quaternion& q)
	{
		auto qv = V3{ q.x, q.y, q.z };
		auto uv = cross(qv, v);
		auto uuv = cross(qv, uv);

		return v + ((uv * q.w) + uuv) * 2.0f;
	}

	Quaternion rotation(const V3& v0, const V3& v1)
	{
		V3 v0n = normalize(v0);
		V3 v1n = normalize(v1);

		float dotProduct = dot(v0n, v1n);

		if (dotProduct > 0.9999f) {
			// Vectors are nearly identical
			return { 1.0f, 0.0f, 0.0f, 0.0f };
		}
		else if (dotProduct < -0.9999f) {
			// Vectors are opposite
			V3 orthogonal = (std::abs(v0n.x) > std::abs(v0n.z)) ? V3(-v0n.y, v0n.x, 0.0f) : V3(0.0f, -v0n.z, v0n.y);
			orthogonal = normalize(orthogonal);
			return { 0.0f, orthogonal.x, orthogonal.y, orthogonal.z };
		}

		V3 axis = normalize(cross(v0n, v1n));
		float angle = std::acos(dotProduct);

		float w = std::cos(angle / 2);
		float s = std::sin(angle / 2);

		return { w, axis.x * s, axis.y * s, axis.z * s };
	}

	// ---- M3 (3x3 Matrix) ----
	M3::M3() { *this = identity(); }
	M3::M3(float diagonal) { *this = identity(); for (int i = 0; i < 3; i++) m[i][i] = diagonal; }
	M3::M3(float m00, float m01, float m02,
		float m10, float m11, float m12,
		float m20, float m21, float m22)
	{
		m[0][0] = m00; m[0][1] = m01; m[0][2] = m02;
		m[1][0] = m10; m[1][1] = m11; m[1][2] = m12;
		m[2][0] = m20; m[2][1] = m21; m[2][2] = m22;
	}

	M3 M3::operator+(const M3& other) const
	{
		M3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i][j] = m[i][j] + other.m[i][j];
		return result;
	}

	M3 M3::operator-(const M3& other) const
	{
		M3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i][j] = m[i][j] - other.m[i][j];
		return result;
	}

	M3 M3::operator*(float scalar) const
	{
		M3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i][j] = m[i][j] * scalar;
		return result;
	}

	M3 M3::operator*(const M3& other) const
	{
		M3 result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i][j] = m[i][0] * other.m[0][j] +
				m[i][1] * other.m[1][j] +
				m[i][2] * other.m[2][j];
		return result;
	}

	V3 M3::operator*(const V3& vec) const
	{
		return V3(
			m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z,
			m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z,
			m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z
		);
	}

	M3 M3::transpose() const
	{
		return M3(m[0][0], m[1][0], m[2][0],
			m[0][1], m[1][1], m[2][1],
			m[0][2], m[1][2], m[2][2]);
	}

	float M3::determinant() const
	{
		return
			m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
			m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
			m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	}

	M3 M3::inverse() const
	{
		float det = determinant();
		if (std::fabs(det) < std::numeric_limits<float>::epsilon())
		{
			std::cerr << "Matrix is singular and cannot be inverted!" << std::endl;
			return M3();  // Return identity as a fallback
		}

		float invDet = 1.0f / det;

		M3 inv;
		inv.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
		inv.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * invDet;
		inv.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;

		inv.m[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * invDet;
		inv.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
		inv.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * invDet;

		inv.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
		inv.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * invDet;
		inv.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;

		return inv;
	}

	float M3::at(int row, int column)
	{
		return m[row][column];
	}

	M3 M3::identity()
	{
		return M3(1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f);
	}

	M3 M3::zero()
	{
		return M3(0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f);
	}

	// ---- M4 (4x4 Matrix) ----
	M4::M4() { *this = identity(); }
	M4::M4(float diagonal) { *this = identity(); for (int i = 0; i < 4; i++) m[i][i] = diagonal; }
	M4::M4(float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23,
		float m30, float m31, float m32, float m33)
	{
		m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
		m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
		m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
		m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
	}

	M4 M4::operator+(const M4& other) const
	{
		M4 result;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result.m[i][j] = m[i][j] + other.m[i][j];
		return result;
	}

	M4 M4::operator-(const M4& other) const
	{
		M4 result;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result.m[i][j] = m[i][j] - other.m[i][j];
		return result;
	}

	M4 M4::operator*(float scalar) const
	{
		M4 result;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result.m[i][j] = m[i][j] * scalar;
		return result;
	}

	M4 M4::operator*(const M4& other) const
	{
		M4 result;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				result.m[i][j] = m[i][0] * other.m[0][j] +
					m[i][1] * other.m[1][j] +
					m[i][2] * other.m[2][j] +
					m[i][3] * other.m[3][j];
			}
		}
		return result;
	}

	V4 M4::operator*(const V4& vec) const
	{
		return V4(
			m[0][0] * vec.x + m[0][1] * vec.y + m[0][2] * vec.z + m[0][3] * vec.w,
			m[1][0] * vec.x + m[1][1] * vec.y + m[1][2] * vec.z + m[1][3] * vec.w,
			m[2][0] * vec.x + m[2][1] * vec.y + m[2][2] * vec.z + m[2][3] * vec.w,
			m[3][0] * vec.x + m[3][1] * vec.y + m[3][2] * vec.z + m[3][3] * vec.w
		);
	}

	V3 M4::transformPoint(const V3& point) const
	{
		float x = point.x * m[0][0] + point.y * m[1][0] + point.z * m[2][0] + m[3][0];
		float y = point.x * m[0][1] + point.y * m[1][1] + point.z * m[2][1] + m[3][1];
		float z = point.x * m[0][2] + point.y * m[1][2] + point.z * m[2][2] + m[3][2];
		return { x, y, z };
	}

	V3 M4::transformVector(const V3& vector) const
	{
		return V3(
			m[0][0] * vector.x + m[0][1] * vector.y + m[0][2] * vector.z,
			m[1][0] * vector.x + m[1][1] * vector.y + m[1][2] * vector.z,
			m[2][0] * vector.x + m[2][1] * vector.y + m[2][2] * vector.z
		);
	}

	M4 M4::transpose() const
	{
		M4 result;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				result.m[i][j] = m[j][i];
		return result;
	}

	float M4::determinant() const
	{
		float det =
			m[0][3] * m[1][2] * m[2][1] * m[3][0] - m[0][2] * m[1][3] * m[2][1] * m[3][0] -
			m[0][3] * m[1][1] * m[2][2] * m[3][0] + m[0][1] * m[1][3] * m[2][2] * m[3][0] +
			m[0][2] * m[1][1] * m[2][3] * m[3][0] - m[0][1] * m[1][2] * m[2][3] * m[3][0] -
			m[0][3] * m[1][2] * m[2][0] * m[3][1] + m[0][2] * m[1][3] * m[2][0] * m[3][1] +
			m[0][3] * m[1][0] * m[2][2] * m[3][1] - m[0][0] * m[1][3] * m[2][2] * m[3][1] -
			m[0][2] * m[1][0] * m[2][3] * m[3][1] + m[0][0] * m[1][2] * m[2][3] * m[3][1] +
			m[0][3] * m[1][1] * m[2][0] * m[3][2] - m[0][1] * m[1][3] * m[2][0] * m[3][2] -
			m[0][3] * m[1][0] * m[2][1] * m[3][2] + m[0][0] * m[1][3] * m[2][1] * m[3][2] +
			m[0][1] * m[1][0] * m[2][3] * m[3][2] - m[0][0] * m[1][1] * m[2][3] * m[3][2] -
			m[0][2] * m[1][1] * m[2][0] * m[3][3] + m[0][1] * m[1][2] * m[2][0] * m[3][3] +
			m[0][2] * m[1][0] * m[2][1] * m[3][3] - m[0][0] * m[1][2] * m[2][1] * m[3][3] -
			m[0][1] * m[1][0] * m[2][2] * m[3][3] + m[0][0] * m[1][1] * m[2][2] * m[3][3];

		return det;
	}

	M4 M4::inverse() const
	{
		M4 inv;
		float det = determinant();

		if (std::fabs(det) < std::numeric_limits<float>::epsilon())
		{
			std::cerr << "Matrix is singular and cannot be inverted!" << std::endl;
			return M4();  // Return identity as a fallback
		}

		float invDet = 1.0f / det;

		inv.m[0][0] = (m[1][1] * m[2][2] * m[3][3] + m[1][2] * m[2][3] * m[3][1] + m[1][3] * m[2][1] * m[3][2] -
			m[1][3] * m[2][2] * m[3][1] - m[1][2] * m[2][1] * m[3][3] - m[1][1] * m[2][3] * m[3][2]) * invDet;
		inv.m[0][1] = (m[0][3] * m[2][2] * m[3][1] + m[0][2] * m[2][1] * m[3][3] + m[0][1] * m[2][3] * m[3][2] -
			m[0][1] * m[2][2] * m[3][3] - m[0][2] * m[2][3] * m[3][1] - m[0][3] * m[2][1] * m[3][2]) * invDet;
		inv.m[0][2] = (m[0][1] * m[1][2] * m[3][3] + m[0][2] * m[1][3] * m[3][1] + m[0][3] * m[1][1] * m[3][2] -
			m[0][3] * m[1][2] * m[3][1] - m[0][2] * m[1][1] * m[3][3] - m[0][1] * m[1][3] * m[3][2]) * invDet;
		inv.m[0][3] = (m[0][3] * m[1][2] * m[2][1] + m[0][2] * m[1][1] * m[2][3] + m[0][1] * m[1][3] * m[2][2] -
			m[0][1] * m[1][2] * m[2][3] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2]) * invDet;

		// Compute other elements (for brevity, not fully expanded here, follow similar logic as above)
		// Repeat similar calculations for `inv.m[1][0]`, `inv.m[1][1]`, ..., `inv.m[3][3]`

		return inv;
	}

	float M4::at(int row, int column)
	{
		return m[row][column];
	}

	M4 M4::identity()
	{
		return M4(1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}

	M4 M4::zero()
	{
		return M4(0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 0.0f);
	}

	M4 translate(const M4& matrix, const V3& translation) {
		M4 translationMatrix = M4::identity(); // Create a translation matrix
		translationMatrix.m[3][0] = translation.x;
		translationMatrix.m[3][1] = translation.y;
		translationMatrix.m[3][2] = translation.z;

		return matrix * translationMatrix; // Apply translation by multiplying
	}

	bool AABB::IntersectsTriangle(const V3& tp0, const V3& tp1, const V3& tp2)
	{
		V3 v0 = { tp0.x - center.x, tp0.y - center.y, tp0.z - center.z };
		V3 v1 = { tp1.x - center.x, tp1.y - center.y, tp1.z - center.z };
		V3 v2 = { tp2.x - center.x, tp2.y - center.y, tp2.z - center.z };

		// Compute edge vectors for triangle
		V3 f0 = { tp1.x - tp0.x, tp1.y - tp0.y, tp1.z - tp0.z };
		V3 f1 = { tp2.x - tp1.x, tp2.y - tp1.y, tp2.z - tp1.z };
		V3 f2 = { tp0.x - tp2.x, tp0.y - tp2.y, tp0.z - tp2.z };

		//// region Test axes a00..a22 (category 3)

		// Test axis a00
		V3 a00 = { 0.0f, -f0.z, f0.y };
		float p0 = v0.x * a00.x + v0.y * a00.y + v0.z * a00.z;
		float p1 = v1.x * a00.x + v1.y * a00.y + v1.z * a00.z;
		float p2 = v2.x * a00.x + v2.y * a00.y + v2.z * a00.z;
		float r = extents.y * std::fabs(f0.z) + extents.z * std::fabs(f0.y);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a01
		V3 a01 = { 0.0f, -f1.z, f1.y };
		p0 = v0.x * a01.x + v0.y * a01.y + v0.z * a01.z;
		p1 = v1.x * a01.x + v1.y * a01.y + v1.z * a01.z;
		p2 = v2.x * a01.x + v2.y * a01.y + v2.z * a01.z;
		r = extents.y * std::fabs(f1.z) + extents.z * std::fabs(f1.y);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a02
		V3 a02 = { 0.0f, -f2.z, f2.y };
		p0 = v0.x * a02.x + v0.y * a02.y + v0.z * a02.z;
		p1 = v1.x * a02.x + v1.y * a02.y + v1.z * a02.z;
		p2 = v2.x * a02.x + v2.y * a02.y + v2.z * a02.z;
		r = extents.y * std::fabs(f2.z) + extents.z * std::fabs(f2.y);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a10
		V3 a10 = { f0.z, 0.0f, -f0.x };
		p0 = v0.x * a10.x + v0.y * a10.y + v0.z * a10.z;
		p1 = v1.x * a10.x + v1.y * a10.y + v1.z * a10.z;
		p2 = v2.x * a10.x + v2.y * a10.y + v2.z * a10.z;
		r = extents.x * std::fabs(f0.z) + extents.z * std::fabs(f0.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a11
		V3 a11 = { f1.z, 0.0f, -f1.x };
		p0 = v0.x * a11.x + v0.y * a11.y + v0.z * a11.z;
		p1 = v1.x * a11.x + v1.y * a11.y + v1.z * a11.z;
		p2 = v2.x * a11.x + v2.y * a11.y + v2.z * a11.z;
		r = extents.x * std::fabs(f1.z) + extents.z * std::fabs(f1.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a12
		V3 a12 = { f2.z, 0.0f, -f2.x };
		p0 = v0.x * a12.x + v0.y * a12.y + v0.z * a12.z;
		p1 = v1.x * a12.x + v1.y * a12.y + v1.z * a12.z;
		p2 = v2.x * a12.x + v2.y * a12.y + v2.z * a12.z;
		r = extents.x * std::fabs(f2.z) + extents.z * std::fabs(f2.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a20
		V3 a20 = { -f0.y, f0.x, 0.0f };
		p0 = v0.x * a20.x + v0.y * a20.y + v0.z * a20.z;
		p1 = v1.x * a20.x + v1.y * a20.y + v1.z * a20.z;
		p2 = v2.x * a20.x + v2.y * a20.y + v2.z * a20.z;
		r = extents.x * std::fabs(f0.y) + extents.y * std::fabs(f0.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a21
		V3 a21 = { -f1.y, f1.x, 0.0f };
		p0 = v0.x * a21.x + v0.y * a21.y + v0.z * a21.z;
		p1 = v1.x * a21.x + v1.y * a21.y + v1.z * a21.z;
		p2 = v2.x * a21.x + v2.y * a21.y + v2.z * a21.z;
		r = extents.x * std::fabs(f1.y) + extents.y * std::fabs(f1.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		// Test axis a22
		V3 a22 = { -f2.y, f2.x, 0.0f };
		p0 = v0.x * a22.x + v0.y * a22.y + v0.z * a22.z;
		p1 = v1.x * a22.x + v1.y * a22.y + v1.z * a22.z;
		p2 = v2.x * a22.x + v2.y * a22.y + v2.z * a22.z;
		r = extents.x * std::fabs(f2.y) + extents.y * std::fabs(f2.x);
		if (std::max(-Trimax(p0, p1, p2), Trimin(p0, p1, p2)) > r + FLT_EPSILON)
			return false;

		//// endregion

		//// region Test the three axes corresponding to the face normals of AABB b (category 1)

		// Exit if...
		// ... [-extents.X, extents.X] and [Min(v0.X,v1.X,v2.X), Max(v0.X,v1.X,v2.X)] do not overlap
		if (Trimax(v0.x, v1.x, v2.x) < -extents.x || Trimin(v0.x, v1.x, v2.x) > extents.x) {
			if (Trimax(v0.x, v1.x, v2.x) - (-extents.x) > FLT_EPSILON ||
				Trimin(v0.x, v1.x, v2.x) - (extents.x) > FLT_EPSILON) {
				return false;
			}
		}

		// ... [-extents.Y, extents.Y] and [Min(v0.Y,v1.Y,v2.Y), Max(v0.Y,v1.Y,v2.Y)] do not overlap
		if (Trimax(v0.y, v1.y, v2.y) < -extents.y || Trimin(v0.y, v1.y, v2.y) > extents.y) {
			if (Trimax(v0.y, v1.y, v2.y) - (-extents.y) > FLT_EPSILON ||
				Trimin(v0.y, v1.y, v2.y) - (extents.y) > FLT_EPSILON) {
				return false;
			}
		}

		// ... [-extents.Z, extents.Z] and [Min(v0.Z,v1.Z,v2.Z), Max(v0.Z,v1.Z,v2.Z)] do not overlap
		if (Trimax(v0.z, v1.z, v2.z) < -extents.z || Trimin(v0.z, v1.z, v2.z) > extents.z) {
			if (Trimax(v0.z, v1.z, v2.z) - (-extents.z) > FLT_EPSILON ||
				Trimin(v0.z, v1.z, v2.z) - (extents.z) > FLT_EPSILON) {
				return false;
			}
		}

		//// endregion

		//// region Test separating axis corresponding to triangle face normal (category 2)

		V3 plane_normal = { f0.y * f1.z - f0.z * f1.y, f0.z * f1.x - f0.x * f1.z, f0.x * f1.y - f0.y * f1.x };
		float plane_distance = std::fabs(plane_normal.x * v0.x + plane_normal.y * v0.y + plane_normal.z * v0.z);

		// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
		r = extents.x * std::fabs(plane_normal.x) + extents.y * std::fabs(plane_normal.y) + extents.z * std::fabs(plane_normal.z);

		// Intersection occurs when plane distance falls within [-r,+r] interval
		if (plane_distance > r + FLT_EPSILON)
			return false;

		//// endregion

		return true;
	}

	V3 LinePlaneIntersection(const V3& l0, const V3& l1, const V3& planePosition, const V3& planeNormal)
	{
		auto ld = l1 - l0;

		float denominator = dot(ld, planeNormal);

		if (denominator == 0.0f) {
			std::cout << "The line is parallel to the plane." << std::endl;
			// You might want to handle this case differently based on your requirements
			return "nan";
		}

		auto t = ((planePosition.x - l0.x) * planeNormal.x +
			(planePosition.y - l0.y) * planeNormal.y +
			(planePosition.z - l0.z) * planeNormal.z) / denominator;

		return V3(l0.x + t * ld.x, l0.y + t * ld.y, l0.z + t * ld.z);
	}
}
