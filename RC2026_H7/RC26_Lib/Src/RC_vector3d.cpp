#include "RC_vector3d.h"

namespace vector3d
{
	// 构造函数实现
	Vector3D::Vector3D() : data_{0.0f, 0.0f, 0.0f} {}

	Vector3D::Vector3D(float x, float y, float z) : data_{x, y, z} {}

	Vector3D::Vector3D(const float32_t* data) : data_{data[0], data[1], data[2]} {}

	/*----------------------------------------------------------------------------------*/
	// 运算符实现
	Vector3D Vector3D::operator+(const Vector3D& other) const
	{
		float32_t result[3];
		arm_add_f32(data_, other.data_, result, 3);
		return Vector3D(result);
	}

	Vector3D Vector3D::operator-(const Vector3D& other) const
	{
		float32_t result[3];
		arm_sub_f32(data_, other.data_, result, 3);
		return Vector3D(result);
	}

	Vector3D Vector3D::operator-() const
	{
		return Vector3D(-data_[0], -data_[1], -data_[2]);
	}

	Vector3D Vector3D::operator*(float scalar) const
	{
		float32_t result[3];
		arm_scale_f32(data_, scalar, result, 3);
		return Vector3D(result);
	}

	Vector3D Vector3D::operator/(float scalar) const
	{
		if (isZero(scalar)) return Vector3D();
		return *this * (1.0f / scalar);
	}

	Vector3D& Vector3D::operator+=(const Vector3D& other)
	{
		arm_add_f32(data_, other.data_, data_, 3);
		return *this;
	}

	Vector3D& Vector3D::operator-=(const Vector3D& other)
	{
		arm_sub_f32(data_, other.data_, data_, 3);
		return *this;
	}

	Vector3D& Vector3D::operator*=(float scalar)
	{
		arm_scale_f32(data_, scalar, data_, 3);
		return *this;
	}

	Vector3D& Vector3D::operator/=(float scalar)
	{
		if (isZero(scalar)) return *this;
		return *this *= (1.0f / scalar);
	}

	/*----------------------------------------------------------------------------------*/
	// 点积运算
	float Vector3D::dot(const Vector3D& other) const
	{
		float32_t result;
		arm_dot_prod_f32(data_, other.data_, 3, &result);
		return result;
	}

	// 叉积运算（三维向量叉积结果为向量）
	Vector3D Vector3D::cross(const Vector3D& other) const
	{
		return Vector3D(
			data_[1] * other.data_[2] - data_[2] * other.data_[1],
			data_[2] * other.data_[0] - data_[0] * other.data_[2],
			data_[0] * other.data_[1] - data_[1] * other.data_[0]
		);
	}

	// 向量长度（模）
	float Vector3D::length() const
	{
		float32_t lenSq = lengthSquared();
		float32_t result;
		arm_sqrt_f32(lenSq, &result);
		return result;
	}

	// 向量长度的平方（避免开方运算）
	float Vector3D::lengthSquared() const
	{
		return dot(*this);
	}

	// 归一化（单位向量）
	Vector3D Vector3D::normalize() const
	{
		float len = length();
		if (isZero(len)) return Vector3D();
		return *this * (1.0f / len);
	}

	// 绕x轴旋转（右手坐标系，逆时针旋转theta弧度）
	Vector3D Vector3D::rotateX(float theta) const
	{
		float32_t sinTheta = arm_sin_f32(theta);
		float32_t cosTheta = arm_cos_f32(theta);
		
		return Vector3D(
			data_[0],
			data_[1] * cosTheta - data_[2] * sinTheta,
			data_[1] * sinTheta + data_[2] * cosTheta
		);
	}

	// 绕y轴旋转（右手坐标系，逆时针旋转theta弧度）
	Vector3D Vector3D::rotateY(float theta) const
	{
		float32_t sinTheta = arm_sin_f32(theta);
		float32_t cosTheta = arm_cos_f32(theta);
		
		return Vector3D(
			data_[0] * cosTheta + data_[2] * sinTheta,
			data_[1],
			-data_[0] * sinTheta + data_[2] * cosTheta
		);
	}

	// 绕z轴旋转（右手坐标系，逆时针旋转theta弧度）
	Vector3D Vector3D::rotateZ(float theta) const
	{
		float32_t sinTheta = arm_sin_f32(theta);
		float32_t cosTheta = arm_cos_f32(theta);
		
		return Vector3D(
			data_[0] * cosTheta - data_[1] * sinTheta,
			data_[0] * sinTheta + data_[1] * cosTheta,
			data_[2]
		);
	}

	// 计算两点之间的距离
	float Vector3D::distance(const Vector3D& a, const Vector3D& b)
	{
		return (b - a).length();
	}

	// 线性插值
	Vector3D Vector3D::lerp(const Vector3D& a, const Vector3D& b, float t)
	{
		// 限制t在[0,1]范围内
		if (t < 0.0f) t = 0.0f;
		if (t > 1.0f) t = 1.0f;
		return a + (b - a) * t;
	}

	// 计算两点之间的距离平方
	float Vector3D::distanceSquared(const Vector3D& a, const Vector3D& b)
	{
		return (b - a).lengthSquared();
	}
}