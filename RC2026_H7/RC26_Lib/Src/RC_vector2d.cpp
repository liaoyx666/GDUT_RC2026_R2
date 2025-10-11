#include "RC_vector2d.h"

namespace vector2d
{
	Vector2D::Vector2D() : data_{0.0f, 0.0f} {}

	Vector2D::Vector2D(float x, float y) : data_{x, y} {}

	Vector2D::Vector2D(const float32_t* data) : data_{data[0], data[1]} {}
	/*----------------------------------------------------------------------------------*/
	Vector2D Vector2D::operator+(const Vector2D& other) const
	{
		float32_t result[2];
		arm_add_f32(data_, other.data_, result, 2);
		return Vector2D(result);
	}

	Vector2D Vector2D::operator-(const Vector2D& other) const
	{
		float32_t result[2];
		arm_sub_f32(data_, other.data_, result, 2);
		return Vector2D(result);
	}
	
	Vector2D Vector2D::operator-() const
	{
		return Vector2D(-data_[0], -data_[1]);
	}

	Vector2D Vector2D::operator*(float scalar) const
	{
		float32_t result[2];
		arm_scale_f32(data_, scalar, result, 2);
		return Vector2D(result);
	}

	Vector2D Vector2D::operator/(float scalar) const
	{
		if (isZero(scalar)) return Vector2D();
		return *this * (1.0f / scalar);
	}

	Vector2D& Vector2D::operator+=(const Vector2D& other)
	{
		arm_add_f32(data_, other.data_, data_, 2);
		return *this;
	}

	Vector2D& Vector2D::operator-=(const Vector2D& other)
	{
		arm_sub_f32(data_, other.data_, data_, 2);
		return *this;
	}

	Vector2D& Vector2D::operator*=(float scalar)
	{
		arm_scale_f32(data_, scalar, data_, 2);
		return *this;
	}

	Vector2D& Vector2D::operator/=(float scalar)
	{
		if (isZero(scalar)) return *this;
		return *this *= (1.0f / scalar);
	}
	/*----------------------------------------------------------------------------------*/
	// 点积运算
	float Vector2D::dot(const Vector2D& other) const
	{
		float32_t result;
		arm_dot_prod_f32(data_, other.data_, 2, &result);
		return result;
	}

	// 叉积的标量结果（二维向量叉积的z分量）
	float Vector2D::cross(const Vector2D& other) const
	{
		return data_[0] * other.data_[1] - data_[1] * other.data_[0];
	}

	float Vector2D::length() const
	{
		float32_t lenSq = lengthSquared();
		float32_t result;
		arm_sqrt_f32(lenSq, &result);
		return result;
	}

	// 向量长度（模）
	float Vector2D::lengthSquared() const
	{
		return dot(*this);
	}

	// 归一化（单位向量）
	Vector2D Vector2D::normalize() const
	{
		float len = length();
		if (isZero(len)) return Vector2D();
		return *this * (1.0f / len);
	}

	// 旋转向量（逆时针旋转theta弧度）
	Vector2D Vector2D::rotate(float theta) const
	{
		float32_t sinTheta, cosTheta;
		
		sinTheta = arm_sin_f32(theta);
		cosTheta = arm_cos_f32(theta);
		
		return Vector2D(
			data_[0] * cosTheta - data_[1] * sinTheta,
			data_[0] * sinTheta + data_[1] * cosTheta
		);
	}

	// 获取垂直法向量（逆时针90度）
	Vector2D Vector2D::perpendicular() const
	{
		return Vector2D(-data_[1], data_[0]);
	}

	// 计算两点之间的距离
	float Vector2D::distance(const Vector2D& a, const Vector2D& b)
	{
		return (b - a).length();
	}

	// 线性插值
	Vector2D Vector2D::lerp(const Vector2D& a, const Vector2D& b, float t)
	{
		// 限制t在[0,1]范围内
		if (t < 0.0f) t = 0.0f;
		if (t > 1.0f) t = 1.0f;
		return a + (b - a) * t;
	}
	
	// 计算两点之间的距离平方
	float Vector2D::distanceSquared(const Vector2D& a, const Vector2D& b)
	{
		return (b - a).lengthSquared();
	}
	
	
	// -pi~pi
	float Vector2D::angleBetween(const Vector2D& a, const Vector2D& b)
	{
		float cross_val = a.cross(b);
		float dot_val = a.dot(b);

		if (isZero(dot_val) && isZero(cross_val))
		{
			return 0.f;
		}

		float result;
		arm_atan2_f32(cross_val, dot_val, &result);
		return result;
	}
	
}