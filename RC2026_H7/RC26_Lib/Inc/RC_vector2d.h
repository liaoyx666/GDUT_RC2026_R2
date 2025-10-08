#pragma once
#include <arm_math.h>

#ifdef __cplusplus
namespace vector2d
{
	class Vector2D
    {
    public:
		// 构造函数
		Vector2D();
		Vector2D(float x, float y);
		Vector2D(const float32_t* data);  // 从数组初始化
		
		// 析构函数
		virtual ~Vector2D() {}

		// 访问x和y分量
		float x() const { return data_[0]; }
		float y() const { return data_[1]; }
		float& x() { return data_[0]; }
		float& y() { return data_[1]; }

		// 获取内部数据指针（用于DSP函数）
		const float32_t* data() const { return data_; }
		float32_t* data() { return data_; }

		// 向量运算
		Vector2D operator+(const Vector2D& other) const;
		Vector2D operator-(const Vector2D& other) const;
		Vector2D operator-() const;
		Vector2D operator*(float scalar) const;
		Vector2D operator/(float scalar) const;

		Vector2D& operator+=(const Vector2D& other);
		Vector2D& operator-=(const Vector2D& other);
		Vector2D& operator*=(float scalar);
		Vector2D& operator/=(float scalar);

		// 点积运算
		float dot(const Vector2D& other) const;

		// 叉积的标量结果（二维向量叉积的z分量）
		float cross(const Vector2D& other) const;

		// 向量长度（模）
		float length() const;

		// 向量长度的平方（避免开方运算）
		float lengthSquared() const;

		// 归一化（单位向量）
		Vector2D normalize() const;

		// 旋转向量（逆时针旋转theta弧度）
		Vector2D rotate(float theta) const;

		// 获取垂直法向量（逆时针90度）
		Vector2D perpendicular() const;

		// 计算两点之间的距离
		static float distance(const Vector2D& a, const Vector2D& b);

		// 线性插值
		static Vector2D lerp(const Vector2D& a, const Vector2D& b, float t);
		
		// 计算两点之间的距离平方
		static float distanceSquared(const Vector2D& a, const Vector2D& b);

    protected:
		float32_t data_[2];  // 存储x和y分量，适配DSP库函数

    private:
		// 辅助函数：检查标量是否接近零
		static bool isZero(float scalar) {
			return (scalar < 0 ? -scalar : scalar) < 1e-6f;
		}
    };
}
#endif
