#pragma once
#include <arm_math.h>

#ifdef __cplusplus
namespace vector3d
{
	class Vector3D
	{
	public:
		// 构造函数
		Vector3D();
		Vector3D(float x, float y, float z);
		Vector3D(const float32_t* data);  // 从数组初始化

		// 析构函数
		virtual ~Vector3D() {}

		// 访问x、y、z分量
		float x() const { return data_[0]; }
		float y() const { return data_[1]; }
		float z() const { return data_[2]; }
		float& x() { return data_[0]; }
		float& y() { return data_[1]; }
		float& z() { return data_[2]; }

		// 获取内部数据指针（适配DSP库）
		const float32_t* data() const { return data_; }
		float32_t* data() { return data_; }

		// 向量运算
		Vector3D operator+(const Vector3D& other) const;
		Vector3D operator-(const Vector3D& other) const;
		Vector3D operator-() const;
		Vector3D operator*(float scalar) const;
		Vector3D operator/(float scalar) const;

		Vector3D& operator+=(const Vector3D& other);
		Vector3D& operator-=(const Vector3D& other);
		Vector3D& operator*=(float scalar);
		Vector3D& operator/=(float scalar);

		// 点积运算
		float dot(const Vector3D& other) const;

		// 叉积运算（三维向量叉积结果为向量）
		Vector3D cross(const Vector3D& other) const;

		// 向量长度（模）
		float length() const;

		// 向量长度的平方（避免开方运算）
		float lengthSquared() const;

		// 归一化（单位向量）
		Vector3D normalize() const;

		// 绕x轴旋转（右手坐标系，逆时针旋转theta弧度）
		Vector3D rotateX(float theta) const;

		// 绕y轴旋转（右手坐标系，逆时针旋转theta弧度）
		Vector3D rotateY(float theta) const;

		// 绕z轴旋转（右手坐标系，逆时针旋转theta弧度）
		Vector3D rotateZ(float theta) const;

		// 计算两点之间的距离
		static float distance(const Vector3D& a, const Vector3D& b);

		// 线性插值
		static Vector3D lerp(const Vector3D& a, const Vector3D& b, float t);

		// 计算两点之间的距离平方
		static float distanceSquared(const Vector3D& a, const Vector3D& b);

	protected:
		float32_t data_[3];  // 存储x、y、z分量，适配DSP库函数

	private:
		// 辅助函数：检查标量是否接近零
		static bool isZero(float scalar)
		{
			return (scalar < 0 ? -scalar : scalar) < 1e-6f;
		}
	};
}
#endif
