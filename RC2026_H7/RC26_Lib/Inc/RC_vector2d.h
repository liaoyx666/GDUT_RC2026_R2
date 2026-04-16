#pragma once
#include <arm_math.h>

#define VECTOR2D_USE_DSP

#ifdef __cplusplus
namespace vector2d
{
	class Vector2D
    {
    public:
		/*构造函数*/ 
		Vector2D() : data_{0.f, 0.f} {}
		Vector2D(float x, float y) : data_{x, y} {}
		Vector2D(const float32_t* data) : data_{data[0], data[1]} {}
		Vector2D(float theta_rad) // 逆时针
		{
			data_[0] = arm_cos_f32(theta_rad);
			data_[1] = arm_sin_f32(theta_rad);
		}
		
		/*析构函数*/
		~Vector2D() = default;

		// 访问x和y分量
		constexpr  float x() const { return data_[0]; }
		constexpr  float y() const { return data_[1]; }
		constexpr  float& x() { return data_[0]; }
		constexpr  float& y() { return data_[1]; }

		// 获取内部数据指针
		constexpr const float32_t* data() const { return data_; }
		constexpr float32_t* data() { return data_; }
		
		
		/*----------------------------------------------------------------------------------*/
		inline Vector2D operator+(const Vector2D& other) const
		{
		#ifdef VECTOR2D_USE_DSP
			float32_t result[2];
			arm_add_f32(data_, other.data_, result, 2);
			return Vector2D(result);     
		#else
			return Vector2D(data_[0] + other.data_[0], data_[1] + other.data_[1]);
		#endif
		}

		inline Vector2D operator-(const Vector2D& other) const
		{
		#ifdef VECTOR2D_USE_DSP
			float32_t result[2];
			arm_sub_f32(data_, other.data_, result, 2);
			return Vector2D(result);
		#else
			return Vector2D(data_[0] - other.data_[0], data_[1] - other.data_[1]);
		#endif
		}
		
		inline Vector2D operator-() const
		{
			return Vector2D(-data_[0], -data_[1]);
		}

		inline Vector2D operator*(float scalar) const
		{
		#ifdef VECTOR2D_USE_DSP
			float32_t result[2];
			arm_scale_f32(data_, scalar, result, 2);
			return Vector2D(result);
		#else
			return Vector2D(data_[0] * scalar, data_[1] * scalar);
		#endif
		}

		inline Vector2D operator/(float scalar) const
		{
			if (isZero(scalar)) return Vector2D();
			return *this * (1.f / scalar);
		}

		inline Vector2D& operator+=(const Vector2D& other)
		{
			arm_add_f32(data_, other.data_, data_, 2);
			return *this;
		}

		inline Vector2D& operator-=(const Vector2D& other)
		{
			arm_sub_f32(data_, other.data_, data_, 2);
			return *this;
		}

		inline Vector2D& operator*=(float scalar)
		{
			arm_scale_f32(data_, scalar, data_, 2);
			return *this;
		}

		inline Vector2D& operator/=(float scalar)
		{
			if (isZero(scalar)) return *this;
			return *this *= (1.0f / scalar);
		}
		
		// 相等比较运算符（带浮点精度容错）
		inline bool operator==(const Vector2D& other) const
		{
			return isZero(x() - other.x()) && isZero(y() - other.y());
		}

		// 不等比较运算符
		inline bool operator!=(const Vector2D& other) const
		{
			return !(*this == other);
		}
		
		/*----------------------------------------------------------------------------------*/
		// 点积运算
		inline float dot(const Vector2D& other) const
		{
			float32_t result;
			arm_dot_prod_f32(data_, other.data_, 2, &result);
			return result;
		}

		 // 叉积的标量结果（二维向量叉积的z分量）
		inline float cross(const Vector2D& other) const
		{
			return data_[0] * other.data_[1] - data_[1] * other.data_[0];
		}

		float length() const
		{
			float32_t result;
			arm_sqrt_f32(lengthSquared(), &result);
			return result;
		}

		// 向量角度pi ~ -pi
		float angle() const
		{
			if (isZero(lengthSquared())) return 0.0f;
			
			float result;
			arm_atan2_f32(data_[1], data_[0], &result);
			return result;
		}

		// 向量长度（模）
		float lengthSquared() const
		{
			return dot(*this);
		}

		// 归一化（单位向量）
		Vector2D normalize() const
		{
			float len = length();
			if (isZero(len)) return Vector2D();
			return *this * (1.f / len);
		}

		// 旋转向量（逆时针旋转theta弧度）
		Vector2D rotate(float theta) const
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
		Vector2D perpendicular() const
		{
			return Vector2D(-data_[1], data_[0]);
		}

		// 计算两点之间的距离
		static float distance(const Vector2D& a, const Vector2D& b)
		{
			return (b - a).length();
		}

		// 线性插值
		static Vector2D lerp(const Vector2D& a, const Vector2D& b, float t)
		{
			// 限制t在[0,1]范围内
			if (t < 0.0f) t = 0.0f;
			if (t > 1.0f) t = 1.0f;
			return a + (b - a) * t;
		}
		
		// 计算两点之间的距离平方
		static float distanceSquared(const Vector2D& a, const Vector2D& b)
		{
			return (b - a).lengthSquared();
		}
		
		
		// -pi~pi  θb - θa
		static float angleBetween(const Vector2D& a, const Vector2D& b)
		{
			float cross_val = a.cross(b);
			float dot_val = a.dot(b);

			if (isZero(dot_val) && isZero(cross_val)) return 0.f;

			float result;
			arm_atan2_f32(cross_val, dot_val, &result);
			return result;
		}
		

		static float curvatureFromThreePoints(const Vector2D& p0, const Vector2D& p1, const Vector2D& p2)
		{
			// 计算向量
			const Vector2D v0 = p1 - p0;  // 从p0到p1的向量
			const Vector2D v1 = p2 - p1;  // 从p1到p2的向量

			// 计算分子：2*(v0 × v1)（叉积的2倍，用于计算三角形面积的2倍）
			const float cross = v0.cross(v1);
			const float numerator = fabsf(2.f * cross);

			// 若叉积为0，三点共线（直线），曲率为0
			if (isZero(numerator)) return 0.f;

			// 计算分母：|v0| * |v1| * |v0 + v1|（三边长度乘积）
			const float len0 = v0.length();
			const float len1 = v1.length();
			const float len2 = (v0 + v1).length();  // p2 - p0的长度

			// 避免分母为0（三点中任意两点重合）
			if (isZero(len0) || isZero(len1) || isZero(len2)) return 0.f;  // 视为直线

			const float denominator = len0 * len1 * len2;

			// 曲率 = 分子 / 分母（曲率 = 1/半径）
			float curvature = numerator / denominator;
			
			// 限制曲率在合理范围内
			if (curvature > 1e6f) curvature = 1e6f;
			if (curvature < 0.f) curvature = 0.f;
			
			return curvature;
		}
		
		// 将当前向量投影到目标向量other上
		Vector2D project(const Vector2D& other) const
		{
			// 目标向量为零向量时，返回零向量
			float otherLenSq = other.lengthSquared();
			if (isZero(otherLenSq)) return Vector2D();
			
			// 计算投影系数：(a·b)/|b|²
			float projScalar = this->dot(other) / otherLenSq;
			
			// 计算投影向量：系数 × 目标向量
			return other * projScalar;
		}
		
		// 计算当前向量在目标向量other上的投影长度（标量）
		float projectLength(const Vector2D& other) const
		{
			float otherLenSq = other.lengthSquared();
			if (isZero(otherLenSq)) return 0.f;
			
			// 投影长度 = (a·b)/|b|
			float otherLen;
			arm_sqrt_f32(otherLenSq, &otherLen); // DSP 优化版开方
			return this->dot(other) / otherLen;
		}
		
		// 辅助函数：检查标量是否接近零
		constexpr static bool isZero(float scalar)
		{
			return (scalar < 0 ? -scalar : scalar) < 1e-6f;
		}
		
    protected:
		float32_t data_[2];  // 存储x和y分量，适配DSP库函数
    };
}
#endif
