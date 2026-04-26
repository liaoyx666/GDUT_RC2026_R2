#pragma once
#include <stdint.h>

#include "RC_vector2d.h"
#include "RC_data_pool.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t GRAPH_NODE_NUM = 15; /*15个区域*/
	constexpr uint8_t GRAPH_INVALID = 16;
	constexpr uint8_t GRAPH_INF = 0xFF; /*不可达标记（最大距离远小于255）*/
	

	struct Graph
	{
		uint8_t node;
		uint8_t w;
	};
	
	/*方向*/
	enum Direction : int8_t
	{
		DIR_F =  1,
		DIR_B =  2,
		DIR_L =  3,
		DIR_R =  4,
		DIR_INVALID = 0,
	};
	
	
	// ==============================
	// 1. 取反 -dir （掉头）
	// ==============================
	inline Direction operator-(Direction dir)
	{
		switch (dir)
		{
			case DIR_F: return DIR_B;
			case DIR_B: return DIR_F;
			case DIR_L: return DIR_R;
			case DIR_R: return DIR_L;
			default:    return DIR_INVALID;
		}
	}

	// ==============================
	// 2. 前置 ++dir 左转循环
	// F→L→B→R→F
	// ==============================
	inline Direction& operator++(Direction& d)
	{
		switch (d)
		{
			case DIR_F: d = DIR_L; break;
			case DIR_L: d = DIR_B; break;
			case DIR_B: d = DIR_R; break;
			case DIR_R: d = DIR_F; break;
			default:    d = DIR_INVALID; break;
		}
		return d;
	}

	// ==============================
	// 3. 前置 --dir 右转循环
	// F→R→B→L→F
	// ==============================
	inline Direction& operator--(Direction& d)
	{
		switch (d)
		{
			case DIR_F: d = DIR_R; break;
			case DIR_R: d = DIR_B; break;
			case DIR_B: d = DIR_L; break;
			case DIR_L: d = DIR_F; break;
			default:    d = DIR_INVALID; break;
		}
		return d;
	}

	// ==============================
	// 4. 加法 dir + int（左转 N 次）
	// ==============================
	inline Direction operator+(Direction dir, int n)
	{
		n = n % 4;
		if (n < 0) n += 4;

		for (int i = 0; i < n; i++)
			++dir;

		return dir;
	}

	// ==============================
	// 5. 减法 dir - int（右转 N 次）
	// ==============================
	inline Direction operator-(Direction dir, int n)
	{
		return dir + (-n);
	}
	
	/*
		B-
		| |
		A-C
	
		A：公共顶点（原点）
		B：A的垂直邻点
		C：A的水平邻点
	*/
	class Area
	{
		public:
			Area(vector2d::Vector2D A_, vector2d::Vector2D B_, vector2d::Vector2D C_)
			: A(A_), AB(B_ - A_), AC(C_ - A_), dotAB(AB.dot(AB)), dotAC(AC.dot(AC)) {}
			~Area() = default;
			
			bool Is_Point_In(const vector2d::Vector2D& p) const
			{
				vector2d::Vector2D AP = p - A;
				
				float d1 = AP.dot(AB);
				float d2 = AP.dot(AC);

				return (d1 >= 0.0f && d1 <= dotAB) && 
					   (d2 >= 0.0f && d2 <= dotAC);
			}
			
			const vector2d::Vector2D& Get_A() const {return A;}
			const vector2d::Vector2D& Get_AB() const {return AB;}
			const vector2d::Vector2D& Get_AC() const {return AC;}
			float Get_dotAB() const {return dotAB;}
			float Get_dotAC() const {return dotAC;}
		
		private:
			const vector2d::Vector2D A;
			const vector2d::Vector2D AB;
			const vector2d::Vector2D AC;
			const float dotAB;
			const float dotAC;
	};

	
	
	
	
	class MapGraph
    {
    public:
		static bool Get_Shortest_Path(uint8_t start, uint8_t end, uint8_t path[], uint8_t &pathLen, uint8_t *dist_);

		/*设置几号MF是否可以到达*/
		static void Set_MF_Valid(uint8_t n, bool valid_);
		
		static uint8_t Get_Node_On_Pos(vector2d::Vector2D p);
		
		static vector2d::Vector2D Get_MF_Center(uint8_t n_);
	
		static bool Get_Path(vector2d::Vector2D s, vector2d::Vector2D e, uint8_t path[], uint8_t &pathLen);
	
		static Direction Dir_From_To(uint8_t f, uint8_t t);
	
		static float Yaw_On_Dir(Direction dir);

		static vector2d::Vector2D Offset_On_Dir(vector2d::Vector2D p, Direction dir, float offset);
		
		static constexpr float MF_SIZE = 1.2f;
		static constexpr float CHASSIS_SIZE = 0.75f;
	
		static constexpr int8_t height[GRAPH_NODE_NUM] = 
		{
			0,
			4, 2, 4,
			2, 4, 6,
			4, 6, 4,
			2, 4, 2,
			0,
			1
		};
		
		/*
			[0] red_right
			[1] blue_left
		*/
		static const inline Area MC[2] = 
		{
			Area(vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0)), 
			Area(vector2d::Vector2D(0, 0), vector2d::Vector2D(3.2, 0), vector2d::Vector2D(0, -6))
		}; /*武馆*/
		
		static const inline Area MF[2] = 
		{
			Area(vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0)),
			Area(vector2d::Vector2D(3.2, -1.2), vector2d::Vector2D(8, -1.2), vector2d::Vector2D(3.2, -4.8))
		}; /*梅林*/
		
		static const inline Area EXIT[2] = 
		{
			Area(vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0)),
			Area(vector2d::Vector2D(8, 0), vector2d::Vector2D(9.45, 0), vector2d::Vector2D(8, -6))
		}; /*出口*/
	
		static const inline Area ARENA[2] = 
		{
			Area(vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0), vector2d::Vector2D(0, 0)),
			Area(vector2d::Vector2D(9.45, 0), vector2d::Vector2D(10.65, 0), vector2d::Vector2D(9.45, -6))
		}; /*对抗区*/

		
		/*图（只读）*/
		static constexpr Graph graph[GRAPH_NODE_NUM][4] = 
		{
			{{1            , 4}, {2            , 2}, {3            , 4}, {GRAPH_INVALID, 0}}, // 0  武馆 MC
			/*-------------------------------------------------------------------------------------------*/
			{{0            , 4}, {2            , 2}, {4            , 2}, {GRAPH_INVALID, 0}}, // 1  MF1
			{{0            , 2}, {1            , 2}, {3            , 2}, {5            , 2}}, // 2  MF2
			{{0            , 4}, {2            , 2}, {6            , 2}, {GRAPH_INVALID, 0}}, // 3  MF3
			{{1            , 2}, {5            , 2}, {7            , 2}, {GRAPH_INVALID, 0}}, // 4  MF4
			{{2            , 2}, {4            , 2}, {6            , 2}, {8            , 2}}, // 5  MF5
			{{3            , 2}, {5            , 2}, {9            , 2}, {GRAPH_INVALID, 0}}, // 6  MF6
			{{4            , 2}, {8            , 2}, {10           , 2}, {GRAPH_INVALID, 0}}, // 7  MF7
			{{5            , 2}, {7            , 2}, {9            , 2}, {11           , 2}}, // 8  MF8
			{{6            , 2}, {8            , 2}, {12           , 2}, {GRAPH_INVALID, 0}}, // 9  MF9
			{{7            , 2}, {11           , 2}, {13           , 2}, {GRAPH_INVALID, 0}}, // 10 MF10
			{{8            , 2}, {10           , 2}, {12           , 2}, {13           , 4}}, // 11 MF11
			{{9            , 2}, {11           , 2}, {13           , 2}, {GRAPH_INVALID, 0}}, // 12 MF12
			/*-------------------------------------------------------------------------------------------*/
			{{10           , 2}, {11           , 4}, {12           , 2}, {14           , 1}}, // 13 出口 EXIT
			/*-------------------------------------------------------------------------------------------*/
			{{13           , 1}, {GRAPH_INVALID, 0}, {GRAPH_INVALID, 0}, {GRAPH_INVALID, 0}}  // 14 对抗区 ARENA
		};
		
		
		
   private:		
		
		/*节点是否可到达（只有MF部分可设置）*/
		static inline bool valid[GRAPH_NODE_NUM] = 
		{
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true,
			true
		};

	};
}
#endif
