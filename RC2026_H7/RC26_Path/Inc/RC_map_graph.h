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
			const float& Get_dotAB() const {return dotAB;}
			const float& Get_dotAC() const {return dotAC;}
		
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
		MapGraph();
		
		bool Get_Shortest_Path(uint8_t start, uint8_t end, uint8_t path[], uint8_t &pathLen, uint8_t *dist_);

		/*设置几号MF是否可以到达*/
		void Set_MF_Valid(uint8_t n, bool valid_);
	
		uint8_t Get_Node_On_Pos(vector2d::Vector2D p) const;
		
    private:
		
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
	
		/*节点是否可到达（只有MF部分可设置）*/
		bool valid[GRAPH_NODE_NUM] = 
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
		
		/*
			[0] red_right
			[1] blue_left
		*/
		const Area MC[2];		/*武馆*/
		const Area MF[2];		/*梅林*/
		const Area EXIT[2];       /*出口*/
		const Area ARENA[2];	/*对抗区*/
    };
}
#endif
