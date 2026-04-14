#include "RC_map_graph.h"

namespace path
{
	// 求完整路径 + 距离
	bool MapGraph::Get_Shortest_Path(uint8_t start, uint8_t end, uint8_t path[], uint8_t &pathLen, uint8_t *dist_)
	{
		pathLen = 0;
		
		// 边界检查
		if (start >= GRAPH_NODE_NUM || end >= GRAPH_NODE_NUM)
			return false;
		if (!valid[start] || !valid[end])
			return false;
		if (start == end)
		{
			path[0] = start;
			pathLen = 1;
			// 【修复】起点终点相同时，也输出距离
			if (dist_ != nullptr)
				*dist_ = 0;
			return true;
		}

		uint8_t local_dist[GRAPH_NODE_NUM]; // 局部距离数组
		int8_t prev[GRAPH_NODE_NUM];         // 前驱节点，用于回溯路径
		bool vis[GRAPH_NODE_NUM] = {false};

		// 初始化
		for (uint8_t i = 0; i < GRAPH_NODE_NUM; i++)
		{
			local_dist[i] = GRAPH_INF;
			prev[i] = -1;
		}
		local_dist[start] = 0;

		// Dijkstra核心
		for (uint8_t i = 0; i < GRAPH_NODE_NUM; i++)
		{
			uint8_t u = GRAPH_INVALID;
			uint8_t minDist = GRAPH_INF;
			for (uint8_t j = 0; j < GRAPH_NODE_NUM; j++)
			{
				if (!vis[j] && valid[j] && local_dist[j] < minDist)
				{
					minDist = local_dist[j];
					u = j;
				}
			}
			if (u == GRAPH_INVALID) break;
			vis[u] = true;

			for (uint8_t k = 0; k < 4; k++)
			{
				uint8_t v = graph[u][k].node;
				uint8_t w = graph[u][k].w;
				if (v == GRAPH_INVALID) break;
				if (!valid[v] || vis[v]) continue;

				if (local_dist[u] != GRAPH_INF && local_dist[v] > local_dist[u] + w)
				{
					local_dist[v] = local_dist[u] + w;
					prev[v] = u;
				}
			}
		}

		if (local_dist[end] == GRAPH_INF)
			return false;

		// 输出距离
		if (dist_ != nullptr)
			*dist_ = local_dist[end];

		// 回溯路径（从终点往回找）
		for (int8_t cur = end; cur != -1; cur = prev[cur])
			path[pathLen++] = cur;

		// 反转成 起点→终点 顺序
		for (uint8_t i = 0; i < pathLen / 2; i++)
		{
			uint8_t tmp = path[i];
			path[i] = path[pathLen - 1 - i];
			path[pathLen - 1 - i] = tmp;
		}

		return true;
	}


	void MapGraph::Set_MF_Valid(uint8_t n, bool valid_)
	{
		if (n == 0 || n >= 13) return;
		
		valid[n] = valid_;
	}

	uint8_t MapGraph::Get_Node_On_Pos(vector2d::Vector2D p)
	{
		if (!data::Is_Side_Init()) return GRAPH_INVALID;
		uint8_t dx = (uint8_t)data::Is_Blue_Left_Side();
		
		if (MC[dx].Is_Point_In(p)) return 0;
		
		if (EXIT[dx].Is_Point_In(p)) return 13;
		
		if (ARENA[dx].Is_Point_In(p)) return 14;
		
		if (MF[dx].Is_Point_In(p))
		{
			float MF_width = sqrtf(MF[dx].Get_dotAC());
			float MF_heigth = sqrtf(MF[dx].Get_dotAB());
			
			vector2d::Vector2D AP = p - MF[dx].Get_A();
			
			float MF_width_proj = AP.projectLength(MF[dx].Get_AC());
			float MF_heigth_proj = AP.projectLength(MF[dx].Get_AB());
			
			/*向上取整*/
			uint8_t w = ceilf(MF_width_proj / (MF_width / 3.f));
			uint8_t h = ceilf(MF_heigth_proj / (MF_heigth / 4.f));
			
			if (w < 1) w = 1;
			if (h < 1) h = 1;
			if (w > 3) w = 3;
			if (w > 4) h = 4;
			
			if (dx)
			{
				return w + (h - 1) * 3;
			}
			else
			{
				return (4 - w) + (h - 1) * 3;
			}
		}
		
		return GRAPH_INVALID;
	}


	vector2d::Vector2D MapGraph::Get_MF_Center(uint8_t n_)
	{
		if (n_ < 1 || n_ > 12) return vector2d::Vector2D();
		
		uint8_t dx = (uint8_t)data::Is_Blue_Left_Side();
		
		uint8_t w = (n_ - 1) % 3 + 1;
		uint8_t h = (n_ - 1) / 3 + 1;
	
		if (dx)
		{
			return MF[dx].Get_A() + MF[dx].Get_AB() * ((float)h - 0.5f) / 4.f + MF[dx].Get_AC() * ((float)w - 0.5f) / 3.f;
		}
		else
		{
			return MF[dx].Get_A() + MF[dx].Get_AB() * ((float)h - 0.5f) / 4.f + MF[dx].Get_AC() * ((float)(4 - w) - 0.5f) / 3.f;
		}
	}
}
