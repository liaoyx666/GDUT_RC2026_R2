# 导入必要的库
import matplotlib

# 使用非交互后端以避免在系统缺少 tkinter / Tcl 时出现初始化错误
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches  # 用于绘制图形元素
import numpy as np  # 用于数值计算
import random  # 用于随机数生成
import heapq  # 用于优先队列（A*算法）
import math  # 用于数学运算
import os  # 用于文件操作
from itertools import combinations, permutations  # 用于生成组合和排列
import copy  # 用于深拷贝
import multiprocessing  # 用于多进程并行计算
from io import StringIO  # 用于捕获输出
import sys  # 用于系统操作
from tqdm import tqdm  # 用于显示进度条
from contextlib import contextmanager


# 轻量通用距离函数（避免频繁构造numpy数组）
def _dist(a, b):
    ax, ay = (a[0], a[1]) if not isinstance(a, np.ndarray) else (float(a[0]), float(a[1]))
    bx, by = (b[0], b[1]) if not isinstance(b, np.ndarray) else (float(b[0]), float(b[1]))
    return math.hypot(ax - bx, ay - by)


# --- 通用：静默输出上下文，用于大规模枚举时关闭打印 ---
@contextmanager
def suppress_stdout():
    """静默所有常见输出：覆盖 print，重定向 stdout/stderr（包括 tqdm）。"""
    import builtins
    old_print = builtins.print
    old_stdout, old_stderr = sys.stdout, sys.stderr
    try:
        builtins.print = lambda *args, **kwargs: None
        sys.stdout = StringIO()
        sys.stderr = StringIO()
        yield
    finally:
        builtins.print = old_print
        sys.stdout = old_stdout
        sys.stderr = old_stderr


# --- 1. 参数设置 ---
PARAMS = {
    # --- 动作耗时 (单位: 秒) ---
    's1': 1,  # 通用小型拾取/放置耗时（当前用于支撑/长矛点后的同步小动作）
    's3': 5,  # 武器交接动作耗时（R2 <-> R1 传递时的停留时间）
    's4': 5,  # R1 放置武器的耗时（预留/兼容）
    's5': 5,  # R2 拾取武器的耗时（预留/兼容）
    's6': 3,  # 秘籍拾取/放置的停留耗时（当前用于R2在邻居台阶或就地拾取秘籍时的停留时间）
    's7': 1.5,  # R2 在台阶上移动时，发生转向（x/y轴切换）的惩罚耗时
    's8': 2.5,  # 登上/走下 高度差~0.2m 台阶的耗时
    's9': 3.5,  # 登上/走下 高度差~0.4m 台阶的耗时

    # --- 细化参数 ---
    'r1_pickup_dwell': 0.2,  # R1 每次拾取自身区域秘籍时的额外原地停留时间（秒）
    # 机器人几何参数（用于验证阶段的占位/碰撞判定；不影响规划）
    'r1_radius': 0.3,  # R1 车体半径（米）
    'r2_radius': 0.2,  # R2 车体半径（米）
    # R1 通道与相邻地面规则
    'r1_left_channel_x': 1.0,  # 左侧通道 X
    'r1_right_channel_x': 5.0,  # 右侧通道 X
    'r1_middle_channel_x': 3.0,  # 中间通道 X（用于中列特殊拾取）
    'r1_middle_pick_rows': (1, 4),  # 允许在中通道邻接拾取的行（仅中列）
    # 相邻地面细化
    'row1_ground_y': 0.7,  # 行1下方相邻地面Y（低于行1台阶底边1.4，确保在地面）
    'row4_above_offset': 0.7,  # 行4上方相邻地面相对台阶中心的Δy（>0.6 才能在台阶上边之外）
    'r1_pickup_clearance_above': 0.25,  # 台阶上方拾取时，相比台阶上边缘(中心+0.6)的额外安全距离
    'r1_pickup_clearance_below': 0.25,  # 台阶下方拾取时，相比台阶下边缘(中心-0.6)的额外安全距离

    # --- 速度 (单位: 米/秒) ---
    'v1': 2.0,  # R1 正常移动速度
    'v2': 1.5,  # R2 正常移动速度
    'v1s': 1.5,  # R1 在特定区域（如斜坡）的加速后速度
    'v2s': 2.0,  # R2 在特定区域的加速后速度

    # --- 任务与物品数量 ---
    'weapon_count': 1,  # 场景中武器的总数
    'r1_manual_total': 3,  # R1 区域秘籍总数
    'r1_manual_needed': 2,  # R1 需要拾取的秘籍数量
    'r2_manual_total': 4,  # R2 区域秘籍总数
    'r2_manual_needed': 3,  # R2 需要拾取的秘籍数量

    # --- 坐标位置 (单位: 米) ---
    'r1_start_pos': (0.5, 8.8),  # R1 机器人起始坐标
    'r2_start_pos': (4.6, 8.9),  # R2 机器人起始坐标
    'support_pos': (2.525, 9.0),  # 支撑点坐标
    'spear_pos': (6, 8.35),  # 长矛坐标
    'ramp_pos': (0.75, 0),  # 斜坡坐标（终点）
    'debug_exit_bias': False,  # 调试：打印出口偏置计算
}

# 地图变体与主题（red | blue）
MAP_VARIANT = 'red'  # 默认 red；可通过 --map 覆盖


def get_theme(map_variant: str):
    """返回当前地图主题的颜色配置。"""
    mv = (map_variant or 'red').lower()
    if mv not in ('red', 'blue'):
        mv = 'red'
    if mv == 'red':
        return {
            'background_top': '#FF9999',
            'background_bottom': 'pink',
            'accent': 'red',  # 高亮（路径、R1 机器人颜色等）
            'accent_alt': 'red',  # 起止点标注
            'title_suffix': ' (map=red)'
        }
    else:  # blue
        return {
            'background_top': '#9999FF',
            'background_bottom': '#ADD8E6',
            'accent': 'blue',
            'accent_alt': 'blue',
            'title_suffix': ' (map=blue)'
        }


# === 并行枚举的进程池全局（Windows 需顶层定义） ===
_MP_STEPS_BASE = None
_MP_R2_CAPS = None


def _mp_enum_init(steps_base, r2_caps):
    """进程池初始化器：在子进程中设置只读全局数据。"""
    global _MP_STEPS_BASE, _MP_R2_CAPS
    _MP_STEPS_BASE = steps_base
    _MP_R2_CAPS = tuple(r2_caps)
    try:
        PARAMS['quiet'] = True
    except Exception:
        pass


def _mp_enum_worker(manual_indices):
    """子进程工作函数：对一个放置组合运行规划并返回输出行列表。"""
    try:
        steps_placement = _MP_STEPS_BASE  # 只读使用
        with suppress_stdout():
            manuals = create_manual_scenario(steps_placement, manual_indices)
            if not manuals:
                return []
            lines = []
            for cap in _MP_R2_CAPS:
                sim = Simulation(copy.deepcopy(steps_placement), manuals, cap, run_planner=True)
                best_plan = sim.plan
                code12, path_str = encode_placement_and_path(steps_placement, manuals, best_plan)
                line = f"{code12} {path_str}" if path_str else code12
                lines.append(line)
        return lines
    except Exception:
        return []


# --- 2. 实体与辅助类 ---
class Entity:
    """实体基类，定义场景中所有物体的基本属性和方法"""

    def __init__(self, pos, size, color, label=''):
        self.pos = np.array(pos, dtype=float)  # 当前位置
        self.original_pos = np.array(pos, dtype=float)  # 初始位置
        self.size = size  # 大小
        self.color = color  # 颜色
        self.label = label  # 标签
        self.patch = None  # 用于绘图的matplotlib对象
        self.carrier = None  # 携带者（机器人）
        self.is_picked_up = False  # 是否被拾取

    def __eq__(self, other):
        """判断两个实体是否相等"""
        return isinstance(other, Entity) and np.allclose(self.original_pos,
                                                         other.original_pos) and self.label == other.label

    def __hash__(self):
        """用于哈希计算，便于在集合中使用"""
        return hash((tuple(self.original_pos), self.label))

    def draw(self, ax):
        """绘制实体，由子类实现"""
        raise NotImplementedError

    def update_patch_pos(self):
        """更新绘图对象的位置，由子类实现"""
        raise NotImplementedError


class CircleEntity(Entity):
    """圆形实体类（如机器人、支撑点等）"""

    def __init__(self, pos, radius, color, label=''):
        super().__init__(pos, radius, color, label)  # 调用父类构造函数

    def draw(self, ax):
        """绘制圆形实体"""
        self.patch = patches.Circle(self.pos, self.size, facecolor=self.color, zorder=5)
        ax.add_patch(self.patch)

    def update_patch_pos(self):
        """更新圆形实体的位置"""
        if self.patch:
            self.patch.set_center(self.pos)


class RectangleEntity(Entity):
    """矩形实体类（如秘籍）"""

    def __init__(self, pos, width, height, color, label=''):
        super().__init__(pos, [width, height], color, label)  # 调用父类构造函数

    def draw(self, ax):
        """绘制矩形实体"""
        # 计算左下角坐标（从中心位置转换）
        bottom_left = (self.pos[0] - self.size[0] / 2, self.pos[1] - self.size[1] / 2)
        self.patch = patches.Rectangle(bottom_left, self.size[0], self.size[1], facecolor=self.color, zorder=5)
        ax.add_patch(self.patch)

    def update_patch_pos(self):
        """更新矩形实体的位置"""
        if self.patch:
            bottom_left = (self.pos[0] - self.size[0] / 2, self.pos[1] - self.size[1] / 2)
            self.patch.set_xy(bottom_left)


class Robot(CircleEntity):
    """机器人类，继承自圆形实体"""

    def __init__(self, robot_type, pos, radius, color):
        super().__init__(pos, radius, color, robot_type)  # 调用父类构造函数
        self.path = []  # 存储路径点
        self.path_timestamps = []  # 存储路径点对应的时间戳
        self.current_path_segment = 0  # 当前路径段索引
        self.carried_items_patches = []  # 携带物品的绘图对象
        self.carried_items_count = 0  # 携带物品数量

    def set_plan(self, path, timestamps):
        """设置机器人的路径计划"""
        self.path = [np.array(p) for p in path]
        self.path_timestamps = timestamps
        self.current_path_segment = 0  # 重置路径段索引

    def draw(self, ax):
        """绘制机器人"""
        self.patch = patches.Circle(self.pos, self.size, facecolor=self.color, zorder=10)
        ax.add_patch(self.patch)

    def update(self, time):
        """根据当前时间更新机器人位置"""
        # 如果没有路径或已到达最后一个路径点，直接返回
        if not self.path or self.current_path_segment >= len(self.path) - 1:
            if self.path:
                self.pos = self.path[-1]
            self.update_patch_pos()
            return

        # 找到当前时间所在的路径段
        while (self.current_path_segment < len(self.path) - 2 and
               time >= self.path_timestamps[self.current_path_segment + 1]):
            self.current_path_segment += 1

        # 获取当前路径段的起点和终点
        start_node, end_node = self.path[self.current_path_segment], self.path[self.current_path_segment + 1]
        start_time, end_time = self.path_timestamps[self.current_path_segment], self.path_timestamps[
            self.current_path_segment + 1]

        # 根据时间计算当前位置
        if time >= end_time:
            self.pos = end_node
        elif time > start_time:
            duration = end_time - start_time
            progress = (time - start_time) / duration if duration > 0 else 0
            self.pos = start_node + (end_node - start_node) * progress
        else:
            self.pos = start_node

        self.update_patch_pos()

    def add_carried_item(self, manual_patch):
        """添加携带的物品"""
        self.carried_items_patches.append(manual_patch)
        self.carried_items_count += 1


# --- 3. 时空规划器 ---
class Planner:
    """规划器类，负责为机器人规划最优路径和任务执行顺序"""

    def __init__(self, steps, manuals, r2_max_step_height):
        self.steps = steps  # 台阶信息
        self.manuals = manuals  # 秘籍信息
        self.r1_manuals = [m for m in manuals if m.label == 'R1']  # R1的秘籍
        self.r2_manuals = [m for m in manuals if m.label == 'R2']  # R2的秘籍
        self.fake_manual = next((m for m in manuals if m.label == '假'), None)  # 假秘籍
        self.best_plan = None  # 最优计划
        self.r2_max_step_height = r2_max_step_height  # R2最大可跨越高度
        self.quiet = bool(PARAMS.get('quiet', False))  # 静默模式：用于大规模枚举
        # 伪台阶支持：为每个入口行台阶(row=4)在其上方相邻地面处添加“伪台阶”，仅与该台阶相邻
        self.pseudo_indices = set()
        self.entry_to_pseudo = {}
        self.enable_legacy_entry_ground_pick = False  # 关闭旧的入口地面拾取特例
        self._add_pseudo_steps()

        # 构建逻辑索引→台阶对象的映射（核心！）
        self.logic_index_to_step = {s['logic_index']: s for s in self.steps}
        # 构建位置→逻辑索引的映射（用于反向查找）
        self.pos_to_logic_index = {tuple(np.round(s['center'], 2)): s['logic_index'] for s in self.steps}
        # 预计算邻接快速查询集合（有向对，便于直接判定相邻）
        self._adjacent_pairs = set()
        for s in self.steps:
            i = s['logic_index']
            for n in s['neighbors']:
                self._adjacent_pairs.add((i, n))
                # 由于 neighbors 已包含双向，我们不强制加入 (n,i)，但冗余加入也无害
                self._adjacent_pairs.add((n, i))
        self.r2_manual_indices = [m.step_index for m in self.r2_manuals]  # 所有R2秘籍所在台阶索引
        self._path_cache = {}  # 路径缓存（避免重复计算）

    def _add_pseudo_steps(self):
        """为入口行(row==4)台阶添加“伪台阶”节点并建立双向邻接。仅在规划内部使用。"""
        if not self.steps:
            return
        max_idx = max(s['logic_index'] for s in self.steps)
        next_idx = max_idx + 1
        logic_to_step = {s['logic_index']: s for s in self.steps}
        new_steps = []
        for s in list(self.steps):
            # 仅对“基础入口行台阶”添加伪台阶，避免重复添加
            if s.get('row') == 4 and not s.get('is_pseudo', False):
                base_idx = s['logic_index']
                if base_idx in self.entry_to_pseudo:
                    # 已为该基础台阶添加过伪台阶，跳过
                    continue
                cx, cy = s['center']
                pseudo_center = (cx, cy + 0.6)
                pseudo_idx = next_idx
                next_idx += 1
                pseudo_step = {
                    'logic_index': pseudo_idx,
                    'list_index': len(self.steps) + len(new_steps),
                    'center': pseudo_center,
                    'row': 4,
                    'col': s['col'],
                    'height': s['height'],
                    'manual': None,
                    'neighbors': [base_idx],
                    'is_pseudo': True,
                }
                # 基台阶加入与伪台阶邻接（双向）
                logic_to_step[base_idx]['neighbors'] = list(set(logic_to_step[base_idx]['neighbors'] + [pseudo_idx]))
                new_steps.append(pseudo_step)
                self.pseudo_indices.add(pseudo_idx)
                self.entry_to_pseudo[base_idx] = pseudo_idx
        if new_steps:
            self.steps.extend(new_steps)
            if not self.quiet:
                try:
                    created = len(new_steps)
                    mapping_preview = list(self.entry_to_pseudo.items())[:3]
                    print(f"[伪台阶] 已为入口行创建 {created} 个伪台阶，示例映射(entry->pseudo): {mapping_preview}")
                    print("[伪台阶] 已禁用旧的入口行地面拾取特例: enable_legacy_entry_ground_pick=False")
                except Exception:
                    pass

    def _build_obstacles(self, r1_non_primary_indices, extra_indices=None, include_fake=True):
        """集中构造障碍集合：R1非主任务 + （可选）假秘籍 + （可选）额外集合。"""
        s = set(r1_non_primary_indices) if r1_non_primary_indices else set()
        if include_fake and self.fake_manual:
            s.add(self.fake_manual.step_index)
        if extra_indices:
            s.update(extra_indices)
        return s

    def are_steps_adjacent(self, idx1, idx2):
        """判断两个台阶是否相邻（行相等且列差1，或列相等且行差1）"""
        # 使用预计算集合快速判断；若无则回退到几何判断
        if hasattr(self, '_adjacent_pairs') and (idx1, idx2) in self._adjacent_pairs:
            return True
        if idx1 not in self.logic_index_to_step or idx2 not in self.logic_index_to_step:
            return False
        s1, s2 = self.logic_index_to_step[idx1], self.logic_index_to_step[idx2]
        return (s1['row'] == s2['row'] and abs(s1['col'] - s2['col']) == 1) or \
            (s1['col'] == s2['col'] and abs(s1['row'] - s2['row']) == 1)

    # 位置→台阶索引的快速映射，保留原有1e-2容差语义
    def _pos_to_idx(self, pos, atol=1e-2):
        try:
            key = tuple(np.round(pos, 2))
            idx = self.pos_to_logic_index.get(key)
            if idx is not None:
                return idx
        except Exception:
            pass
        for idx2, step in self.logic_index_to_step.items():
            if np.allclose(step['center'], pos, atol=atol):
                return idx2
        return -1

    def _map_positions_to_step_indices(self, positions, atol=1e-2):
        seq = []
        for p in positions:
            idx = self._pos_to_idx(p, atol=atol)
            if idx != -1:
                seq.append(idx)
        return seq

    def plan(self):
        """主规划函数，生成最优任务计划"""
        print(f"\n--- 开始规划 (R2最大可跨越高度: {self.r2_max_step_height}m) ---")
        # 检查R2秘籍数量是否足够
        if not self.r2_manuals or len(self.r2_manuals) < PARAMS['r2_manual_needed']:
            print("错误：为R2生成的秘籍数量不足。")
            return None

        # 规划初始阶段路径（机器人到达指定位置）
        initial_r1_path, initial_r1_ts, initial_r2_path, initial_r2_ts, phase1_end_time, r1_forest_entry, r2_start_pos_real = self._plan_initial_phase()
        self.phase1_end_time = phase1_end_time
        self.r2_start_pos_real = r2_start_pos_real

        # 修正R2拼接武器后到入口台阶的路径为正常移动
        # 如果R2初始路径最后一个点不是入口台阶，则补一段移动
        def get_entry_step_center(entry_step):
            return self.logic_index_to_step[entry_step]['center'] if entry_step in self.logic_index_to_step else None

        # 生成R1需要拾取的秘籍组合（从3个中选2个）
        r1_manual_combos = list(combinations(self.r1_manuals, PARAMS['r1_manual_needed']))
        if not self.quiet:
            print(f"评估 {len(r1_manual_combos)} 种R1主任务组合...")

        # === 全局最优方案 ===
        global_best_plan = None
        global_best_time = float('inf')
        # 遍历所有R1秘籍组合

        progress_iter = tqdm(r1_manual_combos, desc="Evaluating R1 Combinations", disable=self.quiet)
        for i, r1_primary_choice in enumerate(progress_iter):
            if not self.quiet:
                print(
                    f"\n  - 方案 {i + 1}/{len(r1_manual_combos)}: R1主任务 {[f'台阶{self.logic_index_to_step[m.step_index]['logic_index']}' for m in r1_primary_choice]}")

            r1_non_primary_manuals = [m for m in self.r1_manuals if m not in r1_primary_choice]
            r1_non_primary_indices = {m.step_index for m in r1_non_primary_manuals}

            all_r2_indices = {m.step_index for m in self.r2_manuals}
            r2_combos = list(combinations(all_r2_indices, PARAMS['r2_manual_needed']))

            if not self.quiet:
                print(f"    - 将为R1主任务评估 {len(r2_combos)} 种R2秘籍组合...")

            # 遍历所有可选入口行台阶
            # 新增限制：入口行台阶必须满足“与出口同款”的能力约束——其高度不超过 R2 的越障能力
            # 入口规则：
            # - 当 r2_max_step_height == 0.2 时，仅允许中列（col=2）
            # - 当 r2_max_step_height >= 0.4 时，入口行为行4的任意列
            allow_any_col = self.r2_max_step_height >= 0.4 - 1e-9
            valid_entries = [
                idx for idx in self.logic_index_to_step
                if not self.logic_index_to_step[idx].get('is_pseudo', False)
                   and self.logic_index_to_step[idx]['row'] == 4
                   and (allow_any_col or self.logic_index_to_step[idx]['col'] == 2)
                   and self.logic_index_to_step[idx]['height'] <= self.r2_max_step_height
                   and idx not in r1_non_primary_indices
                   and ((idx not in all_r2_indices) or (idx in self.entry_to_pseudo))
                   and (not self.fake_manual or idx != self.fake_manual.step_index)
            ]
            if not valid_entries:
                print("    - 无法找到任何本身无秘籍的入口行台阶，跳过此方案。")
                continue

            for entry_step in valid_entries:
                print(f"    - R2进入台阶区域的入口台阶: {entry_step}")
                if not allow_any_col:
                    try:
                        if self.logic_index_to_step.get(entry_step, {}).get('col') != 2:
                            print(f"[DEBUG] 警告：入口台阶{entry_step}并非中列（col=2），该提示仅用于诊断")
                    except Exception:
                        pass

                # 复制初始路径并根据入口台阶进行衔接修正
                this_r2_path = initial_r2_path.copy()
                this_r2_ts = initial_r2_ts.copy()
                this_r2_path, this_r2_ts = self._adjust_entry_path(this_r2_path, this_r2_ts, entry_step,
                                                                   phase1_end_time)

                for r2_manuals_to_get in r2_combos:
                    # 评估该R2组合的所有顺序，得到拾取阶段的best_*
                    initial_r2_path_local = this_r2_path.copy()
                    initial_r2_ts_local = this_r2_ts.copy()
                    (best_path, best_pickup_schedule, best_order, best_total_time,
                     feasible_orders, feasible_orders_full) = self._evaluate_orders_for_combo(
                        initial_r2_path_local, initial_r2_ts_local,
                        r1_non_primary_indices, all_r2_indices, list(r2_manuals_to_get)
                    )

                    # 若无有效best结果，则保持原有提示并跳过出口判定
                    if best_path is None or best_pickup_schedule is None or best_order is None:
                        print(
                            f"    [出口判定跳过] 无有效best结果：best_path={best_path is not None}, best_pickup_schedule={best_pickup_schedule is not None}, best_order={best_order is not None}",
                            flush=True
                        )
                        print("    - 所有R2拾取顺序均无效，跳过此方案。")
                        continue

                    # 出口判定前的总览（保持原日志）
                    print(
                        f"    [出口前总览] best_order={best_order}, 拾取记录数={0 if not best_pickup_schedule else len(best_pickup_schedule)}，best_time(仅拾取)={float('inf') if best_total_time == float('inf') else round(best_total_time, 2)}",
                        flush=True
                    )
                    if not best_pickup_schedule or len(best_pickup_schedule) < PARAMS['r2_manual_needed']:
                        print(
                            f"    [出口判定跳过] 已完成拾取数不足：have={0 if not best_pickup_schedule else len(best_pickup_schedule)}, need={PARAMS['r2_manual_needed']}",
                            flush=True
                        )
                        continue

                    # 构建出口路径
                    exit_path, exit_time, exit_valid = self._build_exit_path(best_path, r1_non_primary_indices)
                    if not exit_valid:
                        continue

                    # 整合R2路径与时间
                    full_r2_path_tuples, r2_final_time = self._integrate_r2_exit(best_path, exit_path, exit_time)
                    if r2_final_time is None:
                        print(f"[DEBUG] 跳过无效方案: r2_final_time={r2_final_time}, error=NoneType detected")
                        continue

                    # 规划R1并尝试更新全局最优
                    global_best_time, global_best_plan = self._finalize_and_update_global(
                        r1_primary_choice, r1_forest_entry, phase1_end_time,
                        r2_final_time, global_best_time,
                        initial_r1_path, initial_r1_ts,
                        initial_r2_path_local, initial_r2_ts_local,
                        full_r2_path_tuples, best_pickup_schedule
                    )

        # 输出全局最优方案详情
        self.best_plan = global_best_plan
        if self.best_plan:
            print(f"\n最优方案已确定！最短总耗时: {self.best_plan['total_time']:.2f}s")

            # 打印拼接武器完成信息（若可用）
            wa_t = self.best_plan.get('weapon_assembly_time')
            wa_pos = self.best_plan.get('weapon_assembly_pos')
            if wa_t is not None and wa_pos is not None:
                print(f"\n--- 武器拼接完成 ---\n时间 {wa_t:.2f}s @ 坐标 {tuple(np.round(wa_pos, 2))}")

            print("\n--- R1 最优拾取路径详情 ---")
            for manual, pickup_time in sorted(self.best_plan['r1_pickup_schedule'].items(), key=lambda item: item[1]):
                manual_pos = manual.original_pos
                manual_step = next((s for s in self.steps if np.allclose(s['center'], manual_pos)), None)

                # 查找拾取时的位置
                r1_pos_at_pickup = None
                for i in range(len(self.best_plan['r1_timestamps'])):
                    if np.isclose(self.best_plan['r1_timestamps'][i], pickup_time):
                        r1_pos_at_pickup = self.best_plan['r1_path'][i]
                        break

                r1_loc_str = ""
                if r1_pos_at_pickup is not None:
                    # 反算网格坐标: x = 0.6 + c * 1.2, y = 0.8 + r * 1.2
                    c = int(round((r1_pos_at_pickup[0] - 0.6) / 1.2))
                    r = int(round((r1_pos_at_pickup[1] - 0.8) / 1.2))
                    r1_loc_str = f"在 [网格 (row:{r}, col:{c})] "

                manual_loc_str = f"台阶 {manual_step['logic_index']} (row:{manual_step['row']}, col:{manual_step['col']})"
                print(f"时间 {pickup_time:.2f}s: R1 {r1_loc_str}拾取位于 [{manual_loc_str}] 的秘籍")

            print("\n--- R2 最优拾取路径详情 ---")
            for manual, pickup_time in sorted(self.best_plan['r2_pickup_schedule'].items(), key=lambda item: item[1]):
                manual_pos = manual.original_pos
                manual_step = next((s for s in self.steps if np.allclose(s['center'], manual_pos)), None)

                # 查找拾取时的位置
                r2_pos_at_pickup = None
                for i in range(1, len(self.best_plan['r2_timestamps'])):
                    if np.isclose(self.best_plan['r2_timestamps'][i - 1], pickup_time) and \
                            np.isclose(self.best_plan['r2_timestamps'][i] - self.best_plan['r2_timestamps'][i - 1],
                                       PARAMS['s6']):
                        r2_pos_at_pickup = self.best_plan['r2_path'][i - 1]
                        break

                if r2_pos_at_pickup is None:
                    for i in range(len(self.best_plan['r2_timestamps'])):
                        if self.best_plan['r2_timestamps'][i] >= pickup_time:
                            r2_pos_at_pickup = self.best_plan['r2_path'][i - 1] if i > 0 else self.best_plan['r2_path'][
                                0]
                            break

                r2_step = next((s for s in self.steps if np.allclose(s['center'], r2_pos_at_pickup)), None)

                r2_loc_str = f"台阶 {r2_step['logic_index']} (row:{r2_step['row']}, col:{r2_step['col']})" if r2_step else f"地面 {np.round(r2_pos_at_pickup, 2)}"
                manual_loc_str = f"台阶 {manual_step['logic_index']} (row:{manual_step['row']}, col:{manual_step['col']})"

                print(f"时间 {pickup_time:.2f}s: R2 在 [{r2_loc_str}] 拾取位于 [{manual_loc_str}] 的秘籍")

        return self.best_plan

    # === Batch B: 私有方法拆分（保持日志与行为不变） ===
    def _adjust_entry_path(self, r2_path, r2_ts, entry_step, phase1_end_time):
        """根据入口台阶，修正R2初始路径的衔接（保持原有日志位置在plan中）。"""
        # 若入口台阶上为R2秘籍，则改为进入其伪台阶（如存在）
        entry_center = None
        if entry_step in self.logic_index_to_step:
            step_obj = self.logic_index_to_step[entry_step]
            use_pseudo = False
            if step_obj.get('manual') is not None and getattr(step_obj['manual'], 'label', None) == 'R2':
                if entry_step in self.entry_to_pseudo:
                    use_pseudo = True
            if use_pseudo:
                pseudo_idx = self.entry_to_pseudo[entry_step]
                entry_center = self.logic_index_to_step[pseudo_idx]['center']
            else:
                entry_center = step_obj['center']
        if r2_path and entry_center is not None:
            last_r2_pos = r2_path[-1]
            if not np.allclose(last_r2_pos, entry_center, atol=1e-2):
                move_time = _dist(last_r2_pos, entry_center) / PARAMS['v2']
                # 新增：进入入口台阶的爬升惩罚，等同于其他处的处理
                climb_penalty = 3.0
                # 若上一步在某台阶中心，则计算高度差；否则（在地面/普通通道），不计高度差惩罚
                prev_idx = self.find_step_by_pos(last_r2_pos)
                # 进入位置的高度：若使用伪台阶，则等于入口台阶高度
                target_idx = self.entry_to_pseudo.get(entry_step, entry_step)
                if prev_idx != -1 and target_idx in self.logic_index_to_step:
                    dz = abs(
                        self.logic_index_to_step[target_idx]['height'] - self.logic_index_to_step[prev_idx]['height'])
                    if dz > 0.01:
                        if np.isclose(dz, 0.2, atol=0.05):
                            climb_penalty = PARAMS.get('s8', 0)
                        elif np.isclose(dz, 0.4, atol=0.05):
                            climb_penalty = PARAMS.get('s9', 0)
                # 严格顺序约束：R1 先进入下层区域（到达 forest_entry），R2 才能进入入口台阶
                start_base = r2_ts[-1] if r2_ts else phase1_end_time
                epsilon = 1e-6
                t0 = max(start_base, phase1_end_time + epsilon)
                r2_path.append(entry_center)
                r2_ts.append(t0 + move_time + climb_penalty)
            else:
                # 已在入口台阶中心，如时间早于 R1 进入下层的时刻，则追加一个零位移等待节点以满足先后关系
                if r2_ts and r2_ts[-1] < phase1_end_time:
                    epsilon = 1e-6
                    r2_path.append(entry_center)
                    r2_ts.append(phase1_end_time + epsilon)
        return r2_path, r2_ts

    def _evaluate_orders_for_combo(self, initial_r2_path, initial_r2_ts,
                                   r1_non_primary_indices, all_r2_indices, r2_manuals_to_get):
        """评估一个R2组合的所有拾取顺序，返回拾取阶段的best_*与可行顺序集合。保持所有原始日志。"""
        best_path = None
        best_pickup_schedule = None
        best_order = None
        best_total_time = float('inf')
        best_pickup_debug_info = None
        best_output_pickup_info = None
        r2_manuals_to_get = list(r2_manuals_to_get)
        print(f"\n      -- 正在评估R2组合: {[f'台阶{idx}' for idx in r2_manuals_to_get]}")
        # 诊断：打印伪台阶与legacy状态，便于确认逻辑是否生效
        try:
            print(
                f"      [伪台阶] 状态检查: pseudo_count={len(getattr(self, 'pseudo_indices', set()))}, legacy_ground_pick={getattr(self, 'enable_legacy_entry_ground_pick', None)}",
                flush=True)
        except Exception:
            pass

        # 仅用于对齐原变量存在性
        r1_final_time = None
        r2_final_time = None

        all_obstacles = r1_non_primary_indices.copy()
        all_obstacles.update(all_r2_indices)
        if self.fake_manual:
            all_obstacles.add(self.fake_manual.step_index)

        feasible_orders = []
        feasible_orders_full = []

        orders = list(permutations(r2_manuals_to_get))
        
        # 新增约束：若待拾取列表中包含第一行（row=4）的台阶（即1,2,3号），
        # 则必须首先拾取其中之一（强制起始点为第一行台阶）
        row4_indices = {idx for idx in r2_manuals_to_get if self.logic_index_to_step[idx]['row'] == 4}
        if row4_indices:
            orders = [o for o in orders if o[0] in row4_indices]
            print(f"      [约束] 检测到第一行台阶{row4_indices}，已过滤顺序，仅保留以第一行台阶开头的路径", flush=True)

        print(f"      >> 本R2组合共有 {len(orders)} 种顺序将被评估", flush=True)
        for order_idx, order in enumerate(orders, start=1):
            print(f"        >> 开始评估顺序 {order_idx}/{len(orders)}: {[f'台阶{idx}' for idx in order]}", flush=True)
            current_pos = initial_r2_path[-1]
            current_idx = self.find_step_by_pos(current_pos)
            current_time = initial_r2_ts[-1]
            print(f"            [起点] 台阶{current_idx} at {np.round(current_pos, 2)}, t={current_time:.2f}s",
                  flush=True)

            picked_indices = []
            pickup_schedule = {}
            path = []
            valid = True
            order_failed = False
            illegal_found = False
            order_failed_reason = None
            order_failed_context = {}
            # 仅允许一次通过“伪台阶”进行拾取
            used_pseudo_pick = False

            for step_no, manual_idx in enumerate(order, start=1):
                # 每一步都根据当前位置刷新台阶索引，以支持“地面位置”的场景
                current_idx = self.find_step_by_pos(current_pos)
                all_unpicked_r2_global_except_current = set(self.r2_manual_indices) - set(picked_indices) - {manual_idx}
                dynamic_obstacles = set(r1_non_primary_indices)
                if self.fake_manual:
                    dynamic_obstacles.add(self.fake_manual.step_index)
                dynamic_obstacles.update(all_unpicked_r2_global_except_current)
                goal_step = self.logic_index_to_step[manual_idx]
                legal_neighbors = [n for n in goal_step['neighbors'] if n not in dynamic_obstacles]
                # 新增：拾取优先级 —— 若存在多个合法邻居，优先选择“行数更低（row 更小）”的台阶
                legal_neighbor_rows = {n: self.logic_index_to_step[n]['row'] for n in legal_neighbors}
                min_legal_row = min(legal_neighbor_rows.values()) if legal_neighbors else None
                fallback_same_row_candidate = None  # (idx, pth2, t3)

                # 打印时标注伪台阶
                def fmt_idx(i):
                    return f"{i}{'(伪)' if i in getattr(self, 'pseudo_indices', set()) else ''}"

                ln_str = [fmt_idx(i) for i in sorted(legal_neighbors)]
                ban_str = [fmt_idx(i) for i in sorted(dynamic_obstacles)]
                print(
                    f"            [第{step_no}步] 目标台阶{fmt_idx(manual_idx)}，合法邻居={ln_str}，禁行台阶={ban_str}，当前台阶={fmt_idx(current_idx)}",
                    flush=True
                )

                # 更高优先级：若“当前位置台阶”本身就是目标秘籍的合法邻居，则直接就地拾取（无需移动）
                if current_idx in legal_neighbors:
                    manual_obj_here = self.logic_index_to_step[manual_idx].get('manual')
                    if manual_obj_here is None or getattr(manual_obj_here, 'label', None) != 'R2':
                        valid = False
                        order_failed = True
                        order_failed_reason = "目标台阶无R2秘籍"
                        order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                        break
                    # 执行就地拾取：耗时为 s6，无位移
                    t3_here = current_time + PARAMS.get('s6', 3)
                    pickup_schedule[manual_obj_here] = t3_here
                    picked_indices.append(manual_idx)
                    # 记录路径时间点（与其他分支保持一致，写入完成时刻即可）
                    path.append((current_pos, t3_here))
                    current_time = t3_here
                    # 伪台阶使用次数限制
                    if current_idx in getattr(self, 'pseudo_indices', set()):
                        if used_pseudo_pick:
                            valid = False
                            order_failed = True
                            order_failed_reason = "伪台阶拾取已使用一次"
                            order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                            break
                        used_pseudo_pick = True
                    print(
                        f"                [到达]（就地）拾取点台阶{fmt_idx(current_idx)}，完成拾取目标{fmt_idx(manual_idx)} (t={current_time:.2f}s)",
                        flush=True
                    )
                    # 拾取后不允许停留在仍含未拾取R2秘籍的台阶上（理论上 legal_neighbors 已规避，但保持一致性校验）
                    all_unpicked_r2_post = set(self.r2_manual_indices) - set(picked_indices)
                    if current_idx in all_unpicked_r2_post:
                        valid = False
                        order_failed = True
                        order_failed_reason = "拾取后停留台阶仍含未拾取R2秘籍"
                        order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                        break
                    # 该目标已完成，进入下一目标
                    continue

                # 使用 A* 导航到目标的合法邻居拾取点，新的优先级：
                # 1) 就地（上方已处理）
                # 2) 可达邻居优先：逐个邻居判定可达性与耗时，择最优；若平手，低行数优先
                # 3) 若无可达邻居，则此顺序失败
                remaining_r2 = set(self.r2_manual_indices) - set(picked_indices) - {manual_idx}
                manuals_to_ignore = set(r1_non_primary_indices)
                if self.fake_manual:
                    manuals_to_ignore.add(self.fake_manual.step_index)

                reachable_candidates = []  # (neighbor_idx, path, total_time, final_pos)
                for n_idx in legal_neighbors:
                    # 单邻居约束：只允许在该邻居作为拾取点
                    pth, tt, _meta, fpos = self._find_path_to_manual(
                        current_pos, current_time, manual_idx,
                        r1_primary_manuals=[],
                        manuals_to_ignore=manuals_to_ignore,
                        remaining_indices=remaining_r2,
                        preferred_row=None,
                        allowed_pickup_indices=[n_idx]
                    )
                    if pth is not None:
                        reachable_candidates.append((n_idx, pth, tt, fpos))

                if reachable_candidates:
                    # 先按时间最短排序，若时间相同则按行数更小排序
                    eps = 1e-6
                    # 找到最小耗时
                    min_t = min(tt for (_n, _p, tt, _f) in reachable_candidates)
                    bests = [(n, p, tt, f) for (n, p, tt, f) in reachable_candidates if abs(tt - min_t) <= eps]
                    if len(bests) > 1:
                        # 按行数更小（row 更小优先）
                        bests.sort(key=lambda item: self.logic_index_to_step[item[0]]['row'])
                    chosen = bests[0]
                    chosen_neighbor, full_path, total_time, final_pos = chosen
                else:
                    full_path, total_time, final_pos = None, float('inf'), current_pos

                if full_path is None:
                    valid = False
                    order_failed = True
                    order_failed_reason = "无法到达目标邻居台阶"
                    order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                    break

                # 记录与状态更新
                manual_obj = self.logic_index_to_step[manual_idx].get('manual')
                if manual_obj is None or getattr(manual_obj, 'label', None) != 'R2':
                    valid = False
                    order_failed = True
                    order_failed_reason = "目标台阶无R2秘籍"
                    order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                    break
                pickup_finish_time = full_path[-1][1] if full_path else current_time
                pickup_schedule[manual_obj] = pickup_finish_time
                picked_indices.append(manual_idx)
                path.extend(full_path)
                current_pos = final_pos
                current_idx = self.find_step_by_pos(final_pos)
                current_time = pickup_finish_time
                if current_idx in getattr(self, 'pseudo_indices', set()):
                    if used_pseudo_pick:
                        valid = False
                        order_failed = True
                        order_failed_reason = "伪台阶拾取已使用一次"
                        order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                        break
                    used_pseudo_pick = True
                print(
                    f"                [到达]（A*一致）拾取点台阶{fmt_idx(current_idx)}，完成拾取目标{fmt_idx(manual_idx)} (t={current_time:.2f}s)",
                    flush=True
                )
                all_unpicked_r2 = set(self.r2_manual_indices) - set(picked_indices)
                if current_idx in all_unpicked_r2:
                    valid = False
                    order_failed = True
                    order_failed_reason = "拾取后停留台阶仍含未拾取R2秘籍"
                    order_failed_context = {'manual_idx': manual_idx, 'current_idx': current_idx}
                    break

            path_step_seq = self._map_positions_to_step_indices([pos for pos, _t in path], atol=1e-2)
            if path_step_seq:
                print(f"            [路径] 经过台阶序列: {path_step_seq}", flush=True)
            else:
                print(f"            [路径] 尚无台阶路径（可能在起点即失败）", flush=True)

            pickup_debug_info_all = []
            for idx in order:
                manual = self.logic_index_to_step.get(idx, {}).get('manual')
                if manual is None or manual not in pickup_schedule:
                    pickup_debug_info_all.append(f"            [拾取] 台阶{idx}: 未完成")
                    continue
                pickup_time = pickup_schedule[manual]
                manual_step = self.logic_index_to_step.get(manual.step_index)
                r2_pick_pos = None
                last_pos = None
                for pos, t in path:
                    if t <= pickup_time + 1e-6:
                        last_pos = pos
                    else:
                        break
                if last_pos is not None:
                    idx2 = self._pos_to_idx(last_pos, atol=1e-2)
                    if idx2 != -1:
                        r2_pick_pos = self.logic_index_to_step[idx2]
                if manual_step and r2_pick_pos:
                    adj = self.are_steps_adjacent(r2_pick_pos['logic_index'], manual_step['logic_index'])
                    adj_str = '相邻' if adj else '非相邻'
                    pickup_debug_info_all.append(
                        f"            [拾取] 在台阶{r2_pick_pos['logic_index']} 拾取{adj_str} 台阶{manual_step['logic_index']} 的秘籍 (t={pickup_time:.2f}s)"
                    )
                else:
                    pickup_debug_info_all.append(f"            [拾取] 台阶{idx}: 定位失败")
            if pickup_debug_info_all:
                for line in pickup_debug_info_all:
                    print(line, flush=True)

            print(f"            [校验] valid={valid}, order_failed={order_failed}, 已拾取数={len(pickup_schedule)}",
                  flush=True)

            if order_failed or (not valid and len(pickup_schedule) < PARAMS['r2_manual_needed']):
                reason_text = order_failed_reason or "约束不满足"
                print(
                    f"        顺序 {[f'台阶{idx}' for idx in order]}: 不成立，最终拾取秘籍数: {len(pickup_schedule)}；原因: {reason_text}",
                    flush=True)
                print(
                    f"        >> 结束评估顺序 {order_idx}/{len(orders)}: 拾取数={len(pickup_schedule)}，不可行，原因: {reason_text}",
                    flush=True)
                continue

            if (len(pickup_schedule) == PARAMS['r2_manual_needed']) and (not order_failed):
                print("        >>> 进入可行分支：满足拾取数量与合法性校验", flush=True)
                illegal_found = False
                pickup_debug_info = []
                actual_pick_indices = []
                for idx in order:
                    manual = self.logic_index_to_step.get(idx, {}).get('manual')
                    if manual is None or manual not in pickup_schedule:
                        pickup_debug_info.append(f"            [调试] 未找到台阶{idx}的拾取记录")
                        continue
                    pickup_time = pickup_schedule[manual]
                    manual_step = self.logic_index_to_step.get(manual.step_index)
                    r2_pick_pos = None
                    last_pos = None
                    for pos, t in path:
                        if t <= pickup_time + 1e-6:
                            last_pos = pos
                        else:
                            break
                    if last_pos is not None:
                        for idx2, step in self.logic_index_to_step.items():
                            if np.allclose(step['center'], last_pos, atol=1e-2):
                                r2_pick_pos = step
                                break
                    if manual_step and r2_pick_pos:
                        actual_pick_indices.append(manual_step['logic_index'])
                        if not self.are_steps_adjacent(r2_pick_pos['logic_index'], manual_step['logic_index']):
                            illegal_found = True
                            break
                        pickup_debug_info.append(
                            f"            [调试] R2在{r2_pick_pos['row']}行{r2_pick_pos['col']}列台阶拾取相邻{manual_step['row']}行{manual_step['col']}列台阶的秘籍")
                    else:
                        pickup_debug_info.append(f"            [调试] R2拾取动作定位失败")
                output_pickup_info = not illegal_found
                if illegal_found:
                    print(f"        [校验] 存在非相邻拾取，标记为不可行", flush=True)
                    order_failed = True
                else:
                    print("        >>> 终审通过：记录为可行顺序，准备加入候选与更新best_*", flush=True)
                    feasible_orders.append((list(order), current_time))
                    feasible_orders_full.append((list(order), current_time, path.copy(), pickup_schedule.copy()))
                    print(f"        顺序 {[f'台阶{idx}' for idx in order]}: 可行，耗时: {current_time:.2f}s", flush=True)
                    print(f"        >> 结束评估顺序 {order_idx}/{len(orders)}: 拾取数={len(pickup_schedule)}，可行",
                          flush=True)
                    print("            [路径明细]")
                    last_idx = self.find_step_by_pos(initial_r2_path[-1])
                    last_time = initial_r2_ts[-1]
                    seg_start = initial_r2_path[-1]
                    seg_start_time = last_time
                    for idx in order:
                        manual = self.logic_index_to_step.get(idx, {}).get('manual')
                        if manual is None or manual not in pickup_schedule:
                            continue
                        pickup_time = pickup_schedule[manual]
                        seg = []
                        for pos, t in path:
                            if t > seg_start_time and t <= pickup_time + 1e-6:
                                seg.append((pos, t))
                        seg_steps = self._map_positions_to_step_indices([pos for pos, _ in seg], atol=1e-2)
                        print(
                            f"              从台阶{fmt_idx(last_idx)}({seg_start}) -> 台阶{fmt_idx(idx)}，耗时: {pickup_time - seg_start_time:.2f}s，路径: {seg_steps}",
                            flush=True)
                        seg_start = self.logic_index_to_step[idx]['center']
                        seg_start_time = pickup_time
                        last_idx = idx
                    try:
                        pre_last_pos, pre_last_time = path[-1] if path else (None, None)
                        if pre_last_pos is None:
                            print("            [出口预检] 无法确定拾取结束位置，跳过预检", flush=True)
                        else:
                            exit_obstacles_dbg = set(r1_non_primary_indices)
                            if self.fake_manual:
                                exit_obstacles_dbg.add(self.fake_manual.step_index)
                            candidate_exits_dbg = [
                                idx for idx, step in self.logic_index_to_step.items()
                                if step['row'] == 1
                                   and idx not in exit_obstacles_dbg
                                   and step['height'] <= self.r2_max_step_height
                            ]
                            if not candidate_exits_dbg:
                                print("            [出口预检] 无可用出口台阶（行1均被阻断）", flush=True)
                            else:
                                min_dist = float('inf')
                                nearest_idx_dbg = None
                                for idx_ce in candidate_exits_dbg:
                                    dist = _dist(self.logic_index_to_step[idx_ce]['center'], pre_last_pos)
                                    if dist < min_dist:
                                        min_dist = dist
                                        nearest_idx_dbg = idx_ce
                                step_dbg = self.logic_index_to_step[nearest_idx_dbg]
                                t_arrive_dbg = pre_last_time + (min_dist / PARAMS['v2'])
                                path_exit_dbg = [(step_dbg['center'], t_arrive_dbg)]
                                ground_pos_dbg = self.get_adjacent_ground(step_dbg)
                                last_pos2_dbg, last_time2_dbg = step_dbg['center'], t_arrive_dbg
                                if ground_pos_dbg:
                                    move_time2_dbg = _dist(step_dbg['center'], ground_pos_dbg) / PARAMS['v2s']
                                    t_ground_dbg = t_arrive_dbg + move_time2_dbg
                                    path_exit_dbg.append((ground_pos_dbg, t_ground_dbg))
                                    last_pos2_dbg, last_time2_dbg = ground_pos_dbg, t_ground_dbg
                                ramp_pos_dbg = PARAMS['ramp_pos']
                                if not np.allclose(last_pos2_dbg, ramp_pos_dbg, atol=0.1):
                                    move_time3_dbg = _dist(last_pos2_dbg, ramp_pos_dbg) / PARAMS['v2s']
                                    path_exit_dbg.append((ramp_pos_dbg, last_time2_dbg + move_time3_dbg))
                                pre_last_idx = self.find_step_by_pos(pre_last_pos)
                                print(
                                    f"            [出口预检] 起点: 台阶{pre_last_idx} at {np.round(pre_last_pos, 2)}, t={pre_last_time:.2f}",
                                    flush=True)
                                print(
                                    f"            [出口预检] 最近出口台阶: 台阶{nearest_idx_dbg} at {np.round(step_dbg['center'], 2)}",
                                    flush=True)
                                exit_steps_dbg = self._map_positions_to_step_indices(
                                    [pos_dbg for pos_dbg, _ in path_exit_dbg], atol=1e-2)
                                end_pos_dbg = path_exit_dbg[-1][0]
                                reached_dbg = np.allclose(end_pos_dbg, ramp_pos_dbg, atol=0.1)
                                print(
                                    f"            [出口预检] 预检路径经过台阶: {exit_steps_dbg}，末端: {np.round(end_pos_dbg, 2)}，到达终点: {reached_dbg}",
                                    flush=True)
                    except Exception as _e:
                        print(f"            [出口预检] 调试时出现异常: {_e}", flush=True)
                    print(
                        f"            >>> 可行分支：当前耗时={current_time:.2f}s，对比best_total_time={float('inf') if best_total_time == float('inf') else round(best_total_time, 2)}s",
                        flush=True)
                    if current_time < best_total_time:
                        print(
                            f"            [选择最佳] 更新最佳顺序: {[f'台阶{idx}' for idx in order]}，拾取耗时={current_time:.2f}s (原best={float('inf') if best_total_time == float('inf') else round(best_total_time, 2)}s)",
                            flush=True
                        )
                        best_total_time = current_time
                        best_path = path.copy()
                        best_pickup_schedule = pickup_schedule.copy()
                        best_order = list(order)
                        best_pickup_debug_info = pickup_debug_info.copy()
                        best_output_pickup_info = output_pickup_info
                    else:
                        print("            >>> 可行分支：未更新best_*（当前不优于best）", flush=True)

            if valid and len(pickup_schedule) < PARAMS['r2_manual_needed']:
                print(
                    f"        顺序 {[f'台阶{idx}' for idx in order]}: 部分完成，拾取数={len(pickup_schedule)}，继续评估其它顺序",
                    flush=True
                )

        # 组合结束汇总
        print(
            f"      >> 组合循环结束：可行顺序数量={len(feasible_orders)}，best是否已设置：path={best_path is not None}, schedule={best_pickup_schedule is not None}, order={best_order is not None}",
            flush=True)
        if feasible_orders:
            print("      >> 本R2组合所有可行顺序及耗时：")
            for ord_, t_ in feasible_orders:
                print(f"         顺序 {[f'台阶{idx}' for idx in ord_]}，耗时: {t_:.2f}s")
        else:
            print("      >> 本R2组合无可行顺序。")

        if (best_path is None or best_pickup_schedule is None or best_order is None) and feasible_orders_full:
            fallback = min(feasible_orders_full, key=lambda x: x[1])
            best_order, best_total_time, best_path, best_pickup_schedule = fallback[0], fallback[1], fallback[2], \
                fallback[3]
            print(
                f"      >> [回退] 使用可行顺序中的最优方案：{[f'台阶{idx}' for idx in best_order]}，拾取耗时={best_total_time:.2f}s",
                flush=True)

        return best_path, best_pickup_schedule, best_order, best_total_time, feasible_orders, feasible_orders_full

    def _build_exit_path(self, best_path, r1_non_primary_indices):
        """根据拾取结束点用 A* 构建出口路径，避免台阶区内的直线“斜走”。返回 (exit_path, exit_time, exit_valid)。"""
        exit_obstacles = r1_non_primary_indices.copy()
        if self.fake_manual:
            exit_obstacles.add(self.fake_manual.step_index)

        # 保留原有判定日志（仅列出可作为出口的行1台阶，供诊断），但真正路径由 A* 产生
        candidate_exits = [
            idx for idx, step in self.logic_index_to_step.items()
            if step['row'] == 1
               and idx not in exit_obstacles
               and step['height'] <= self.r2_max_step_height
        ]
        print(f"    [出口判定] 障碍台阶={sorted(list(exit_obstacles))}，候选出口={sorted(candidate_exits)}", flush=True)

        last_pos, last_time = best_path[-1] if best_path else (None, None)
        print(
            f"    [出口判定] R2拾取结束位置={np.round(last_pos, 2) if last_pos is not None else None}，t={last_time:.2f}",
            flush=True)

        # 使用 A* 直接规划到终点（斜坡），内部会在台阶->地面时遵守高度与邻接规则
        path_astar, exit_time, end_pos = self._find_path_to_exit(
            start_pos=last_pos,
            start_time=last_time,
            r1_primary_manuals=[],
            manuals_to_ignore=exit_obstacles,
            remaining_r2_indices=None,
        )

        if not path_astar:
            print("    [出口路径] A* 未找到出口路径", flush=True)
            return [], float('inf'), False

        # 去掉起点（与拾取末位置相同），只保留实际移动段
        exit_path = path_astar[1:] if len(path_astar) >= 2 else []
        if exit_path:
            print(f"    [出口路径] 使用 A* 规划，共 {len(exit_path)} 段，出口用时(相对拾取末时刻)={exit_time:.2f}",
                  flush=True)
        else:
            print(f"    [出口路径] 起点已在终点，无需额外移动", flush=True)

        exit_valid = exit_path and np.allclose(exit_path[-1][0], PARAMS['ramp_pos'], atol=0.1)
        if not exit_valid:
            end_pos_dbg = exit_path[-1][0] if exit_path else None
            print(
                f"    [调试] R2出口路径失败: end_pos={np.round(end_pos_dbg, 2) if end_pos_dbg is not None else None}, ramp={PARAMS['ramp_pos']}",
                flush=True
            )
        else:
            print(
                f"    [调试] R2出口路径成功: 路径长度={len(exit_path)}, 用时={exit_time:.2f}，末端={np.round(exit_path[-1][0], 2)}",
                flush=True
            )
        return exit_path, exit_time, exit_valid

    def _integrate_r2_exit(self, r2_full_path, exit_path, exit_time):
        """将出口路径与拾取路径衔接，并返回(full_r2_path_tuples, r2_final_time)。保持原日志。"""
        r2_exit_start_time = r2_full_path[-1][1] if r2_full_path else 0
        exit_path_fixed = exit_path.copy()
        if exit_path_fixed:
            last_pos, last_time = r2_full_path[-1] if r2_full_path else (None, None)
            first_exit_pos, first_exit_time = exit_path_fixed[0]
            if last_pos is not None and np.allclose(last_pos, first_exit_pos, atol=1e-6):
                if abs(first_exit_time - last_time) > 1e-6:
                    exit_path_fixed[0] = (first_exit_pos, last_time)
                    print(
                        f"    [衔接修正] 首段时间对齐：pos={np.round(first_exit_pos, 2)}，from {first_exit_time:.4f} -> {last_time:.4f}",
                        flush=True
                    )
            elif last_pos is not None:
                move_time = _dist(last_pos, first_exit_pos) / PARAMS['v2']
                move_arrival_time = last_time + move_time
                move_segment = []
                if not np.allclose(last_pos, first_exit_pos, atol=1e-6):
                    move_segment = [(first_exit_pos, move_arrival_time)]
                    time_offset = move_arrival_time - first_exit_time
                    exit_path_fixed = move_segment + [(p, t + time_offset) for (p, t) in exit_path_fixed]
                    print(
                        f"    [衔接修正] 追加移动段：{np.round(last_pos, 2)} -> {np.round(first_exit_pos, 2)}，到达t={move_arrival_time:.2f}；整体平移{time_offset:.4f}s",
                        flush=True
                    )
                else:
                    if abs(first_exit_time - last_time) > 1e-6:
                        exit_path_fixed[0] = (first_exit_pos, last_time)
                        print(
                            f"    [衔接修正] 位置相同时间修正：pos={np.round(first_exit_pos, 2)}，from {first_exit_time:.4f} -> {last_time:.4f}",
                            flush=True
                        )
        full_r2_path_tuples = r2_full_path + exit_path_fixed
        r2_final_time = r2_exit_start_time + (exit_time if exit_time is not None else 0)
        print(
            f"    [出口计时] r2_exit_start_time={r2_exit_start_time:.2f}，exit_time={0 if exit_time is None else round(exit_time, 2)}，r2_final_time={r2_final_time:.2f}",
            flush=True
        )
        return full_r2_path_tuples, r2_final_time

    def _finalize_and_update_global(self, r1_primary_choice, r1_forest_entry, phase1_end_time,
                                    r2_final_time, global_best_time,
                                    initial_r1_path, initial_r1_ts,
                                    initial_r2_path_local, initial_r2_ts_local,
                                    full_r2_path_tuples, r2_pickup_schedule):
        """规划R1路径，计算总时长并尝试更新全局最优方案。保持原日志。返回更新后的(global_best_time, global_best_plan)。"""
        # 只有在total_time更优时才更新best_total_time（保持原有变量行为）
        if r2_final_time is None:
            print(f"[DEBUG] 跳过无效方案: r2_final_time={r2_final_time}, error=NoneType detected")
            return global_best_time, self.best_plan

        # 规划R1
        r1_main_tasks = list(r1_primary_choice)
        print(
            f"    - R1任务: {[f'台阶{self.logic_index_to_step[m.step_index]['logic_index']}' for m in r1_main_tasks]}")
        # 计算 R2 首次到达各台阶的时间（仅对真实台阶，忽略伪台阶），用于指导 R1 拾取顺序
        r2_first_arrival: dict[int, float] = {}
        try:
            # 仅收集基础台阶（非伪台阶）的中心坐标到逻辑索引映射
            base_steps = [s for s in self.steps if not s.get('is_pseudo')]
            pos_to_logic_index = {tuple(np.round(s['center'], 2)): int(s['logic_index']) for s in base_steps}
            # 仅关注本次 R1 主任务涉及的逻辑索引集合
            r1_task_indices = set(int(m.step_index) for m in r1_main_tasks if hasattr(m, 'step_index'))
            for pos, t in (full_r2_path_tuples or []):
                key = tuple(np.round(pos, 2))
                idx = pos_to_logic_index.get(key)
                if idx is None:
                    continue
                if r1_task_indices and idx not in r1_task_indices:
                    continue
                if idx not in r2_first_arrival:
                    r2_first_arrival[idx] = float(t)
        except Exception:
            # 容错：如构建失败则回退为不使用该排序信息
            r2_first_arrival = {}

        r1_path, r1_ts, r1_pickup_schedule = self._plan_r1_path(
            r1_forest_entry, phase1_end_time, r1_main_tasks, r2_first_arrival=r2_first_arrival or None
        )

        if r1_ts:
            r1_end_time = r1_ts[-1]
            r1_final_pos = r1_path[-1]
        else:
            r1_end_time = phase1_end_time
            r1_final_pos = r1_forest_entry
        if not np.allclose(r1_final_pos, PARAMS['ramp_pos'], atol=1e-2):
            # --- 使用 5x6 网格系统规划回程路径 ---
            # 定义网格参数 (需与 _plan_r1_path 保持一致)
            GRID_ORIGIN_X = 0.6
            GRID_ORIGIN_Y = 0.8
            GRID_STEP = 1.2
            GRID_COLS = 5
            GRID_ROWS = 6

            def grid_to_pos(r, c):
                return np.array([GRID_ORIGIN_X + c * GRID_STEP, GRID_ORIGIN_Y + r * GRID_STEP])

            def pos_to_grid(pos):
                c = int(round((pos[0] - GRID_ORIGIN_X) / GRID_STEP))
                r = int(round((pos[1] - GRID_ORIGIN_Y) / GRID_STEP))
                c = max(0, min(GRID_COLS - 1, c))
                r = max(0, min(GRID_ROWS - 1, r))
                return (r, c)

            # 定义可行走区域（外围一圈）
            valid_grids = set()
            for r in range(GRID_ROWS):
                for c in range(GRID_COLS):
                    if r == 0 or r == GRID_ROWS - 1 or c == 0 or c == GRID_COLS - 1:
                        valid_grids.add((r, c))

            # 构建邻接图（4邻域 - 曼哈顿）
            adj = {}
            for r, c in valid_grids:
                neighbors = []
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in valid_grids:
                        dist = math.hypot(dr * GRID_STEP, dc * GRID_STEP)
                        neighbors.append(((nr, nc), dist))
                adj[(r, c)] = neighbors

            # A* 寻路
            def astar_grid(start_node, end_node):
                if start_node == end_node:
                    return [start_node], 0.0
                pq = [(0, 0, start_node, [start_node])]
                visited = {}
                while pq:
                    f, g, curr, p = heapq.heappop(pq)
                    if curr in visited and visited[curr] <= g: continue
                    visited[curr] = g
                    if curr == end_node: return p, g
                    for next_node, dist in adj.get(curr, []):
                        new_g = g + dist
                        # 曼哈顿距离启发
                        h = abs(end_node[0] - next_node[0]) * GRID_STEP + abs(end_node[1] - next_node[1]) * GRID_STEP
                        heapq.heappush(pq, (new_g + h, new_g, next_node, p + [next_node]))
                return None, float('inf')

            # 执行回程规划
            start_grid = pos_to_grid(r1_final_pos)
            ramp_pos = np.array(PARAMS['ramp_pos'])
            
            # 找到离出口最近的合法网格点
            target_grid = min(valid_grids, key=lambda g: _dist(ramp_pos, grid_to_pos(*g)))
            
            # 如果起点不在合法网格中（例如从非网格点出发），先找到最近的合法网格
            if start_grid not in valid_grids:
                start_grid = min(valid_grids, key=lambda g: _dist(r1_final_pos, grid_to_pos(*g)))

            curr_pos = np.array(r1_final_pos)
            
            # 1. 规划网格路径
            grid_path, _ = astar_grid(start_grid, target_grid)
            
            if grid_path:
                # 如果当前位置离起点网格有距离，先走到起点网格
                start_grid_pos = grid_to_pos(*start_grid)
                if not np.allclose(curr_pos, start_grid_pos, atol=1e-2):
                     dist = _dist(curr_pos, start_grid_pos)
                     r1_end_time += dist / PARAMS['v1s']
                     r1_path.append(tuple(start_grid_pos))
                     r1_ts.append(r1_end_time)
                     curr_pos = start_grid_pos

                # 沿网格路径移动
                for g_node in grid_path[1:]: # 跳过第一个点（即start_grid）
                    next_pos = grid_to_pos(*g_node)
                    dist = _dist(curr_pos, next_pos)
                    r1_end_time += dist / PARAMS['v1s']
                    r1_path.append(tuple(next_pos))
                    r1_ts.append(r1_end_time)
                    curr_pos = next_pos
            
            # 2. 从最后一个网格点走到斜坡终点
            dist_ramp = _dist(curr_pos, ramp_pos)
            r1_end_time += dist_ramp / PARAMS['v1s']
            r1_path.append(tuple(ramp_pos))
            r1_ts.append(r1_end_time)
            
            r1_final_pos = tuple(ramp_pos)
        r1_final_time = r1_end_time

        try:
            total_time = r2_final_time
            print(f"    - 此方案总耗时: {total_time:.2f}s (R2: {r2_final_time:.2f}s, R1: {r1_final_time:.2f}s)")
        except Exception as e:
            print(f"[DEBUG] 跳过无效方案: r2_final_time={r2_final_time}, error={e}")
            return global_best_time, self.best_plan

        # 决策逻辑：优先 R2 时间，若 R2 时间相差在 0.05s 以内，则对比 R1 时间
        is_better = False
        if self.best_plan is None:
            is_better = True
        else:
            current_best_r2 = self.best_plan['r2_final_time']
            current_best_r1 = self.best_plan['r1_final_time']
            
            if r2_final_time < current_best_r2 - 0.05:
                is_better = True
            elif r2_final_time > current_best_r2 + 0.05:
                is_better = False
            else:
                # R2 时间相近，比较 R1 时间
                if r1_final_time < current_best_r1:
                    is_better = True
                elif abs(r1_final_time - current_best_r1) < 1e-3:
                    # R1 也相近，则取 R2 更小的那个
                    if r2_final_time < current_best_r2:
                        is_better = True

        if is_better:
            global_best_time = total_time
            self.best_plan = {
                'r1_path': initial_r1_path + r1_path,
                'r1_timestamps': initial_r1_ts + r1_ts,
                'r2_path': initial_r2_path_local + [p for p, t in full_r2_path_tuples],
                'r2_timestamps': initial_r2_ts_local + [t for p, t in full_r2_path_tuples],
                'r1_final_time': r1_final_time,
                'r2_final_time': r2_final_time,
                'total_time': total_time,
                'r1_pickup_schedule': r1_pickup_schedule,
                'r2_pickup_schedule': r2_pickup_schedule,
                # 新增：拼接武器完成时间与位置（来自初始阶段计算）
                'weapon_assembly_time': getattr(self, 'weapon_assembly_time', None),
                'weapon_assembly_pos': getattr(self, 'weapon_assembly_pos', None),
            }
        return global_best_time, self.best_plan

    def _plan_initial_phase(self):
        """规划初始阶段路径：机器人到达各自的初始工作位置"""
        # 初始化路径和时间戳
        r1_path, r1_ts, r2_path, r2_ts = [PARAMS['r1_start_pos']], [0], [PARAMS['r2_start_pos']], [0]

        # 计算R1到达支撑点的时间
        t_r1_sup = _dist(PARAMS['r1_start_pos'], PARAMS['support_pos']) / PARAMS['v1']
        r1_path.append(PARAMS['support_pos'])
        r1_ts.append(t_r1_sup)

        # 计算R2到达长矛位置的时间（应使用R2速度）
        t_r2_spear = _dist(PARAMS['r2_start_pos'], PARAMS['spear_pos']) / PARAMS['v2']
        r2_path.append(PARAMS['spear_pos'])
        r2_ts.append(t_r2_spear)

        # 等待双方到达后，执行拾取动作
        t_after_pickup = max(t_r1_sup, t_r2_spear) + PARAMS['s1']
        r1_path.append(PARAMS['support_pos'])
        r1_ts.append(t_after_pickup)
        r2_path.append(PARAMS['spear_pos'])
        r2_ts.append(t_after_pickup)

        # 更新当前状态
        current_time, r1_pos, r2_pos = t_after_pickup, np.array(PARAMS['support_pos']), np.array(PARAMS['spear_pos'])

        # 处理武器传递
        for i in range(PARAMS['weapon_count']):
            # R2移动到R1位置（使用R2速度）
            current_time += _dist(r2_pos, r1_pos) / PARAMS['v2']
            r2_pos = r1_pos
            r2_path.append(tuple(r2_pos))
            r2_ts.append(current_time)
            r1_path.append(tuple(r1_pos))
            r1_ts.append(current_time)

            # 执行武器传递动作
            current_time += PARAMS['s3']
            r1_path.append(tuple(r1_pos))
            r1_ts.append(current_time)
            r2_path.append(tuple(r2_pos))
            r2_ts.append(current_time)

            # 如果不是最后一个武器，R2返回长矛位置
            if i < PARAMS['weapon_count'] - 1:
                current_time += _dist(r2_pos, np.array(PARAMS['spear_pos'])) / PARAMS['v2']
                r2_pos = np.array(PARAMS['spear_pos'])
                r2_path.append(tuple(r2_pos))
                r2_ts.append(current_time)
                r1_path.append(tuple(r1_pos))
                r1_ts.append(current_time)

        # 记录“拼接武器完成”的时间戳与位置（位于支撑点处）；便于后续输出与分析
        try:
            self.weapon_assembly_time = float(current_time)
            # 拼接发生在 R1 所在支撑点位置
            self.weapon_assembly_pos = tuple(r1_pos)
        except Exception:
            # 容错：如上面变量不存在则忽略记录
            self.weapon_assembly_time = None
            self.weapon_assembly_pos = None

        # R1不再走到固定点；仅计算“离开上层区域”的最早时刻，作为R2进入下层的先后门槛
        y_boundary = 7.0
        dy = max(0.0, float(r1_pos[1]) - y_boundary)
        t_exit = dy / max(1e-9, float(PARAMS['v1']))
        phase1_end_time = current_time + t_exit

        # 起点传递使用当前R1位置（不追加任何移动段）
        r1_forest_entry = np.array(r1_pos)

        return r1_path, r1_ts, r2_path, r2_ts, phase1_end_time, r1_forest_entry, r2_pos

    def _plan_r1_path(self, start_pos, start_time, manuals_to_collect, r2_first_arrival: dict | None = None):
        """为R1规划拾取秘籍的路径（基于5x6网格的A*搜索与全排列枚举）"""
        path, timestamps, pickup_schedule = [], [], {}
        if not manuals_to_collect:
            return path, timestamps, pickup_schedule

        # --- 1. 定义 5x6 网格系统 ---
        # 坐标系：Row 0..5 (y), Col 0..4 (x)
        # 物理坐标公式：x = 0.6 + c * 1.2, y = 0.8 + r * 1.2
        GRID_ORIGIN_X = 0.6
        GRID_ORIGIN_Y = 0.8
        GRID_STEP = 1.2
        GRID_COLS = 5
        GRID_ROWS = 6

        def grid_to_pos(r, c):
            return np.array([GRID_ORIGIN_X + c * GRID_STEP, GRID_ORIGIN_Y + r * GRID_STEP])

        def pos_to_grid(pos):
            # 找最近的网格中心
            c = int(round((pos[0] - GRID_ORIGIN_X) / GRID_STEP))
            r = int(round((pos[1] - GRID_ORIGIN_Y) / GRID_STEP))
            c = max(0, min(GRID_COLS - 1, c))
            r = max(0, min(GRID_ROWS - 1, r))
            return (r, c)

        # --- 2. 定义可行走区域（外围一圈） ---
        # 台阶区在中间：Row 1..4, Col 1..3
        # 可行走：Row 0, Row 5, Col 0, Col 4
        valid_grids = set()
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if r == 0 or r == GRID_ROWS - 1 or c == 0 or c == GRID_COLS - 1:
                    valid_grids.add((r, c))

        # --- 3. 构建邻接图（4邻域 - 曼哈顿） ---
        adj = {}
        for r, c in valid_grids:
            neighbors = []
            # 仅允许上下左右移动，不允许斜向
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) in valid_grids:
                    dist = math.hypot(dr * GRID_STEP, dc * GRID_STEP)
                    neighbors.append(((nr, nc), dist))
            adj[(r, c)] = neighbors

        # --- 4. A* 寻路 (Grid -> Grid) ---
        def astar_grid(start_node, end_node, speed):
            if start_node == end_node:
                return [start_node], 0.0

            # (f, g, current_node, path_list)
            pq = [(0, 0, start_node, [start_node])]
            visited = {}

            while pq:
                f, g, curr, p = heapq.heappop(pq)

                if curr in visited and visited[curr] <= g:
                    continue
                visited[curr] = g

                if curr == end_node:
                    return p, g

                for next_node, dist in adj.get(curr, []):
                    new_g = g + dist / speed
                    # 启发式：欧氏距离
                    h_dist = math.hypot((end_node[0] - next_node[0]) * GRID_STEP,
                                        (end_node[1] - next_node[1]) * GRID_STEP)
                    h = h_dist / speed
                    heapq.heappush(pq, (new_g + h, new_g, next_node, p + [next_node]))
            return None, float('inf')

        # --- 5. 确定起点与目标点 ---
        start_grid = pos_to_grid(start_pos)
        # 若起点不在合法网格（如在支撑点），找最近的合法入口
        if start_grid not in valid_grids:
            start_grid = min(valid_grids, key=lambda g: _dist(start_pos, grid_to_pos(*g)))

        # 映射每个秘籍到最佳站位点
        manual_targets = []
        for m in manuals_to_collect:
            step_idx = getattr(m, 'step_index', -1)
            target_grid = None
            if step_idx != -1 and step_idx in self.logic_index_to_step:
                step = self.logic_index_to_step[step_idx]
                # 直接使用台阶物理坐标计算其所在的网格坐标
                step_grid_r, step_grid_c = pos_to_grid(step['center'])
                
                # 在该网格的4邻域中寻找合法的站位点（valid_grids）
                candidates = []
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nr, nc = step_grid_r + dr, step_grid_c + dc
                    if (nr, nc) in valid_grids:
                        candidates.append((nr, nc))
                
                if candidates:
                    # 如果有多个候选，优先选择距离当前位置最近的，或者简单地选第一个
                    # 这里为了确定性，可以选距离台阶中心最近的（虽然逻辑上都一样近，都是1格距离）
                    # 或者优先选左右通道（col=0或4），其次上下通道
                    def sort_key(g):
                        # 优先 col=0 或 col=4 (左右通道)
                        is_side = (g[1] == 0 or g[1] == GRID_COLS - 1)
                        return (0 if is_side else 1, g)
                    
                    candidates.sort(key=sort_key)
                    target_grid = candidates[0]

            # 兜底：找最近的合法网格
            if target_grid is None:
                target_grid = min(valid_grids, key=lambda g: _dist(m.pos, grid_to_pos(*g)))

            manual_targets.append((m, target_grid))

        # --- 6. 全排列枚举寻找最优顺序 ---
        best_total_time = float('inf')
        best_path_points = []
        best_timestamps = []
        best_schedule = {}

        # 初始移动段（从 start_pos 到 start_grid）
        # 注意：这段是公共的，但为了方便放在循环里或者单独算
        # 这里简单处理：先算好这一小段
        grid_start_pos = grid_to_pos(*start_grid)
        initial_move_time = 0.0
        initial_path = [tuple(start_pos)]
        initial_ts = [start_time]

        if not np.allclose(start_pos, grid_start_pos, atol=1e-2):
            dt = _dist(start_pos, grid_start_pos) / PARAMS['v1']
            initial_move_time = dt
            initial_path.append(tuple(grid_start_pos))
            initial_ts.append(start_time + dt)

        base_time = start_time + initial_move_time

        for perm in permutations(manual_targets):
            current_grid = start_grid
            current_time = base_time

            # 复制初始路径
            path_points = list(initial_path)
            timestamps = list(initial_ts)
            local_schedule = {}
            valid_perm = True

            for i, (manual, target_grid) in enumerate(perm):
                # 第一本之后加速
                speed = PARAMS['v1'] if i == 0 else PARAMS['v1s']

                # A* 规划
                grid_path, _ = astar_grid(current_grid, target_grid, speed)
                if grid_path is None:
                    valid_perm = False
                    break

                # 转换路径点（跳过起点，因为起点就是 current_grid）
                for g_node in grid_path[1:]:
                    pos = grid_to_pos(*g_node)
                    prev_pos = grid_to_pos(*grid_path[grid_path.index(g_node) - 1])
                    dt = _dist(prev_pos, pos) / speed
                    current_time += dt
                    path_points.append(tuple(pos))
                    timestamps.append(current_time)

                # 拾取停留
                dwell = float(PARAMS.get('r1_pickup_dwell', 1.0))
                current_time += dwell
                path_points.append(tuple(grid_to_pos(*target_grid)))
                timestamps.append(current_time)
                local_schedule[manual] = current_time

                current_grid = target_grid

            if valid_perm:
                if current_time < best_total_time:
                    best_total_time = current_time
                    best_path_points = path_points
                    best_timestamps = timestamps
                    best_schedule = local_schedule

        return best_path_points, best_timestamps, best_schedule

    def _find_path_to_target(self, start_pos, start_time, r1_primary_manuals, manuals_to_ignore, goal_func,
                             goal_pos=None, remaining_r2_indices=None, is_exit_path=False, max_iterations=10000,
                             v_override=None, debug=False, include_exit_future: bool = True,
                             exit_future_weight: float = 1.0,
                             exit_bias_g_weight: float = 2.0,
                             goal_context_key=None, preferred_row_key=None):
        """A*算法实现的路径搜索函数，寻找从起点到目标的最优路径"""
        # 设置移动速度
        v_normal = v_override if v_override is not None else PARAMS['v2']
        v_special = v_override if v_override is not None else PARAMS['v2s']

        if remaining_r2_indices is None:
            remaining_r2_indices = set()

        # 确定起始位置所在的台阶
        start_node_idx = self.find_step_by_pos(start_pos)
        is_inside_initial = (start_node_idx != -1)
        is_forest = False
        if start_node_idx == -1:
            # 检查是否在地面
            on_ground = False
            for step in self.steps:
                adj_g = self.get_adjacent_ground(step)
                if adj_g and np.allclose(start_pos, adj_g):
                    on_ground = True
                    break
            if not on_ground:
                is_forest = True
                start_node_idx = -2  # 森林状态标记
            else:
                is_inside_initial = False

        start_pos_key = tuple(np.round(start_pos, 2))  # 用于缓存的位置键

        # 路径缓存检查（避免重复计算相同路径）
        cache_key = (
            start_pos_key,
            goal_func.__code__.co_code,
            frozenset(m.label for m in r1_primary_manuals),
            frozenset(manuals_to_ignore),
            frozenset(remaining_r2_indices or []),
            bool(is_exit_path),
            bool(include_exit_future),
            float(exit_future_weight) if exit_future_weight is not None else None,
            float(exit_bias_g_weight) if exit_bias_g_weight is not None else None,
            goal_context_key,
            preferred_row_key,
        )
        if cache_key in self._path_cache:
            cached_path, cached_time, cached_end_pos, cached_end_idx = self._path_cache[cache_key]
            if cached_path is not None:
                # 调整时间戳以匹配起始时间
                adjusted_path = [(pos, t - cached_path[0][1] + start_time) for pos, t in cached_path]
                return adjusted_path, cached_time, cached_end_pos, cached_end_idx
            else:
                return None, float('inf'), None, -1
        # 组合所有障碍物
        all_obstacles = manuals_to_ignore.copy()
        if self.fake_manual:
            all_obstacles.add(self.fake_manual.step_index)
        all_obstacles.update(remaining_r2_indices)

        # 将R1主任务台阶从障碍物中移除（R2可以经过这些台阶）
        current_obstacles = all_obstacles.copy()
        for m in r1_primary_manuals:
            if m.step_index in current_obstacles:
                current_obstacles.remove(m.step_index)

        # 预计算：行1台阶从相邻地面到斜坡的时间（用于未来出口代价校正）
        row1_ground_to_ramp = {}
        row1_time_by_col = {}
        min_row1_ground_time = None
        if include_exit_future:
            ramp_pos = np.array(PARAMS['ramp_pos'])
            for step in self.steps:
                if step['row'] == 1:
                    gpos = self.get_adjacent_ground(step)
                    if gpos is not None:
                        t_ground = np.linalg.norm(ramp_pos - np.array(gpos)) / (v_special if v_special else 1.0)
                        row1_ground_to_ramp[step['logic_index']] = t_ground
                        row1_time_by_col[step['col']] = t_ground
                        if (min_row1_ground_time is None) or (t_ground < min_row1_ground_time):
                            min_row1_ground_time = t_ground
        if min_row1_ground_time is None:
            min_row1_ground_time = 0.0

        # 启发式函数
        def heuristic_idx(pos, idx_for_addon: int):
            if goal_pos is not None:
                base = np.sum(np.abs(np.array(pos) - np.array(goal_pos))) / v_normal
            else:
                # 目标不固定时：使用到最近出口地面的距离
                min_dist = float('inf')
                for step in self.steps:
                    exit_pos = self.get_adjacent_ground(step)
                    if exit_pos:
                        dist = np.linalg.norm(np.array(pos) - np.array(exit_pos))
                        min_dist = min(min_dist, dist)
                base = min_dist / v_normal

            if include_exit_future and (min_row1_ground_time is not None):
                addon = 0.0
                if idx_for_addon is not None and idx_for_addon >= 0 and idx_for_addon in self.logic_index_to_step:
                    # 用该节点所属列的行1台阶时间偏置
                    col = self.logic_index_to_step[idx_for_addon]['col']
                    t_col = row1_time_by_col.get(col)
                    if t_col is not None:
                        addon = max(0.0, t_col - min_row1_ground_time)
                # 若不是台阶节点，不加偏置
                return base + exit_future_weight * addon
            return base

        # 优先队列初始化（A*算法的核心）
        # (f值, g值, 位置, 台阶索引, 是否在台阶上, 上一移动轴, 是否暂停, min_row)
        min_row_init = None
        if start_node_idx >= 0:
            min_row_init = self.logic_index_to_step[start_node_idx]['row']
        else:
            min_row_init = None
        pq = [(0, start_time, start_pos, start_node_idx, is_inside_initial, None, False, min_row_init)]
        visited = {}  # 记录已访问状态
        state_key = (start_node_idx, start_pos_key, is_inside_initial or is_forest, None, False, min_row_init)
        visited[state_key] = {'cost': start_time, 'parent': None, 'pos': start_pos, 'min_row': min_row_init}

        iterations = 0
        while pq and iterations < max_iterations:
            iterations += 1

            # 取出f值最小的节点
            f, g, current_pos, current_idx, is_inside, prev_axis, is_paused, min_row = heapq.heappop(pq)

            current_pos_key = tuple(np.round(current_pos, 2))
            state_key = (current_idx, current_pos_key, is_inside or (current_idx == -2), prev_axis, is_paused, min_row)
            # 如果当前状态的成本更高，跳过
            if debug:
                print(
                    f"[A*] 当前节点: 位置={current_pos}, 台阶idx={current_idx}, is_inside={is_inside}, prev_axis={prev_axis}, is_paused={is_paused}, min_row={min_row}, f={f:.2f}, g={g:.2f}")
                print(f"[A*] 当前障碍物集合: {sorted(list(all_obstacles))}")
            if g > visited.get(state_key, {}).get('cost', float('inf')):
                if debug:
                    print(
                        f"[A*] 跳过节点: 位置={current_pos}, 台阶idx={current_idx}, g={g:.2f} > 已访问cost={visited.get(state_key, {}).get('cost', float('inf')):.2f}")
                continue

            # 检查是否到达目标
            if goal_func(current_pos, current_idx):
                # 回溯构建路径
                path = []
                crawl_pos, crawl_idx, crawl_is_inside, crawl_prev_axis, crawl_is_paused, crawl_min_row = current_pos, current_idx, is_inside, prev_axis, is_paused, min_row
                while True:
                    crawl_pos_key = tuple(np.round(crawl_pos, 2))
                    state_key = (crawl_idx, crawl_pos_key, crawl_is_inside or (crawl_idx == -2), crawl_prev_axis,
                                 crawl_is_paused, crawl_min_row)
                    node_info = visited.get(state_key)
                    if not node_info:
                        break
                    path.append((node_info['pos'], node_info['cost']))
                    parent_info = node_info.get('parent')
                    if parent_info:
                        crawl_pos, crawl_idx, crawl_is_inside, crawl_prev_axis, crawl_is_paused, crawl_min_row = parent_info
                    else:
                        break
                path.reverse()  # 反转路径以获得正确顺序
                if debug:
                    debug_path_str = []
                    for i in range(1, len(path)):
                        prev_pos, _ = path[i - 1]
                        curr_pos, _ = path[i]
                        prev_idx = self.find_step_by_pos(prev_pos)
                        curr_idx = self.find_step_by_pos(curr_pos)
                        debug_path_str.append(f"{prev_idx}->{curr_idx}")
                        if prev_idx != -1 and curr_idx != -1:
                            if curr_idx not in self.logic_index_to_step[prev_idx]['neighbors']:
                                print(f"[A*路径非法] 路径: {' | '.join(debug_path_str)}")
                                raise RuntimeError(f"A*路径不合法：台阶{prev_idx}和{curr_idx}不是相邻台阶！")
                    print(f"[A*] 路径回溯: {' | '.join(debug_path_str)}，总步数: {len(path)}")
                # 存储到缓存
                self._path_cache[cache_key] = (path, g - start_time, current_pos, current_idx)
                return path, g - start_time, current_pos, current_idx

            # 增加停顿状态转换（用于处理需要停留的动作）
            if not is_paused:
                pause_state_key = (current_idx, current_pos_key, is_inside or (current_idx == -2), prev_axis, True,
                                   min_row)
                if pause_state_key not in visited or g < visited[pause_state_key]['cost']:
                    visited[pause_state_key] = {'cost': g,
                                                'parent': (current_pos, current_idx, is_inside, prev_axis, is_paused,
                                                           min_row),
                                                'pos': current_pos,
                                                'min_row': min_row}
                    heapq.heappush(pq, (f, g, current_pos, current_idx, is_inside, prev_axis, True, min_row))
                    if debug:
                        print(
                            f"[A*] 节点入队: 位置={current_pos}, 台阶idx={current_idx}, is_paused=True, min_row={min_row}, f={f:.2f}, g={g:.2f}")

            # 动态更新障碍物（R1主任务台阶不是障碍）
            current_obstacles = all_obstacles.copy()
            for m in r1_primary_manuals:
                if m.step_index in current_obstacles:
                    current_obstacles.remove(m.step_index)

            # 状态转换逻辑
            if current_idx == -2:  # 当前在森林中
                # 移动到地面
                for step in self.steps:
                    adj_g = self.get_adjacent_ground(step)
                    if adj_g:
                        # 计算移动成本
                        move_cost = np.linalg.norm(np.array(current_pos) - np.array(adj_g)) / v_special
                        new_g = g + move_cost
                        h = heuristic_idx(adj_g, -1)
                        new_is_inside = False
                        ground_pos_key = tuple(np.round(adj_g, 2))
                        ground_state_key = (-1, ground_pos_key, new_is_inside, None, False, min_row)
                        if ground_state_key not in visited or new_g < visited[ground_state_key]['cost']:
                            visited[ground_state_key] = {'cost': new_g,
                                                         'parent': (current_pos, current_idx, is_inside, prev_axis,
                                                                    is_paused, min_row),
                                                         'pos': adj_g,
                                                         'min_row': min_row}
                            heapq.heappush(pq, (new_g + h, new_g, adj_g, -1, new_is_inside, None, False, min_row))

            elif current_idx != -1:  # 当前在台阶上
                current_step = self.logic_index_to_step[current_idx]
                # 移动到相邻台阶
                for neighbor_idx in current_step['neighbors']:
                    neighbor_row = self.logic_index_to_step[neighbor_idx]['row']
                    # 新增：严格不可回头——只允许走到等于或低于已到达的最小行数
                    # 取当前状态的min_row
                    if 'min_row' in locals():
                        cur_min_row = min_row
                    else:
                        cur_min_row = current_step['row']
                    if neighbor_row > cur_min_row:
                        continue
                    next_min_row = min(cur_min_row, neighbor_row)
                    # 检查邻居是否是障碍物
                    if neighbor_idx in all_obstacles:
                        is_primary_r1 = False
                        for m in r1_primary_manuals:
                            if m.step_index == neighbor_idx:
                                is_primary_r1 = True
                                break
                        if not is_primary_r1:
                            continue  # 是障碍物且不是R1主任务台阶，跳过

                    neighbor_step = self.logic_index_to_step[neighbor_idx]

                    # 检查高度差是否在R2能力范围内
                    dz = abs(neighbor_step['height'] - current_step['height'])
                    if dz > self.r2_max_step_height:
                        continue

                    # 计算爬升成本（只有在停顿状态下才计算）
                    climb_penalty = 0
                    if is_paused:
                        if np.isclose(dz, 0.2, atol=0.05):
                            climb_penalty = PARAMS['s8']
                        elif np.isclose(dz, 0.4, atol=0.05):
                            climb_penalty = PARAMS['s9']

                    # 如果没停顿且有高度差，则不能移动
                    if not is_paused and dz > 0.01:
                        continue

                    # 计算移动成本
                    move_cost = np.linalg.norm(np.array(current_pos) - np.array(neighbor_step['center'])) / v_normal

                    # 判断移动方向（仍保留axis用于状态键，不再用于惩罚判定）
                    dx = neighbor_step['center'][0] - current_pos[0]
                    dy = neighbor_step['center'][1] - current_pos[1]
                    axis = 'x' if abs(dx) >= abs(dy) else 'y'

                    # 计算转向惩罚（新规则）：向“同行”台阶移动时直接施加惩罚
                    turn_penalty = PARAMS.get('s7', 0) if neighbor_step['row'] == current_step['row'] else 0

                    # 行1列偏置：把“该列地面->斜坡时间”差额加到真实代价上，确保偏向更近列
                    extra_exit_bias = 0.0
                    if include_exit_future and neighbor_step['row'] == 1 and (min_row1_ground_time is not None):
                        t_col = row1_time_by_col.get(neighbor_step['col']) if 'row1_time_by_col' in locals() else None
                        if t_col is not None:
                            extra_exit_bias = exit_bias_g_weight * max(0.0, t_col - min_row1_ground_time)

                    # 总移动成本
                    new_g = g + move_cost + climb_penalty + turn_penalty + extra_exit_bias
                    h = heuristic_idx(neighbor_step['center'], neighbor_idx)
                    new_is_inside = True
                    neighbor_pos_key = tuple(np.round(neighbor_step['center'], 2))
                    neighbor_state_key = (neighbor_idx, neighbor_pos_key, new_is_inside, axis, False, next_min_row)

                    # 更新状态
                    if neighbor_state_key not in visited or new_g < visited[neighbor_state_key]['cost']:
                        visited[neighbor_state_key] = {'cost': new_g,
                                                       'parent': (current_pos, current_idx, is_inside, prev_axis,
                                                                  is_paused, cur_min_row),
                                                       'pos': neighbor_step['center']}
                        heapq.heappush(pq,
                                       (new_g + h, new_g, neighbor_step['center'], neighbor_idx, new_is_inside, axis,
                                        False, next_min_row))

                # 从台阶移动到地面（出口）
                # 注意：扩展到地面时也要传递min_row
                dz = current_step['height']
                climb_penalty = 0
                if is_paused or is_exit_path:
                    if dz <= self.r2_max_step_height:
                        if np.isclose(dz, 0.2, atol=0.05):
                            climb_penalty = PARAMS['s8']
                        elif np.isclose(dz, 0.4, atol=0.05):
                            climb_penalty = PARAMS['s9']
                    else:
                        continue  # 高度差超出能力范围

                # 如果没停顿且有高度差，则不能移动（除非是退出路径）
                if not is_paused and not is_exit_path and dz > 0.01:
                    continue

                # 获取台阶相邻的地面位置
                ground_pos = self.get_adjacent_ground(current_step)
                if ground_pos:
                    # 计算移动成本
                    move_cost = np.linalg.norm(np.array(current_pos) - np.array(ground_pos)) / v_special
                    # 对行1的“台阶->地面”也加入列偏置，避免两边无障碍时走向更远列
                    extra_exit_bias_ground = 0.0
                    if include_exit_future and current_step['row'] == 1 and (min_row1_ground_time is not None):
                        t_col = row1_time_by_col.get(current_step['col']) if 'row1_time_by_col' in locals() else None
                        if t_col is not None:
                            extra_exit_bias_ground = exit_bias_g_weight * max(0.0, t_col - min_row1_ground_time)
                            if PARAMS.get('debug_exit_bias'):
                                print(
                                    f"[偏置] 台阶->地面 行1列{current_step['col']} 额外代价 {extra_exit_bias_ground:.3f}")
                    new_g = g + move_cost + climb_penalty + extra_exit_bias_ground
                    h = heuristic_idx(ground_pos, -1)
                    new_is_inside = False
                    ground_pos_key = tuple(np.round(ground_pos, 2))
                    ground_state_key = (-1, ground_pos_key, new_is_inside, None, False, min_row)
                    if ground_state_key not in visited or new_g < visited[ground_state_key]['cost']:
                        visited[ground_state_key] = {'cost': new_g,
                                                     'parent': (current_pos, current_idx, is_inside, prev_axis,
                                                                is_paused, min_row),
                                                     'pos': ground_pos}
                        heapq.heappush(pq, (new_g + h, new_g, ground_pos, -1, new_is_inside, None, False, min_row))
            else:
                # 当前在地面（仅在出口路径中允许地面直达终点）
                if is_exit_path and goal_pos is not None:
                    move_cost = np.linalg.norm(np.array(current_pos) - np.array(goal_pos)) / v_special
                    new_g = g + move_cost
                    h = 0.0
                    new_is_inside = False
                    goal_pos_key = tuple(np.round(goal_pos, 2))
                    ground_state_key = (-1, goal_pos_key, new_is_inside, None, False, min_row)
                    if ground_state_key not in visited or new_g < visited[ground_state_key]['cost']:
                        visited[ground_state_key] = {'cost': new_g,
                                                     'parent': (current_pos, current_idx, is_inside, prev_axis,
                                                                is_paused, min_row),
                                                     'pos': goal_pos,
                                                     'min_row': min_row}
                        heapq.heappush(pq, (new_g + h, new_g, goal_pos, -1, new_is_inside, None, False, min_row))

        # 如果超过最大迭代次数或队列为空仍未找到路径，返回失败
        if iterations >= max_iterations or not pq:
            print(f"[A*调试] 未找到路径")
            if iterations >= max_iterations:
                print(f"[A*调试] 达到最大迭代次数{max_iterations}")
            else:
                print(f"[A*调试] 优先队列为空，提前终止")
            print(f"[A*调试] 已探索状态数: {len(visited)}")
            print(f"[A*调试] 最后处理的位置: {current_pos}")
        return None, float('inf'), None, -1

    def get_adjacent_ground(self, step):
        """获取台阶相邻的地面位置（只允许第1行台阶有地面出口）"""
        if step['row'] == 1:
            return (step['center'][0], 0.7)  # 只允许第1行台阶下方的地面
        return None

    def _find_path_to_manual(self, start_pos, start_time, goal_idx, r1_primary_manuals, manuals_to_ignore,
                             remaining_indices, preferred_row=None, allowed_pickup_indices=None):
        """寻找从当前位置到秘籍拾取点的路径（A*目标严格限定为目标台阶的邻居台阶）"""
        goal_step = self.logic_index_to_step[goal_idx]
        start_idx = self.find_step_by_pos(start_pos)
        start_row = self.logic_index_to_step[start_idx]['row'] if start_idx != -1 else 5  # 地面视为第5行

        # 只允许目标台阶的邻居台阶作为合法拾取点
        legal_pickup_indices = set(goal_step['neighbors']) - set(remaining_indices)
        # 若提供 allowed_pickup_indices，则进一步收缩集合
        if allowed_pickup_indices is not None:
            legal_pickup_indices = legal_pickup_indices.intersection(set(allowed_pickup_indices))
        # 若指定preferred_row，则进一步限定为该行的邻居
        if preferred_row is not None:
            legal_pickup_indices = {n for n in legal_pickup_indices if
                                    self.logic_index_to_step[n]['row'] == preferred_row}

        def is_pickup_spot(pos, idx):
            # 禁止在有未被拾取R2秘籍的台阶上拾取任何秘籍
            if idx in remaining_indices:
                return False
            # 只能在目标秘籍的邻居台阶上拾取，且该邻居台阶本身不能有未被拾取的R2秘籍
            if idx in legal_pickup_indices:
                # 不能在目标台阶本身拾取
                if idx == goal_idx:
                    return False
                # 邻居台阶本身不能有未被拾取的R2秘籍
                if idx in remaining_indices:
                    return False
                return True
            return False

        # 检查当前是否已经在可拾取位置，且起点不能是有R2秘籍的台阶
        if is_pickup_spot(start_pos, start_idx) and start_idx not in remaining_indices:
            pickup_finish_time = start_time + PARAMS['s6']
            path = [(start_pos, start_time), (start_pos, pickup_finish_time)]
            total_time = pickup_finish_time - start_time
            return path, total_time, {}, start_pos

        # 只允许A*终点为合法邻居台阶，且A*扩展时禁止经过未被拾取R2秘籍台阶（目标台阶本身除外）
        def strict_goal_func(pos, idx):
            # 禁止在有未被拾取R2秘籍的台阶上作为终点（包括目标台阶本身）
            if idx in remaining_indices or idx == goal_idx:
                return False
            return is_pickup_spot(pos, idx)

        def strict_is_valid_step(idx):
            # 严格禁止经过任何未被拾取的R2秘籍台阶（包括目标台阶本身），确保“不可穿越未拾取R2”
            if idx in remaining_indices or idx == goal_idx:
                return False
            return True

        # 包装A*，在扩展时加上严格判定
        orig_find_step_by_pos = self.find_step_by_pos

        def patched_find_step_by_pos(pos):
            idx = orig_find_step_by_pos(pos)
            if not strict_is_valid_step(idx):
                return -99999  # 返回非法索引，A*会跳过
            return idx

        self.find_step_by_pos, old_find = patched_find_step_by_pos, self.find_step_by_pos
        try:
            path_to_spot, time_to_spot, pickup_pos, pickup_idx = self._find_path_to_target(
                start_pos, start_time, r1_primary_manuals,
                manuals_to_ignore, goal_func=strict_goal_func, goal_pos=goal_step['center'],
                remaining_r2_indices=remaining_indices, include_exit_future=True, exit_future_weight=3.0,
                exit_bias_g_weight=0.0,
                goal_context_key=frozenset(legal_pickup_indices),
                preferred_row_key=preferred_row
            )
        finally:
            self.find_step_by_pos = old_find

        if path_to_spot is None or pickup_idx not in legal_pickup_indices:
            return None, float('inf'), None, start_pos

        # 重构路径与时间信息
        full_path = path_to_spot[1:] if len(path_to_spot) > 1 else []
        arrival_at_pickup_time = start_time + time_to_spot
        pickup_finish_time = arrival_at_pickup_time + PARAMS['s6']

        if not full_path or not np.allclose(full_path[-1][0], pickup_pos):
            full_path.append((pickup_pos, arrival_at_pickup_time))
        full_path.append((pickup_pos, pickup_finish_time))

        total_time = pickup_finish_time - start_time
        final_pos = pickup_pos

        return full_path, total_time, {}, final_pos

    def find_step_by_pos(self, pos):
        """根据位置查找对应的台阶索引（使用预构建的坐标映射，加速）。"""
        try:
            key = tuple(np.round(pos, 2))
            idx = self.pos_to_logic_index.get(key)
            if idx is not None:
                return idx
        except Exception:
            pass
        # 回退：精确比较，尽量少用
        for idx, step in self.logic_index_to_step.items():
            if np.allclose(step['center'], pos, atol=1e-2):
                return idx
        return -1  # 未找到对应台阶

    def _find_path_to_exit(self, start_pos, start_time, r1_primary_manuals, manuals_to_ignore,
                           remaining_r2_indices=None):
        """寻找从当前位置到终点（斜坡）的路径"""
        if remaining_r2_indices is None:
            remaining_r2_indices = set()

        # 目标是到达斜坡位置
        goal_pos = np.array(PARAMS['ramp_pos'])

        # 定义目标函数：当位置接近 ramp_pos 时返回 True
        def is_at_ramp(pos, idx):
            ramp_pos = np.array(PARAMS['ramp_pos'])
            return np.allclose(pos, ramp_pos, atol=0.1)

        # 使用 A* 算法直接寻找通往终点的路径
        path, time, end_pos, end_idx = self._find_path_to_target(
            start_pos,
            start_time,
            r1_primary_manuals,
            manuals_to_ignore,
            goal_func=is_at_ramp,
            goal_pos=goal_pos,
            remaining_r2_indices=remaining_r2_indices,
            is_exit_path=True,  # 标记这是出口路径
            include_exit_future=True,
            exit_future_weight=3.0,
            exit_bias_g_weight=1.0
        )

        return path, time, end_pos


# --- 5. 主模拟类 (执行器) ---
class Simulation:
    """模拟器类，负责场景可视化和动画生成"""

    def __init__(self, steps, manuals, r2_max_step_height, run_planner=True):
        self.fig = None  # 图形对象
        self.ax = None  # 坐标轴对象
        self.time_text = None  # 时间文本
        self.entities = {}  # 场景中的实体
        # 保守复制：避免上层已深拷贝时再次深拷贝；若外部引用可能复用则在 Planner 中不会回写
        self.steps = steps if isinstance(steps, list) else copy.deepcopy(steps)
        self.manuals = manuals if isinstance(manuals, list) else copy.deepcopy(manuals)
        self.r2_max_step_height = r2_max_step_height  # R2最大越障高度
        self.plan = None  # 规划结果
        if run_planner and self.steps:
            # 运行规划器生成计划
            self.plan = Planner(self.steps, self.manuals, self.r2_max_step_height).plan()

    def setup_figure(self):
        """设置图形界面"""
        self.fig, self.ax = plt.subplots(figsize=(10, 13))
        self.setup_map_base()  # 设置地图基础元素
        self.setup_scenario_entities()  # 设置场景实体

    def setup_map_base(self):
        """设置地图基础元素"""
        self.ax.set_aspect('equal')
        theme = get_theme(MAP_VARIANT)
        self.ax.set_title(f"R2能力: {self.r2_max_step_height}m{theme.get('title_suffix', '')}")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.ax.set_xlim(0, 7)  # 调整x轴范围（左右对称，覆盖镜像对象）
        self.ax.set_ylim(-1.5, 10)  # 调整y轴范围

        # 绘制区域背景
        self.ax.add_patch(
            patches.Rectangle((0, 7.3), 6, 2, facecolor=theme['background_top'], zorder=0, alpha=0.5))  # 上方区域
        self.ax.add_patch(
            patches.Rectangle((0, 0), 6, 7.3, facecolor=theme['background_bottom'], zorder=0, alpha=0.5))  # 下方区域

        # 绘制网格
        grid_size = 1.2
        # 绘制 5x6 网格线 (Row 0..5, Col 0..4)
        # 物理范围：x: 0.6 ~ 0.6+4*1.2=5.4; y: 0.8 ~ 0.8+5*1.2=6.8
        # 扩展一点显示范围
        for r in range(6):
            for c in range(5):
                # 判断是否为台阶区 (Row 1..4, Col 1..3)
                is_step_area = (r >= 1 and r <= 4) and (c >= 1 and c <= 3)

                # 计算格子左下角
                gx = 0.6 + c * 1.2 - 0.6  # 格子中心是 0.6+c*1.2，左下角减半宽
                gy = 0.8 + r * 1.2 - 0.6

                # 仅绘制外围可行走区域的网格
                if not is_step_area:
                    rect = patches.Rectangle((gx, gy), 1.2, 1.2,
                                             facecolor='#E0E0E0', edgecolor='white',
                                             linestyle=':', zorder=-1)
                    self.ax.add_patch(rect)
                    # 可选：显示网格坐标
                    # self.ax.text(gx+0.6, gy+0.6, f"{r},{c}", ha='center', va='center', 
                    #              fontsize=8, color='gray', alpha=0.5)

        # 添加时间文本
        self.time_text = self.ax.text(6.5, 9.5, 'Time: 0.0s', ha='right', va='top', fontsize=12)

    def setup_scenario_entities(self):
        """设置场景中的实体（机器人、支撑点、长矛、台阶、秘籍等）"""
        # 绘制台阶（排除伪台阶）
        step_colors = {0.2: 'darkgreen', 0.4: "#66B01B", 0.6: 'yellowgreen'}
        for step in (s for s in self.steps if not s.get('is_pseudo')):
            color = step_colors.get(step['height'], 'gray')
            rect = patches.Rectangle(
                (step['center'][0] - 0.6, step['center'][1] - 0.6), 1.2, 1.2,
                facecolor=color, edgecolor='black', linewidth=1.5, zorder=2, alpha=0.7
            )
            self.ax.add_patch(rect)
            # 台阶编号
            self.ax.text(step['center'][0], step['center'][1], f"{step['logic_index']}", ha='center', va='center',
                         fontsize=10, color='white', zorder=3)

        # 绘制支撑点
        support = CircleEntity(PARAMS['support_pos'], 0.18, 'purple', '支撑点')
        support.draw(self.ax)
        self.entities['support'] = support

        # 绘制长矛
        spear = RectangleEntity(PARAMS['spear_pos'], 0.3, 0.08, 'gold', '长矛')
        spear.draw(self.ax)
        self.entities['spear'] = spear

        # 绘制秘籍（只add一次patch，且只为未被拾取的秘籍draw）
        for manual in self.manuals:
            if not manual.is_picked_up:
                manual.draw(self.ax)

        # 创建并注册R1和R2机器人
        theme = get_theme(MAP_VARIANT)
        r1 = Robot('R1', PARAMS['r1_start_pos'], 0.25, theme['accent'])
        r2 = Robot('R2', PARAMS['r2_start_pos'], 0.25, 'orange')
        self.entities['r1'] = r1
        self.entities['r2'] = r2
        # 绘制机器人
        if self.ax is not None:
            r1.draw(self.ax)
            r2.draw(self.ax)

    step_colors = {0.2: 'darkgreen', 0.4: "#66B01B", 0.6: 'yellowgreen'}


def render_base_map(ax, steps, manuals, title="地图布局"):
    """在给定的Axes上绘制底图、台阶与秘籍。"""
    ax.set_aspect('equal')
    theme = get_theme(MAP_VARIANT)
    ax.set_title(title + theme.get('title_suffix', ''))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_xlim(0, 7)
    ax.set_ylim(-1.5, 10)

    # 区域底色
    ax.add_patch(patches.Rectangle((0, 7.3), 6, 2, facecolor=theme['background_top'], zorder=0, alpha=0.5))
    ax.add_patch(patches.Rectangle((0, 0), 6, 7.3, facecolor=theme['background_bottom'], zorder=0, alpha=0.5))

    # 背景网格
    grid_size = 1.2
    for i in range(6):
        for j in range(7):
            is_step_area = (i >= 1 and i <= 3) and (j >= 1 and j <= 4)
            if not is_step_area:
                ax.add_patch(patches.Rectangle((i * grid_size, (j - 1) * grid_size), grid_size, grid_size,
                                               facecolor='#E0E0E0', edgecolor='white', zorder=-1))

    # 台阶（排除伪台阶）
    step_colors = {0.2: 'darkgreen', 0.4: "#66B01B", 0.6: 'yellowgreen'}
    for step in (s for s in steps if not s.get('is_pseudo')):
        color = step_colors.get(step['height'], 'gray')
        rect = patches.Rectangle(
            (step['center'][0] - 0.6, step['center'][1] - 0.6), 1.2, 1.2,
            facecolor=color, edgecolor='black', linewidth=1.5, zorder=2, alpha=0.7
        )
        ax.add_patch(rect)
        ax.text(step['center'][0], step['center'][1], f"{step['logic_index']}",
                ha='center', va='center', fontsize=10, color='white', zorder=3)

    # 秘籍
    for manual in manuals:
        manual.draw(ax)


def draw_map(steps, manuals, filename=None):
    """绘制地图并保存或显示（不含路径）。"""
    fig, ax = plt.subplots(figsize=(10, 13))
    render_base_map(ax, steps, manuals, title="地图布局")
    if filename:
        plt.savefig(filename)
    else:
        plt.show()
    plt.close(fig)
    print("地图绘图功能运行成功")
    if filename:
        print(f"地图已保存为: {filename}")


def draw_map_with_r2_path(steps, manuals, r2_path_points, filename):
    """绘制地图并叠加R2路径（红色折线）"""
    fig, ax = plt.subplots(figsize=(10, 13))
    render_base_map(ax, steps, manuals, title="地图布局 + R2路径")

    # R2路径（红色）
    if r2_path_points and len(r2_path_points) >= 2:
        theme = get_theme(MAP_VARIANT)
        xs = [p[0] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r2_path_points]
        ys = [p[1] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r2_path_points]
        ax.plot(xs, ys, color=theme['accent'], linewidth=2.5, zorder=15, label='R2路径')
        # 起点/终点标记
        ax.scatter([xs[0]], [ys[0]], color=theme['accent_alt'], s=40, zorder=16)
        ax.scatter([xs[-1]], [ys[-1]], color=theme['accent_alt'], s=40, zorder=16)
        ax.legend(loc='lower right')
    else:
        print("[提示] R2路径为空，地图仅绘制底图与实体。")

    plt.savefig(filename)
    plt.close(fig)
    print(f"地图已保存为: {filename}")


def draw_map_with_paths(steps, manuals, r2_path_points, r1_path_points, filename, title_suffix: str | None = None,
                        show_r1: bool = True):
    """绘制地图并叠加R2与R1路径。
    - R2: 橙色
    - R1: 主题强调色（red/blue），可通过 show_r1=False 关闭绘制
    """
    fig, ax = plt.subplots(figsize=(10, 13))
    title = "地图布局 + R1/R2路径"
    if title_suffix:
        title += f" - {title_suffix}"
    render_base_map(ax, steps, manuals, title=title)

    theme = get_theme(MAP_VARIANT)

    # R2路径（橙色）
    if r2_path_points and len(r2_path_points) >= 2:
        xs = [p[0] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r2_path_points]
        ys = [p[1] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r2_path_points]
        ax.plot(xs, ys, color='orange', linewidth=2.5, zorder=15, label='R2路径')
        ax.scatter([xs[0]], [ys[0]], color='orange', s=40, zorder=16)
        ax.scatter([xs[-1]], [ys[-1]], color='orange', s=40, zorder=16)
    else:
        print("[提示] R2路径为空，地图仅绘制底图与实体。")

    # R1路径（主题强调色，可开关）
    if show_r1:
        if r1_path_points and len(r1_path_points) >= 2:
            xs1 = [p[0] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r1_path_points]
            ys1 = [p[1] if isinstance(p, (list, tuple, np.ndarray)) else p for p in r1_path_points]
            ax.plot(xs1, ys1, color=theme['accent'], linewidth=2.0, linestyle='--', zorder=14, label='R1路径')
            ax.scatter([xs1[0]], [ys1[0]], color=theme['accent_alt'], s=30, zorder=16)
            ax.scatter([xs1[-1]], [ys1[-1]], color=theme['accent_alt'], s=30, zorder=16)
        else:
            print("[提示] R1路径为空。")

    ax.legend(loc='lower right')
    plt.savefig(filename)
    plt.close(fig)
    print(f"地图已保存为: {filename}")


def create_base_scenario(seed, map_variant: str = None):
    """创建基础场景（台阶和随机放置的秘籍）"""
    random.seed(seed)
    np.random.seed(seed)
    steps, manuals = [], []
    mv = (map_variant or MAP_VARIANT).lower()
    step_size = 1.2
    # 预设台阶高度分布（4行3列）
    heights = [[0.2, 0.4, 0.2], [0.4, 0.6, 0.4], [0.2, 0.4, 0.6], [0.4, 0.2, 0.4]]

    # 创建台阶
    for row_idx in range(3, -1, -1):  # 从第4行到第1行
        for col_idx in range(3):  # 列1到3
            i = row_idx  # heights[3] 是顶部
            j = col_idx  # 高度仍按原列索引映射
            # 根据地图变体决定物理列用于 X 坐标（blue 做左右镜像）
            j_physical = (2 - col_idx) if mv == 'blue' else col_idx
            # 新坐标公式：中心=单元格起点 + (台阶中心到边框距离0.6)
            # 使用0基索引：列 j_physical ∈ {0,1,2}，行 i ∈ {0,1,2,3}
            # red: x = 0.6 + step_size * (j_physical + 1)
            #      y = 0.8 + step_size * (i + 1)   （等价于 2.0 + 1.2*i）
            x = 0.6 + step_size * (j_physical + 1)
            y = 0.8 + step_size * (i + 1)
            # 1-based逻辑索引（用户可见）。
            # red: 左上=10，右上=12；左下=1，右下=3（原有）。
            # blue: 进行左右镜像：右下=1，左上=12。
            if (mv == 'blue'):
                # 修正：blue 编号按“左上=1，右下=12”（与 red 一致）；仅物理坐标做左右镜像
                logic_index = (3 - row_idx) * 3 + col_idx + 1
            else:
                logic_index = (3 - row_idx) * 3 + col_idx + 1
            steps.append({
                'logic_index': logic_index,  # 明确命名为逻辑索引
                'list_index': len(steps),  # 0-based列表索引（内部用）
                'center': (x, y),
                'row': row_idx + 1,
                'col': col_idx + 1,
                'height': heights[i][j],
                'manual': None,
                'neighbors': []  # 存储邻居的逻辑索引（而非列表索引）
            })

    # 设置台阶邻居关系
    for step in steps:
        step['neighbors'] = []
        for other in steps:
            if other['logic_index'] == step['logic_index']:
                continue
            # 严格判断相邻：行差1且列相同，或列差1且行相同
            if (abs(step['row'] - other['row']) == 1 and step['col'] == other['col']) or \
                    (abs(step['col'] - other['col']) == 1 and step['row'] == other['row']):
                step['neighbors'].append(other['logic_index'])

    # 取消手动交换台阶高度的调整：保持原始高度布局，不再进行第3行的高度互换

    # 放置假秘籍（排除入口行）——统一改为 1-based 逻辑编号集合
    all_logic_indices = {s['logic_index'] for s in steps}
    non_entry_logic_indices = {s['logic_index'] for s in steps if s['row'] != 4}
    fake_manual_logic = random.choice(sorted(list(non_entry_logic_indices)))

    # 放置R1秘籍
    available_for_r1_logic = {s['logic_index'] for s in steps if
                              (s['col'] != 2) or (s['row'] in [1, 4] and s['col'] == 2)} - {fake_manual_logic}
    if len(available_for_r1_logic) < PARAMS['r1_manual_total']:
        print("R1秘籍可用台阶数量不足")
        return None, None
    r1_manual_logic = random.sample(sorted(list(available_for_r1_logic)), PARAMS['r1_manual_total'])

    # 放置R2秘籍（不再强制入口行至少一个）
    available_for_r2_logic = all_logic_indices - {fake_manual_logic} - set(r1_manual_logic)
    if len(available_for_r2_logic) < PARAMS['r2_manual_total']:
        print("R2秘籍可用台阶数量不足")
        return None, None
    r2_manual_logic = random.sample(sorted(list(available_for_r2_logic)), PARAMS['r2_manual_total'])

    # 创建秘籍实体
    manual_size = 0.35
    for logic_idx in r1_manual_logic:
        step = next(s for s in steps if s['logic_index'] == logic_idx)
        manual = RectangleEntity(step['center'], manual_size, manual_size, 'darkblue', 'R1')
        manuals.append(manual)
        step['manual'] = manual
    for logic_idx in r2_manual_logic:
        step = next(s for s in steps if s['logic_index'] == logic_idx)
        manual = RectangleEntity(step['center'], manual_size, manual_size, 'skyblue', 'R2')
        manuals.append(manual)
        step['manual'] = manual
    step = next(s for s in steps if s['logic_index'] == fake_manual_logic)
    manual = RectangleEntity(step['center'], manual_size, manual_size, 'black', '假')
    manuals.append(manual)
    step['manual'] = manual

    # 设置秘籍所在台阶索引
    for step in steps:
        if step['manual']:
            step['manual'].step_index = step['logic_index']  # 明确使用逻辑索引

    print(f"--- 基础场景生成完毕 (种子: {seed}) ---")
    print("地图生成功能运行成功")
    return steps, manuals


def create_manual_scenario(steps, manual_indices):
    """手动放置秘籍：接受 1-based 逻辑编号字典，比如 {'R1':[1,2,7], 'R2':[4,3,6,12], 'fake':9}。"""
    manuals = []
    used_logic = set()

    manual_size = 0.35
    colors = {'R1': 'darkblue', 'R2': 'skyblue', 'fake': 'black'}
    labels = {'R1': 'R1', 'R2': 'R2', 'fake': '假'}

    logic_index_set = {s['logic_index'] for s in steps}

    # 关键对齐：先清空所有台阶上的残留手动/随机摆放，避免影响伪台阶与入口判断
    for s in steps:
        s['manual'] = None

    for key, indices in manual_indices.items():
        if key not in colors:
            print(f"错误：未知秘籍类型 '{key}'")
            return None
        if key == 'fake':
            indices = [indices]  # 假秘籍只有一个编号
        # 数量校验
        expected_count = PARAMS['r1_manual_total'] if key == 'R1' else PARAMS['r2_manual_total'] if key == 'R2' else 1
        if len(indices) != expected_count:
            print(f"错误：{key} 秘籍数量不匹配（预期 {expected_count} 个）")
            return None

        for logic_idx in indices:
            if not isinstance(logic_idx, int) or logic_idx not in logic_index_set:
                print(f"错误：逻辑编号 {logic_idx} 不在有效集合 {sorted(logic_index_set)} 内")
                return None
            if logic_idx in used_logic:
                print(f"错误：逻辑编号 {logic_idx} 被重复使用")
                return None
            used_logic.add(logic_idx)

            # 匹配台阶并创建秘籍
            step = next(s for s in steps if s['logic_index'] == logic_idx)
            manual = RectangleEntity(step['center'], manual_size, manual_size, colors[key], labels[key])
            manual.step_index = step['logic_index']
            manuals.append(manual)
            step['manual'] = manual

    print("--- 手动场景生成完毕 ---")
    return manuals


# --- 5. 遍历所有合法放置与编码输出 ---
def build_manuals_from_code(steps, code12):
    """根据12位编码生成对应的秘籍放置。
    规则：1=R1, 2=R2, 3=假, 4=空；必须恰好有 PARAMS['r1_manual_total'] 个1，PARAMS['r2_manual_total'] 个2，且恰好一个3。
    code12: 长度12的可迭代（字符串或列表/元组），按逻辑索引1..12对应位。
    返回：manuals 列表；若非法返回 None。
    """
    if isinstance(code12, str):
        if len(code12) != 12 or any(c not in '1234' for c in code12):
            return None
        digits = [int(c) for c in code12]
    else:
        if len(code12) != 12 or any(d not in (1, 2, 3, 4) for d in code12):
            return None
        digits = list(code12)

    if digits.count(1) != PARAMS['r1_manual_total']:
        return None
    if digits.count(2) != PARAMS['r2_manual_total']:
        return None
    if digits.count(3) != 1:
        return None

    manual_indices = {'R1': [], 'R2': [], 'fake': None}
    for i, d in enumerate(digits, start=1):
        if d == 1:
            manual_indices['R1'].append(i)
        elif d == 2:
            manual_indices['R2'].append(i)
        elif d == 3:
            manual_indices['fake'] = i

    if manual_indices['fake'] is None:
        return None

    manuals = create_manual_scenario(steps, manual_indices)
    return manuals


def encode_placement_and_path(steps, manuals, best_plan):
    """生成 12 位放置编码与带拾取标注的路径字符串。
    返回 (code12_str, annotated_path_str)；若无法生成路径则返回 (code12_str, '').

    标注规则：
    - 仍输出去重后的 R2 经过台阶逻辑序列；
    - 若在某个台阶完成了 R2 秘籍拾取，则在该台阶编号后追加括号，括号内为本次（或同一步的多次）拾取的“秘籍所在台阶编号”；
      例如：3(6) 6(5 9) 9 12；表示在台阶 3 拾取了位于台阶 6 的秘籍；在台阶 6 连续拾取了位于台阶 5 与 9 的秘籍；
    - 同一拾取台阶出现多本时，括号内排序遵循“同行优先、低行靠后”（先与拾取台阶同一行的秘籍，随后按行号从高到低）。
    """
    code = ['4'] * 12
    for m in manuals:
        idx = m.step_index  # 1-based
        if m.label == 'R1':
            code[idx - 1] = '1'
        elif m.label == 'R2':
            code[idx - 1] = '2'
        elif m.label == '假':
            code[idx - 1] = '3'
    code12 = ''.join(code)

    path_str = ''
    if best_plan and best_plan.get('r2_path'):
        # 1) 建立位置到台阶信息的映射（包含伪台阶）
        # key: (x, y) -> {'index': logic_index, 'is_pseudo': bool, 'row': row, 'col': col}
        pos_to_info = {}
        for s in steps:
            key = tuple(np.round(s['center'], 2))
            pos_to_info[key] = {
                'index': s['logic_index'],
                'is_pseudo': s.get('is_pseudo', False),
                'row': s.get('row'),
                'col': s.get('col'),
                'center': s['center']
            }
        
        # 2) 生成路径序列（区分伪台阶与普通台阶）
        # 序列元素为 (logic_index, is_pseudo)
        raw_seq = []
        for p in best_plan['r2_path']:
            key = tuple(np.round(p, 2))
            info = pos_to_info.get(key)
            if info is None:
                # 几何回退匹配
                for s in steps:
                    if np.allclose(s['center'], p, atol=1e-2):
                        info = {
                            'index': s['logic_index'],
                            'is_pseudo': s.get('is_pseudo', False),
                            'row': s.get('row'),
                            'col': s.get('col'),
                            'center': s['center']
                        }
                        break
            
            if info and 1 <= info['index'] <= 12:
                raw_seq.append((info['index'], info['is_pseudo']))
        
        # 去重（保留顺序，区分伪台阶与真台阶）
        seen = set()
        ordered = []
        for item in raw_seq:
            if item not in seen:
                seen.add(item)
                ordered.append(item)

        # 3) 建立“拾取事件 -> (台阶索引, 是否伪台阶)”的映射
        picks_by_loc: dict[tuple[int, bool], list[int]] = {}
        try:
            r2_ts = list(best_plan.get('r2_timestamps') or [])
            r2_path = list(best_plan.get('r2_path') or [])

            def _time_to_loc_key(t: float) -> tuple[int, bool] | None:
                if not r2_ts:
                    return None
                i = None
                for k in range(len(r2_ts) - 1, -1, -1):
                    if float(r2_ts[k]) <= float(t) + 1e-6:
                        i = k
                        break
                if i is None:
                    i = 0
                
                # 搜索最近的台阶点
                for off in range(0, 4):
                    for sgn in (-1, 1):
                        j = i + sgn * off
                        if j < 0 or j >= len(r2_path):
                            continue
                        key = tuple(np.round(r2_path[j], 2))
                        info = pos_to_info.get(key)
                        if info:
                            return (info['index'], info['is_pseudo'])
                        # 几何回退
                        for s in steps:
                            if np.allclose(s['center'], r2_path[j], atol=1e-2):
                                return (s['logic_index'], s.get('is_pseudo', False))
                return None

            r2_pick_sched = best_plan.get('r2_pickup_schedule') or {}
            if r2_pick_sched:
                items = sorted(
                    [(getattr(m, 'step_index', None), float(t)) for m, t in r2_pick_sched.items() if
                     hasattr(m, 'step_index')],
                    key=lambda x: x[1]
                )
                for manual_idx, t in items:
                    loc_key = _time_to_loc_key(t)
                    if loc_key:
                        picks_by_loc.setdefault(loc_key, []).append(int(manual_idx))

                # 排序：同行优先、低行靠后
                # 使用所有台阶（含伪台阶）建立索引映射
                idx_to_step_info = {s['logic_index']: s for s in steps}
                
                for (step_idx, is_pseudo), m_list in picks_by_loc.items():
                    # 获取当前所在台阶的行（用于判断同行）
                    base_info = idx_to_step_info.get(step_idx, {})
                    pick_row = base_info.get('row')

                    def sort_key(mid: int):
                        st = idx_to_step_info.get(mid, {})
                        r = st.get('row')
                        c = st.get('col')
                        return (0 if (pick_row is not None and r == pick_row) else 1, -(r if r is not None else 0),
                                (c if c is not None else 0))

                    m_list.sort(key=sort_key)
        except Exception:
            picks_by_loc = {}

        # 4) 生成 Token 字符串
        tokens = []
        for (step_idx, is_pseudo) in ordered:
            pick_list = picks_by_loc.get((step_idx, is_pseudo))
            
            if is_pseudo:
                # 伪台阶逻辑：
                # 1. 若有拾取，按顺序输出 (m1) (m2) ...
                # 2. 若无拾取，不输出任何内容（隐式经过）
                if pick_list:
                    for m in pick_list:
                        tokens.append(f"({m})")
            else:
                # 真台阶逻辑：
                # 输出 step_idx
                # 若有拾取，追加 (m1 m2 ...)
                if pick_list:
                    inner = ' '.join(str(x) for x in pick_list)
                    tokens.append(f"{step_idx}({inner})")
                else:
                    tokens.append(str(step_idx))
        
        path_str = ' '.join(tokens)

        # 5) 追加 R1 拾取信息
        # 格式：0 <R1拾取台阶1> <R1拾取台阶2> ...
        r1_pick_sched = best_plan.get('r1_pickup_schedule') or {}
        if r1_pick_sched:
            # 按时间排序
            r1_items = sorted(
                [(getattr(m, 'step_index', None), float(t)) for m, t in r1_pick_sched.items() if
                 hasattr(m, 'step_index')],
                key=lambda x: x[1]
            )
            r1_tokens = [str(int(idx)) for idx, _ in r1_items if idx is not None]
            if r1_tokens:
                path_str += f" 0 {' '.join(r1_tokens)}"

    return code12, path_str


def _is_placement_legal(steps, manual_indices, step_meta: dict | None = None):
    """根据项目规则判断放置是否合法（不运行规划，仅做静态筛查）。
    规则口径（可按需扩展）：
    - 假秘籍不在入口行（row = 4）
    - R1 不能放在中列（col = 2），但行 1 和行 4 的中列允许（与 create_base_scenario 保持一致）
    """
    # 使用预计算的 step_meta 加速（避免每次都构造映射）
    if step_meta is not None:
        rows = step_meta['rows']  # 1-based 索引 -> row
        cols = step_meta['cols']  # 1-based 索引 -> col
        valid_idx = step_meta['valid']

        def _row(i):
            return rows[i]

        def _col(i):
            return cols[i]

        def _is_valid(i):
            return i in valid_idx
    else:
        logic_to_step = {s['logic_index']: s for s in steps if not s.get('is_pseudo')}

        def _row(i):
            return logic_to_step[i]['row']

        def _col(i):
            return logic_to_step[i]['col']

        def _is_valid(i):
            return i in logic_to_step
    # 1) 假秘籍不在入口行
    fake_idx = manual_indices.get('fake')
    if not isinstance(fake_idx, int) or not _is_valid(fake_idx):
        return False
    if _row(fake_idx) == 4:
        return False
    # 2) R1 中列限制（除行1与行4外，col=2 不允许）
    r1_list = manual_indices.get('R1') or []
    if not r1_list or any((not isinstance(i, int) or not _is_valid(i)) for i in r1_list):
        return False
    for i in r1_list:
        if _col(i) == 2 and _row(i) not in (1, 4):
            return False
    # 通过静态规则
    return True


def enumerate_all_placements_and_write(seed, r2_capabilities=(0.4,), out_file=None, show_progress=False,
                                       flush_every=200, max_tasks=None, processes=8, map_variant: str = None):
    """枚举所有“合法放置”，求每个放置在指定 R2 能力下的最优路径，输出到 txt。
    合法口径：当前实现为“假不在入口行、R1 中列限制（除行1/4外禁 col=2）”。如需加严/放宽请说明。
    输出格式（默认仅 0.4m 能力）：每行 "<12位编码> <路径序列>"；若无可行路径则仅输出 "<12位编码>"。
    在此函数内抑制地图绘制与调试输出。
    """
    # 仅生成台阶，且抑制其内部打印
    with suppress_stdout():
        steps, _ = create_base_scenario(seed, map_variant=map_variant)
    if not steps:
        return

    # 预计算 rows/cols/valid 加速 legality 检查
    base_steps = [s for s in steps if not s.get('is_pseudo')]
    rows = {s['logic_index']: s['row'] for s in base_steps}
    cols = {s['logic_index']: s['col'] for s in base_steps}
    valid_idx = set(rows.keys())
    step_meta = {'rows': rows, 'cols': cols, 'valid': valid_idx}

    # 所有 12 位位点：选择 R1 的位3个、R2 的位4个、假 的位1个，余下为空
    all_indices = tuple(range(1, 13))
    from itertools import combinations
    # 先剪枝 R1：排除中列(2)但行1/4除外
    def r1_combo_legal(r1_combo):
        for i in r1_combo:
            c = cols[i]
            r = rows[i]
            if (c == 2) and (r not in (1, 4)):
                return False
        return True

    r1_slots = [c for c in combinations(all_indices, PARAMS['r1_manual_total']) if r1_combo_legal(c)]

    # 构建“合法放置”任务列表（仅存放 manual_indices），并统计总数
    tasks = []
    # 预先筛出可作为假秘籍的位置（非入口行）
    fake_candidates = tuple(i for i in all_indices if rows[i] != 4)
    for r1_pos in r1_slots:
        remain_after_r1 = [i for i in all_indices if i not in r1_pos]
        for r2_pos in combinations(remain_after_r1, PARAMS['r2_manual_total']):
            remain_after_r2 = [i for i in remain_after_r1 if i not in r2_pos]
            # 假秘籍只在 fake_candidates 与剩余位置的交集里选
            for fake_idx in (i for i in remain_after_r2 if i in fake_candidates):
                manual_indices = {'R1': list(r1_pos), 'R2': list(r2_pos), 'fake': fake_idx}
                # 快速合法性判断（使用预计算元数据）
                if _is_placement_legal(steps, manual_indices, step_meta=step_meta):
                    tasks.append(manual_indices)
    # 可选：限制前 max_tasks 个任务用于小样本验证
    if max_tasks is not None and max_tasks > 0:
        tasks = tasks[:int(max_tasks)]
    total_placements = len(tasks)

    # 采用流式写入，避免在内存中累积所有结果
    lines_written = 0
    pbar = tqdm(total=total_placements, desc='枚举放置方案', unit='placement', leave=False,
                disable=not show_progress) if show_progress else None
    f = None
    try:
        if not out_file:
            mv = (map_variant or MAP_VARIANT).lower()
            out_file = f"placements_and_paths_{mv}.txt"
        f = open(out_file, 'w', encoding='utf-8')
        # 并行计算：仅主进程维护一个进度条与文件写入，子进程只返回结果
        from multiprocessing import Pool

        # 启动进程池（Windows 下需在 __main__ 环境调用本函数）
        if not processes or processes <= 0:
            try:
                processes = max(1, min(8, os.cpu_count() or 8))
            except Exception:
                processes = 8
        with Pool(processes=processes, initializer=_mp_enum_init, initargs=(steps, r2_capabilities)) as pool:
            # 使用较小的 chunksize 以提升进度条响应度（性能与体验折中）
            base = len(tasks) // (processes * 64) if processes else 1
            chunk = max(8, min(64, base if base > 0 else 1))
            for lines in pool.imap_unordered(_mp_enum_worker, tasks, chunksize=chunk):
                if lines:
                    for line in lines:
                        f.write(line + '\n')
                        lines_written += 1
                        if flush_every and (lines_written % flush_every == 0):
                            try:
                                f.flush()
                            except Exception:
                                pass
    finally:
        if pbar:
            try:
                pbar.close()
            except Exception:
                pass
        try:
            if f:
                f.flush()
                f.close()
        except Exception:
            pass
    if show_progress:
        print(f"已写出 {lines_written} 行结果到 {out_file}")


# === 验证并行支持（Windows 需顶层定义） ===
_VAL_BASE_STEPS = None


def _validate_init(steps):
    """子进程初始化：共享基础台阶，并开启静默模式。"""
    global _VAL_BASE_STEPS
    _VAL_BASE_STEPS = steps
    try:
        PARAMS['quiet'] = True
    except Exception:
        pass


def _validate_job_worker(args):
    """子进程工作：对一个(摆放,能力)实例计算是否需要等待，并返回摘要。
    返回: (feasible:bool, wait_required:bool, total_time:float|None, summary:dict|None)
    summary 包含：cap、manual_indices、r1/r2拾取、r2事件（climb/turn）、等待点。
    """
    # 支持两种参数形态：
    # 1) (manual_indices, cap)
    # 2) (manual_indices, cap, enable_block, r1_radius, r2_radius)
    if isinstance(args, tuple):
        manual_indices = args[0]
        cap = args[1] if len(args) > 1 else 0.4
        enable_block = bool(args[2]) if len(args) > 2 else False
        r1_radius = float(args[3]) if len(args) > 3 and args[3] is not None else float(PARAMS.get('r1_radius', 0.5))
        r2_radius = float(args[4]) if len(args) > 4 and args[4] is not None else float(PARAMS.get('r2_radius', 0.4))
    else:
        manual_indices, cap = args, 0.4
        enable_block = False
        r1_radius = float(PARAMS.get('r1_radius', 0.5))
        r2_radius = float(PARAMS.get('r2_radius', 0.4))
    try:
        steps_copy = _VAL_BASE_STEPS  # 只读基础台阶
        with suppress_stdout():
            manuals = create_manual_scenario(steps_copy, manual_indices)
        if not manuals:
            return (False, False, None, None)
        with suppress_stdout():
            sim = Simulation(copy.deepcopy(steps_copy), manuals, cap, run_planner=True)
        plan = sim.plan
        if not plan:
            return (False, False, None, None)
        # 基础台阶映射
        base_steps = [s for s in steps_copy if not s.get('is_pseudo')]
        pos_to_logic_index = {tuple(np.round(s['center'], 2)): s['logic_index'] for s in base_steps}
        logic_to_step = {s['logic_index']: s for s in base_steps}
        # R1拾取时刻
        r1_pick = {int(m.step_index): float(t) for m, t in (plan.get('r1_pickup_schedule') or {}).items() if
                   hasattr(m, 'step_index')}
        # R1 拾取停留事件：通过相邻同坐标点识别 (t_start, t_end)
        r1_path = plan.get('r1_path') or []
        r1_ts = plan.get('r1_timestamps') or []
        r1_stops = []  # [{'pos': (x,y), 't_start': t0, 't_end': t1}]
        if r1_path and r1_ts and len(r1_path) == len(r1_ts):
            for i in range(1, len(r1_path)):
                p0, p1 = r1_path[i - 1], r1_path[i]
                if np.allclose(p0, p1, atol=1e-6):
                    t0, t1 = float(r1_ts[i - 1]), float(r1_ts[i])
                    if t1 > t0:
                        r1_stops.append({'pos': tuple(np.round(p1, 6)), 't_start': t0, 't_end': t1})
        # R2首次到达时刻（原始时间线）
        r2_visit_time = {}
        r2_path = plan.get('r2_path') or []
        r2_ts = plan.get('r2_timestamps') or []
        for pos, t in zip(r2_path, r2_ts):
            key = tuple(np.round(pos, 2))
            idx = pos_to_logic_index.get(key)
            if idx is not None and idx not in r2_visit_time:
                r2_visit_time[idx] = float(t)
        # 若启用阻塞检查：基于 R1 停留和半径重放 R2 时间线
        r2_visit_time_blocked = None
        r2_delay_events = []
        if enable_block and r2_path and r2_ts and r1_stops:
            # 仅在“台阶与其相邻地面区域”内检查阻塞（不包含武器拼接区与终点区域）
            try:
                step_centers_y = [float(s['center'][1]) for s in base_steps]
                step_centers_x = [float(s['center'][0]) for s in base_steps]
                # 基于台阶中心构造一个保守包围盒：上下各扩 1.2m，左右各扩 1.2m
                y_min_area = min(step_centers_y) - 1.6  # 覆盖行1地面 (~0.7)
                y_max_area = max(step_centers_y) + 1.0  # 留一定上边余量，但排除拼接(>~8)
                x_min_area = min(step_centers_x) - 1.2
                x_max_area = max(step_centers_x) + 1.2
            except Exception:
                # 回退：若异常则不进行区域裁剪
                y_min_area = -float('inf')
                y_max_area = float('inf')
                x_min_area = -float('inf')
                x_max_area = float('inf')

            def _in_check_region(pos_tuple):
                try:
                    x, y = float(pos_tuple[0]), float(pos_tuple[1])
                    # 显式排除终点附近（斜坡）
                    ramp = PARAMS.get('ramp_pos')
                    if ramp is not None:
                        if np.hypot(x - float(ramp[0]), y - float(ramp[1])) < 0.5:
                            return False
                    return (x_min_area <= x <= x_max_area) and (y_min_area <= y <= y_max_area)
                except Exception:
                    return True

            new_ts = [float(r2_ts[0])]
            for j in range(1, len(r2_path)):
                p0 = np.array(r2_path[j - 1], dtype=float)
                p1 = np.array(r2_path[j], dtype=float)
                t0_orig, t1_orig = float(r2_ts[j - 1]), float(r2_ts[j])
                t0 = float(new_ts[-1])
                dur_orig = max(1e-9, t1_orig - t0_orig)
                seg = p1 - p0
                seg_norm2 = float(np.dot(seg, seg))
                t_block_end_needed = None
                block_with_idx = None
                for st in r1_stops:
                    # 区域过滤：仅在台阶/相邻地面范围内考虑 R1 停留造成的阻塞
                    if not _in_check_region(st['pos']):
                        continue
                    # 时间相交
                    if not (t0 <= st['t_end'] and (t0 + dur_orig) >= st['t_start']):
                        continue
                    c = np.array(st['pos'], dtype=float)
                    if seg_norm2 < 1e-12:
                        dist = float(np.hypot(*(c - p0)))
                    else:
                        tau = float(np.clip(np.dot(c - p0, seg) / seg_norm2, 0.0, 1.0))
                        nearest = p0 + tau * seg
                        dist = float(np.hypot(*(c - nearest)))
                    if dist < (r1_radius + r2_radius) - 1e-9:
                        t_block_end = float(st['t_end'])
                        if t_block_end_needed is None or t_block_end > t_block_end_needed:
                            t_block_end_needed = t_block_end
                            keyc = tuple(np.round(c, 2))
                            block_with_idx = pos_to_logic_index.get(keyc)
                if t_block_end_needed is not None and t_block_end_needed > t0:
                    delay = t_block_end_needed - t0
                    new_ts.append(t_block_end_needed + dur_orig)
                    r2_delay_events.append(
                        {'at_index': j, 'delay': delay, 't_block_end': t_block_end_needed, 'r1_idx': block_with_idx})
                else:
                    new_ts.append(t0 + dur_orig)
            r2_visit_time_blocked = {}
            for pos, t in zip(r2_path, new_ts):
                key = tuple(np.round(pos, 2))
                idx = pos_to_logic_index.get(key)
                if idx is not None and idx not in r2_visit_time_blocked:
                    r2_visit_time_blocked[idx] = float(t)
        # 判定是否需要等待：
        # 1) 若启用阻塞检查并出现任意阻塞延迟事件，则视为“需要等待”（即便最后到达时刻未早于R1拾取）
        # 2) 否则，按到达时刻与R1拾取对比（若有阻塞到达则优先使用）
        arrival_for_wait = r2_visit_time_blocked if r2_visit_time_blocked is not None else r2_visit_time
        wait = False
        # 1) 阻塞即等待
        if enable_block and r2_delay_events:
            wait = True
        # 2) 到达早于R1拾取也视为等待
        if not wait:
            for idx, t_r1 in r1_pick.items():
                t_r2 = arrival_for_wait.get(idx)
                if t_r2 is not None and (t_r2 + 1e-6) < t_r1:
                    wait = True
                    break
        # 生成摘要：R2事件（上台阶/转向）
        r2_events = []  # [{type, from, to, t, extra}]
        step_seq = []
        for pos in r2_path:
            key = tuple(np.round(pos, 2))
            step_idx = pos_to_logic_index.get(key)
            step_seq.append(step_idx)
        for k in range(1, len(step_seq)):
            i_prev, i_curr = step_seq[k - 1], step_seq[k]
            if i_prev is None or i_curr is None:
                continue
            if not isinstance(i_prev, int) or not isinstance(i_curr, int):
                continue
            if i_prev in logic_to_step and i_curr in logic_to_step and i_prev != i_curr:
                s_prev, s_curr = logic_to_step[i_prev], logic_to_step[i_curr]
                t_curr = float(r2_ts[k]) if k < len(r2_ts) else None
                # 转向（同行移动）
                if s_prev['row'] == s_curr['row']:
                    r2_events.append({
                        'type': 'turn', 'from': i_prev, 'to': i_curr, 't': t_curr
                    })
                # 上/下台阶（高度变化）
                dz = abs(s_prev['height'] - s_curr['height'])
                if dz > 0.01:
                    r2_events.append({
                        'type': 'climb', 'from': i_prev, 'to': i_curr, 't': t_curr, 'dz': round(dz, 3)
                    })
        # 等待点（若需等待）
        wait_points = []
        if wait:
            for idx, t_r1 in r1_pick.items():
                t_r2 = arrival_for_wait.get(idx)
                if t_r2 is not None and (t_r2 + 1e-6) < t_r1:
                    wait_points.append({'step': idx, 't_r2_arrive': t_r2, 't_r1_pick': t_r1})
        total_time = float(plan.get('total_time')) if plan.get('total_time') is not None else None
        summary = {
            'cap': cap,
            'manual_indices': manual_indices,
            'r1_pick': r1_pick,
            'r2_pick': {int(m.step_index): float(t) for m, t in (plan.get('r2_pickup_schedule') or {}).items() if
                        hasattr(m, 'step_index')},
            'r2_events': r2_events,
            'wait_points': wait_points,
            # 新增：R2 首次到达各 R1 主任务台阶的时刻（用于对比三种情况）
            'r2_first_arrival': r2_visit_time,
            # 新增：考虑阻塞后的 R2 首次到达与延迟事件（若启用）
            'r2_first_arrival_blocked': r2_visit_time_blocked,
            'r2_block_delays': r2_delay_events,
            # 新增：武器拼接完成时间与位置（如果有）
            'weapon_assembly_time': (float(plan.get('weapon_assembly_time'))
                                     if plan.get('weapon_assembly_time') is not None else None),
            'weapon_assembly_pos': (tuple(plan.get('weapon_assembly_pos'))
                                    if plan.get('weapon_assembly_pos') is not None else None),
        }
        return (True, wait, total_time, summary)
    except Exception:
        return (False, False, None, None)


def validate_r2_no_wait_across_placements(seed: int, limit: int | None = None, r2_capabilities=(0.4,),
                                          map_variant: str | None = None, processes: int = 8, plot_maps: bool = False,
                                          plots_dir: str | None = None, plot_show_r1_path: bool = True,
                                          enable_r1_block_check: bool = True, r1_radius: float | None = None,
                                          r2_radius: float | None = None):
    """验证在当前参数与任意（合法）秘籍摆放下，R2是否“无需等待”R1清理主任务R1秘籍。
    判据（非侵入、事后计算）：若 R2 在时间线上“首次进入某主任务R1台阶”的时刻早于 R1 对该台阶的拾取时刻，
    则该实例在“必须等待”规则下会产生等待，记为 wait_required=True；否则为 False。
    输出：单一总进度条 + 最终统计汇总。
    """
    from tqdm import tqdm

    # 生成基础台阶（不关心随机秘籍），随后用手动摆放重置
    with suppress_stdout():
        steps, _ = create_base_scenario(seed, map_variant=map_variant)
    if not steps:
        print("验证失败：无法生成基础台阶")
        return

    # 组合所有合法摆放（带预计算与剪枝）
    base_steps = [s for s in steps if not s.get('is_pseudo')]
    rows = {s['logic_index']: s['row'] for s in base_steps}
    cols = {s['logic_index']: s['col'] for s in base_steps}
    step_meta = {'rows': rows, 'cols': cols, 'valid': set(rows.keys())}

    all_indices = tuple(range(1, 13))
    from itertools import combinations
    def r1_combo_legal(r1_combo):
        for i in r1_combo:
            c = cols[i]
            r = rows[i]
            if (c == 2) and (r not in (1, 4)):
                return False
        return True

    tasks = []  # list[dict]
    fake_candidates = tuple(i for i in all_indices if rows[i] != 4)
    for r1_pos in (c for c in combinations(all_indices, PARAMS['r1_manual_total']) if r1_combo_legal(c)):
        remain_after_r1 = [i for i in all_indices if i not in r1_pos]
        for r2_pos in combinations(remain_after_r1, PARAMS['r2_manual_total']):
            remain_after_r2 = [i for i in remain_after_r1 if i not in r2_pos]
            for fake_idx in (i for i in remain_after_r2 if i in fake_candidates):
                manual_indices = {'R1': list(r1_pos), 'R2': list(r2_pos), 'fake': fake_idx}
                if _is_placement_legal(steps, manual_indices, step_meta=step_meta):
                    tasks.append(manual_indices)

    if limit is not None and limit > 0:
        tasks = tasks[:int(limit)]

    total = len(tasks) * len(r2_capabilities)
    if total == 0:
        print("无可验证的任务（可能是限制过小或规则过严）")
        return

    # 静默规划与日志；保留最终统计输出
    old_quiet = PARAMS.get('quiet', False)
    PARAMS['quiet'] = True

    checked, infeasible = 0, 0
    wait_required_cases = 0  # 至少一次“会等待”的实例计数
    worst = {'time': -1.0, 'summary': None}
    # 新增：在所有需要等待的实例中，记录“等待时长（R1拾取 - R2到达）最大”的情况
    worst_wait = {'wait_dt': -1.0, 'summary': None, 'point': None, 'time': None}
    # 新增：在所有需要等待的实例中，记录“总耗时最大”的实例
    wait_worst_total = {'time': -1.0, 'summary': None}

    # 构造任务列表：(manual_indices, cap)
    jobs = [
        (mi, cap, bool(enable_r1_block_check), r1_radius if r1_radius is not None else PARAMS.get('r1_radius', 0.5),
         r2_radius if r2_radius is not None else PARAMS.get('r2_radius', 0.4))
        for mi in tasks for cap in r2_capabilities
    ]

    pbar = tqdm(total=total, desc='验证R2无需等待', leave=True)
    try:
        if processes and processes > 1:
            # 并行（固定参数）
            from multiprocessing import Pool
            with Pool(processes=processes, initializer=_validate_init, initargs=(steps,)) as pool:
                base = len(jobs) // (processes * 64) if processes else 1
                chunk = max(8, min(64, base if base > 0 else 1))
                for feasible, wait, t_total, summary in pool.imap_unordered(_validate_job_worker, jobs,
                                                                            chunksize=chunk):
                    checked += 1
                    if not feasible:
                        infeasible += 1
                    else:
                        if wait:
                            wait_required_cases += 1
                            # 统计该实例中最长等待时长
                            try:
                                if summary and summary.get('wait_points'):
                                    local_point = max(
                                        summary['wait_points'],
                                        key=lambda w: max(0.0, float(w['t_r1_pick']) - float(w['t_r2_arrive']))
                                    )
                                    local_dt = max(0.0,
                                                   float(local_point['t_r1_pick']) - float(local_point['t_r2_arrive']))
                                    if local_dt > worst_wait['wait_dt']:
                                        worst_wait['wait_dt'] = local_dt
                                        worst_wait['summary'] = summary
                                        worst_wait['point'] = local_point
                                        worst_wait['time'] = t_total
                            except Exception:
                                pass
                            # 记录需等待实例中的“最耗时实例”
                            if t_total is not None and t_total > wait_worst_total['time']:
                                wait_worst_total['time'] = t_total
                                wait_worst_total['summary'] = summary
                        if t_total is not None and t_total > worst['time']:
                            worst['time'] = t_total
                            worst['summary'] = summary
                    pbar.update(1)
        else:
            # 串行
            _validate_init(steps)
            for job in jobs:
                feasible, wait, t_total, summary = _validate_job_worker(job)
                checked += 1
                if not feasible:
                    infeasible += 1
                else:
                    if wait:
                        wait_required_cases += 1
                        # 统计该实例中最长等待时长
                        try:
                            if summary and summary.get('wait_points'):
                                local_point = max(
                                    summary['wait_points'],
                                    key=lambda w: max(0.0, float(w['t_r1_pick']) - float(w['t_r2_arrive']))
                                )
                                local_dt = max(0.0, float(local_point['t_r1_pick']) - float(local_point['t_r2_arrive']))
                                if local_dt > worst_wait['wait_dt']:
                                    worst_wait['wait_dt'] = local_dt
                                    worst_wait['summary'] = summary
                                    worst_wait['point'] = local_point
                                    worst_wait['time'] = t_total
                        except Exception:
                            pass
                        # 记录需等待实例中的“最耗时实例”
                        if t_total is not None and t_total > wait_worst_total['time']:
                            wait_worst_total['time'] = t_total
                            wait_worst_total['summary'] = summary
                    if t_total is not None and t_total > worst['time']:
                        worst['time'] = t_total
                        worst['summary'] = summary
                pbar.update(1)
    finally:
        try:
            pbar.close()
        except Exception:
            pass

    PARAMS['quiet'] = old_quiet

    # 汇总输出（仿 run_full_enumeration：仅最终统计）
    feasible = checked - infeasible
    print("\n=== 验证报告：R2是否需要等待R1清理主任务R1秘籍 ===")
    print(f"样本摆放数: {len(tasks)}，能力组合: {len(r2_capabilities)}，总实例: {total}")
    print(f"已评估实例: {checked}，不可行实例: {infeasible}，可行实例: {feasible}")
    print(f"需等待实例数: {wait_required_cases}，不需等待实例数: {max(0, feasible - wait_required_cases)}")

    # 通用详情打印函数（与“最耗时实例详情”一致的版式）
    def _print_full_details(title: str, ws: dict, t_total=None, extra_prefix_lines: list | None = None):
        if ws is None:
            return
        print(f"\n=== {title} ===")
        if t_total is not None:
            print(f"R2能力: {ws['cap']}m；耗时: {float(t_total):.2f}s")
        else:
            print(f"R2能力: {ws['cap']}m；耗时: N/A")
        print(f"摆放(R1/R2/fake): {ws['manual_indices']}")
        # 等待标记：有等待点或有阻塞延迟事件都视为需要等待
        wait_flag = bool(ws.get('wait_points')) or bool(ws.get('r2_block_delays'))
        print(f"需等待: {'是' if wait_flag else '否'}")
        if extra_prefix_lines:
            for line in extra_prefix_lines:
                print(line)
        # 武器拼接
        if ws.get('weapon_assembly_time') is not None and ws.get('weapon_assembly_pos') is not None:
            pos = ws['weapon_assembly_pos']
            if isinstance(pos, (list, tuple, np.ndarray)):
                pos_fmt = tuple(np.round(pos, 2))
            else:
                pos_fmt = pos
            print(f"-- 武器拼接完成: t={ws['weapon_assembly_time']:.2f}s @ 坐标 {pos_fmt}")
        # R1拾取（三种相对关系）
        if ws.get('r1_pick'):
            r1_pick = ws['r1_pick']
            # 默认对比使用阻塞后的到达（若存在），并显示原始到达供参考
            r2_arr = ws.get('r2_first_arrival_blocked') or ws.get('r2_first_arrival') or {}
            r2_arr_raw = ws.get('r2_first_arrival') or {}
            eps = 1e-6
            earlier, equal, later = [], [], []
            for idx, t1 in r1_pick.items():
                t2 = r2_arr.get(int(idx))
                if t2 is None:
                    later.append((idx, t1, None))
                else:
                    if t1 + eps < t2:
                        earlier.append((idx, t1, t2))
                    elif abs(t1 - t2) <= eps:
                        equal.append((idx, t1, t2))
                    else:
                        later.append((idx, t1, t2))

            def _print_group(title2, arr):
                print(title2)
                if not arr:
                    print("  (无)")
                else:
                    for idx, t1, t2 in sorted(arr, key=lambda x: x[1]):
                        if t2 is None:
                            print(f"  台阶{idx}: R1={t1:.2f}s, R2=未到达")
                        else:
                            # 若存在原始到达，与阻塞到达不同则一起显示
                            t2_raw = r2_arr_raw.get(int(idx))
                            if t2_raw is not None and abs(t2_raw - t2) > 1e-6:
                                print(f"  台阶{idx}: R1={t1:.2f}s, R2(阻塞)={t2:.2f}s, R2(原始)={t2_raw:.2f}s")
                            else:
                                print(f"  台阶{idx}: R1={t1:.2f}s, R2={t2:.2f}s")

            _print_group("-- R1 拾取时间戳：R1 先于 R2 到达", earlier)
            _print_group("-- R1 拾取时间戳：R1 与 R2 同时(容差)到达", equal)
            _print_group("-- R1 拾取时间戳：R1 晚于 R2 到达/或R2未到达", later)
        # 若存在阻塞延迟事件，打印
        if ws.get('r2_block_delays'):
            print("-- R2 阻塞延迟事件：")
            for ev in ws['r2_block_delays']:
                info = []
                if ev.get('r1_idx'):
                    info.append(f"受 R1 台阶{ev['r1_idx']} 停留影响")
                print(
                    f"  段#{ev['at_index']}: 延迟 {float(ev['delay']):.2f}s, 解封 t={float(ev['t_block_end']):.2f}s" + (
                        " (" + ", ".join(info) + ")" if info else ""))
        # R2拾取
        if ws.get('r2_pick'):
            print("-- R2 拾取时间戳(按时序)：")
            for idx, t in sorted(ws['r2_pick'].items(), key=lambda x: x[1]):
                print(f"  台阶{idx}: t={t:.2f}s")
        # R2事件
        if ws.get('r2_events'):
            print("-- R2 事件时间戳：")
            for ev in ws['r2_events']:
                if ev['type'] == 'climb':
                    print(f"  上/下台阶: {ev['from']} -> {ev['to']}, Δh={ev.get('dz')}m, t={ev['t']:.2f}s")
                elif ev['type'] == 'turn':
                    print(f"  转向(同行): {ev['from']} -> {ev['to']}, t={ev['t']:.2f}s")
        # 等待点
        if ws.get('wait_points'):
            print("-- R2 等待点：")
            for w in ws['wait_points']:
                dt = max(0.0, float(w['t_r1_pick']) - float(w['t_r2_arrive']))
                print(
                    f"  台阶{w['step']}: R2到达 t={w['t_r2_arrive']:.2f}s, R1拾取 t={w['t_r1_pick']:.2f}s (等待时长={dt:.2f}s)")

    # 确保输出目录
    if plot_maps:
        try:
            out_dir = plots_dir or 'validation_plots'
            os.makedirs(out_dir, exist_ok=True)
        except Exception:
            plot_maps = False
            print("[WARN] 地图输出目录创建失败，已禁用绘图。")

    def _rebuild_and_plot(summary: dict, label: str, t_total: float | None = None):
        """基于 summary 重建场景并输出带 R1/R2 路径的静态图。"""
        if not plot_maps or not summary:
            return
        try:
            cap = float(summary.get('cap')) if summary.get('cap') is not None else 0.4
            # 重建基础台阶与手动摆放
            with suppress_stdout():
                steps2, _ = create_base_scenario(seed, map_variant=map_variant)
            if not steps2:
                return
            manuals2 = create_manual_scenario(steps2, summary['manual_indices'])
            if not manuals2:
                return
            # 运行一次规划以获取完整路径
            with suppress_stdout():
                sim2 = Simulation(copy.deepcopy(steps2), manuals2, cap, run_planner=True)
            plan2 = getattr(sim2, 'plan', None)
            if not plan2 or not plan2.get('r2_path') or not plan2.get('r1_path'):
                return
            r2_pts = list(plan2['r2_path'])
            r1_pts = list(plan2['r1_path'])
            # 生成文件名
            mv = (map_variant or MAP_VARIANT).lower()
            cap_str = f"{cap:.1f}m".replace('.0', '')
            ttag = f"{t_total:.2f}s" if isinstance(t_total, (int, float)) else "NA"
            fname = os.path.join(plots_dir or 'validation_plots', f"{mv}_seed{seed}_cap{cap_str}_{label}_{ttag}.png")
            draw_map_with_paths(steps2, manuals2, r2_pts, r1_pts, filename=fname, title_suffix=f"{label}",
                                show_r1=bool(plot_show_r1_path))
        except Exception as e:
            print(f"[WARN] 绘制 {label} 地图失败: {e}")

    # 输出最耗时实例详情
    if worst['summary'] is not None:
        _print_full_details("最耗时实例详情", worst['summary'], worst['time'])
        _rebuild_and_plot(worst['summary'], label='overall_worst', t_total=worst['time'])

    # 需等待实例中的“最耗时实例详情”（完整）
    if wait_required_cases > 0 and wait_worst_total['summary'] is not None:
        _print_full_details("需等待实例中的最耗时实例详情", wait_worst_total['summary'], wait_worst_total['time'])
        _rebuild_and_plot(wait_worst_total['summary'], label='wait_group_worst_total', t_total=wait_worst_total['time'])

    # 需等待实例中的“最长等待实例详情”（完整）
    if wait_required_cases > 0 and worst_wait['summary'] is not None and worst_wait['point'] is not None:
        wp = worst_wait['point']
        extra = [
            f"最长等待时长: {worst_wait['wait_dt']:.2f}s @ 台阶{wp['step']}",
            f"  细节: R2到达 t={float(wp['t_r2_arrive']):.2f}s, R1拾取 t={float(wp['t_r1_pick']):.2f}s"
        ]
        _print_full_details("需等待实例中的最长等待实例详情", worst_wait['summary'], worst_wait.get('time'),
                            extra_prefix_lines=extra)
        _rebuild_and_plot(worst_wait['summary'], label='wait_group_longest_delay', t_total=worst_wait.get('time'))


def run_simulation_planning(args):
    """多进程执行规划的函数"""
    steps, manuals, r2_max_step_height, suffix = args
    # 捕获输出
    old_stdout = sys.stdout
    sys.stdout = captured_output = StringIO()
    # 诊断：打印子进程导入的模块文件路径
    try:
        print(f"[子进程模块路径] {__file__}")
    except Exception:
        pass
    sim = Simulation(steps, manuals, r2_max_step_height)
    # 诊断：若已生成plan且Planner存在伪台阶结构，输出数量与legacy状态
    try:
        # 无法直接访问Planner实例，这里仅在日志中标记子进程诊断已运行
        print("[子进程诊断] 规划执行完毕，已输出日志（如存在伪台阶创建日志，应在上方可见）")
    except Exception:
        pass
    sys.stdout = old_stdout
    log_output = captured_output.getvalue()
    return sim, suffix, log_output


def validate_step_mappings(steps):
    """严格校验映射关系，初始化时排查错误"""
    logic_indices = [s['logic_index'] for s in steps]
    # 校验逻辑索引唯一
    if len(logic_indices) != len(set(logic_indices)):
        raise ValueError("台阶逻辑索引存在重复")
    # 校验坐标与行列匹配（支持 blue 模式下的 X 轴镜像）
    mv = (MAP_VARIANT or 'red').lower()
    for s in steps:
        # 新中心坐标期望：x = 0.6 + 1.2 * col，y = 0.8 + 1.2 * row（col,row 为 1-based）
        if mv == 'blue':
            # blue: 左右镜像，镜像轴为 x = 2.4；等价于 col' = 4 - col
            col_eff = 4 - s['col']
        else:
            col_eff = s['col']
        expected_x = 0.6 + 1.2 * col_eff
        expected_y = 0.8 + 1.2 * s['row']
        if not np.isclose(s['center'][0], expected_x, atol=0.01) or \
                not np.isclose(s['center'][1], expected_y, atol=0.01):
            raise ValueError(f"台阶{s['logic_index']}坐标与行列不匹配："
                             f"实际({s['center']}) vs 预期({expected_x:.2f},{expected_y:.2f})")
    # 校验邻居关系为逻辑索引且真实相邻
    for s in steps:
        for neighbor_logic_idx in s['neighbors']:
            neighbor = next((n for n in steps if n['logic_index'] == neighbor_logic_idx), None)
            if not neighbor:
                raise ValueError(f"台阶{s['logic_index']}的邻居{neighbor_logic_idx}不存在")
            # 校验物理相邻
            if not (abs(s['row'] - neighbor['row']) == 1 and s['col'] == neighbor['col']) and \
                    not (abs(s['col'] - neighbor['col']) == 1 and s['row'] == neighbor['row']):
                raise ValueError(f"台阶{s['logic_index']}与{neighbor_logic_idx}非物理相邻，却被标记为邻居")
    print("台阶映射关系校验通过")


if __name__ == '__main__':
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False

    simulation_seed = 54  # 随机种子（可修改）
    run_full_enumeration = True  # 设为 True 运行“遍历所有合法放置并输出txt”模式
    use_manual_placement = False  # 设置为 True 启用手动放置，False 为随机
    # 验证开关（简单True/False）：开启后将对“任意合法摆放”执行验证，统计是否存在R2需要等待R1的迹象
    run_validate_r2_no_wait = False
    # 验证样本上限（None 表示全部合法摆放；调小可做快速验证）
    validate_sample_limit = None
    # 手动放置秘籍的位置（使用 1-based 逻辑编号 logic_index）
    manual_indices = {
        'R1': [1, 2, 4],  # 3个 R1 秘籍（逻辑编号）
        'R2': [3, 8, 7, 12],  # 4个 R2 秘籍（逻辑编号）
        'fake': 5  # 假秘籍（逻辑编号）
    }

    # 轻量命令行参数：--enum-sample N（只跑前N个合法放置）；--enum-processes P；--enum-out file；--map red|blue
    # 内部可编辑配置：无需命令行参数，直接在这里修改即可生效
    # 如需恢复命令行控制，可自行改回 argparse 或将这些字段与外部参数对齐
    INTERNAL_ARGS = {
        # 枚举相关
        'enum_sample': None,  # 仅枚举前 N 个合法放置（None 表示不限制）
        'enum_processes': 8,  # 枚举时的并行进程数
        'enum_out': None,  # 输出文件名（None 使用默认）

        # 地图主题（None 表示由 use_blue_map 决定；也可强制 'red' 或 'blue'）
        'map': None,

        # 验证相关
        'validate_r2_no_wait': False,  # 是否开启“R2无需等待”验证
        'validate_plots': True,  # 是否在验证报告中导出三张关键实例地图
        'validate_plots_dir': None,  # 地图输出目录（None=validation_plots）
        'validate_processes': 8,  # 验证并行进程数
        'plot_r1_path': True,  # 地图中是否绘制 R1 路径（仅影响验证报告中的静态图）
        # 是否在验证中同时包含 0.2m 能力（默认仅 0.4m）
        'validate_include_02': False,
        # 验证期的碰撞/阻塞考虑（不影响规划，仅改变“是否需要等待”的判定）
        'enable_r1_block_check': True,
        'r1_radius': None,  # 覆盖 R1 半径，None=使用 PARAMS
        'r2_radius': None,  # 覆盖 R2 半径，None=使用 PARAMS

        # 行为参数覆盖
        'r1_pickup_dwell': None,  # 覆盖 R1 每次拾取停留秒数，None 表示使用 PARAMS 中的默认值

        # 是否在遍历中同时包含 0.2m 能力（默认仅 0.4m）
        'enum_include_02': False,
    }
    from types import SimpleNamespace

    args = SimpleNamespace(**INTERNAL_ARGS)

    # 应用地图变体参数或本地布尔开关
    # 若命令行未提供 --map，可通过此布尔开关快速切换（True=blue，False=red）
    use_blue_map = True  # 将此改为 True 可用蓝图（右下=1，左上=12；X 轴镜像）
    if args.map:
        MAP_VARIANT = args.map
    else:
        MAP_VARIANT = 'blue' if use_blue_map else 'red'

    # 在 blue 模式下：将启动区/对接区/支架/矛头/终点等关键点的 X 坐标做镜像
    if (MAP_VARIANT or 'red').lower() == 'blue':
        mirror_axis_x = 3.0  # 按新公式中列中心 x = 3.0


        def _mx(p):
            return (2 * mirror_axis_x - p[0], p[1])


        # 启动区
        PARAMS['r1_start_pos'] = _mx(PARAMS['r1_start_pos'])
        PARAMS['r2_start_pos'] = _mx(PARAMS['r2_start_pos'])
        # 支架/对接区（支撑点）
        PARAMS['support_pos'] = _mx(PARAMS['support_pos'])
        # 矛头区
        PARAMS['spear_pos'] = _mx(PARAMS['spear_pos'])
        # 终点
        PARAMS['ramp_pos'] = _mx(PARAMS['ramp_pos'])

    # 覆盖R1拾取停留参数（如提供）
    if getattr(args, 'r1_pickup_dwell', None) is not None:
        try:
            dwell = float(args.r1_pickup_dwell)
            if dwell < 0:
                print('[WARN] --r1-pickup-dwell 小于0，将按0处理')
                dwell = 0.0
            PARAMS['r1_pickup_dwell'] = dwell
        except Exception:
            print('[WARN] --r1-pickup-dwell 非法，保持默认值')

    if run_validate_r2_no_wait or args.validate_r2_no_wait:
        # 启动验证模式：不改变任何规划与枚举逻辑，仅做事后验证统计
        mv = args.map or MAP_VARIANT
        limit = validate_sample_limit if run_validate_r2_no_wait else args.enum_sample
        # 根据开关决定验证能力集
        caps_validate = (0.4, 0.2) if bool(getattr(args, 'validate_include_02', False)) else (0.4,)
        validate_r2_no_wait_across_placements(
            seed=simulation_seed,
            limit=limit,
            r2_capabilities=caps_validate,
            map_variant=mv,
            processes=int(getattr(args, 'validate_processes', 8) or 8),
            plot_maps=bool(getattr(args, 'validate_plots', False)),
            plots_dir=getattr(args, 'validate_plots_dir', None),
            plot_show_r1_path=bool(getattr(args, 'plot_r1_path', True)),
            enable_r1_block_check=bool(getattr(args, 'enable_r1_block_check', True)),
            r1_radius=getattr(args, 'r1_radius', None),
            r2_radius=getattr(args, 'r2_radius', None),
        )
        sys.exit(0)
    elif run_full_enumeration or args.enum_sample:
        # 遍历模式：不输出地图、不输出调试，仅显示总进度条并生成txt
        # 根据开关决定遍历能力集
        caps_enum = (0.4, 0.2) if bool(getattr(args, 'enum_include_02', False)) else (0.4,)
        enumerate_all_placements_and_write(
            simulation_seed,
            r2_capabilities=caps_enum,
            out_file=args.enum_out,
            show_progress=True,
            max_tasks=args.enum_sample,
            processes=args.enum_processes,
            map_variant=MAP_VARIANT,
        )
        sys.exit(0)
    elif use_manual_placement:
        # 手动模式：先生成台阶，再手动放置秘籍
        steps, _ = create_base_scenario(simulation_seed, map_variant=MAP_VARIANT)  # 只生成台阶
        if steps:
            validate_step_mappings(steps)  # 初始化后立即校验
            manuals = create_manual_scenario(steps, manual_indices)
            if not manuals:
                print("手动放置失败，回退到随机模式")
                steps, manuals = create_base_scenario(simulation_seed, map_variant=MAP_VARIANT)
                if steps:
                    validate_step_mappings(steps)  # 初始化后立即校验
        else:
            print("无法生成地图")
            exit(1)
    else:
        # 随机模式
        steps, manuals = create_base_scenario(simulation_seed, map_variant=MAP_VARIANT)
        if steps:
            validate_step_mappings(steps)  # 初始化后立即校验

    # 校验台阶映射关系
    try:
        validate_step_mappings(steps)
    except ValueError as e:
        print(f"错误：{e}")
        exit(1)

    if steps and manuals:
        # 定义要模拟的场景（不同R2越障能力）
        mv = (MAP_VARIANT or 'red').lower()
        scenarios = [
            (steps, manuals, 0.2, f"{mv}_R2_step_0.2m_seed{simulation_seed}"),
            (steps, manuals, 0.4, f"{mv}_R2_step_0.4m_seed{simulation_seed}")
        ]
        # 调试：可禁用多进程以便完整输出日志
        use_multiprocessing = False
        if use_multiprocessing:
            num_processes = min(8, os.cpu_count() if os.cpu_count() else 1)
            print(f"\n--- 使用 {num_processes} 个进程并行执行规划 ---")
            with multiprocessing.Pool(processes=num_processes) as pool:
                results = pool.map(run_simulation_planning, scenarios)
            print("\n--- 所有规划任务已完成 ---")
        else:
            print("\n--- 单进程顺序执行规划（调试模式） ---")
            results = [run_simulation_planning(s) for s in scenarios]

        all_failed = True
        # 处理每个场景的结果
        for sim, suffix, log_output in results:
            print("\n" + "=" * 20 + f" 场景: {suffix} " + "=" * 20)
            print(log_output)

            # 有有效计划则输出带R2路径的地图；否则仅输出底图
            if sim.plan and sim.plan.get('r2_path') and sim.plan.get('r2_timestamps'):
                all_failed = False
                r2_path_points = sim.plan['r2_path']
                r1_path_points = sim.plan.get('r1_path', [])
                print("将输出含R1/R2完整路径的静态地图（来源：规划结果）")
                out_name = f"map_with_paths_{suffix}.png"
                draw_map_with_paths(steps, manuals, r2_path_points, r1_path_points, filename=out_name, show_r1=True)
            else:
                print(f"场景 {suffix} 无法生成有效计划，仅输出底图")
                out_name = f"map_{suffix}.png"
                draw_map(steps, manuals, filename=out_name)

        # 如果所有场景都失败，输出地图
        if all_failed:
            print("\nR2在0.2m和0.4m越障能力下都无法生成有效计划，输出该种子的地图：")
            mv = (MAP_VARIANT or 'red').lower()
            draw_map(steps, manuals, filename=f"map_{mv}_seed{simulation_seed}.png")
    else:
        print("无法创建有效的地图布局。")
