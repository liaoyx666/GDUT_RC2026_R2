// 递归搜索最优路径
// 设置打分机制，初始KFS摆放，输出最佳路径
// 默认最后一行不平移
#include <string.h>
#include <iostream>

int count = 0;// 递归次数

#define MAX_PATH_NUM 100// 最大储存路径数量

// 为了简化计算，不允许机器人后退，只允许前，左，右移动

typedef enum KFS {
    R1___,
    R2___,
    FAKE_,
    EMPTY,
    INVALID // 无效
} KFS;

/*-----------------------------------------------------------------------*/
/*----------------------------要改的地方-----------------------------------*/
/*-----------------------------------------------------------------------*/
// 约束条件，简化计算
#define MAX_TURN        3// 最横向移动数
#define MAX_STEP        7// 最大步数
#define MIN_KFS         3// 最少拿几个KFS
#define MAX_R1_HELP     3// 最大R1帮助次数

// 打分标准
int TURN_SCORE = -63;// 横向移动一次
int STEP_SCORE = -55;// 走一步
int KFS_SCORE = 124;// 一个kfs
int R1_HELP_SCORE = -13;// R1帮助一次
int DISTANCE_SCORE = -4;// 在非梅花桩区域运动的距离打分

// 初始kfs摆放
const KFS kfs[12] = {
    R1___,R2___,R2___,
    EMPTY,FAKE_,R1___,
    R1___,R2___,EMPTY,
    R2___,EMPTY,R1___
};
//         |启动区|
//               |
//|--------      |
//|  1,  2,  3   |
//|  4,  5,  6   |
//|  7,  8,  9   |
//|  10, 11, 12  |
//|    ----------|
//|通道|
/*-------------------------------------------------------------------*/
/*----------------------------要改的地方-------------------------------*/
/*-------------------------------------------------------------------*/

typedef enum Direction {
    FRONT,
    BACK,
    LEFT,
    RIGHT
} Direction;

typedef struct Path {
    int R1_help_num;    // R1帮助次数
    int step_num;       // 步数
    int turn_num;       // 横向移动次数
    int kfs_num;        // 拿取kfs数量
    int step[MAX_STEP]; // 步骤
    bool valid;         // 是否有效
    KFS now_kfs[12];    // 当前kfs摆放

    int score;          // 分数
} Path;

/*-------------------------------------------------------------*/

Path valid_path[MAX_PATH_NUM] = {0};
int valid_path_num = 0;


Path best_path[MAX_PATH_NUM] = {0};
int best_path_num = 0;

/*--------------------------------------------------------*/
void InitPath(Path *path) {
    path->R1_help_num = 0;
    path->step_num = 0;
    path->turn_num = 0;
    path->kfs_num = 0;
    path->valid = true;
    memset(path->step, 0, MAX_STEP * sizeof(int));
    memcpy(path->now_kfs, kfs, sizeof(kfs));

    path->score = 100;
}

void CopyPath(Path *dest, const Path *src) {
    dest->R1_help_num = src->R1_help_num;
    dest->step_num = src->step_num;
    dest->turn_num = src->turn_num;
    dest->kfs_num = src->kfs_num;
    memcpy(dest->step, src->step, MAX_STEP * sizeof(int));
    dest->valid = src->valid;
    memcpy(dest->now_kfs, src->now_kfs, sizeof(kfs));
}

void AddPath(const Path *path) {
    if (path->valid && valid_path_num < MAX_PATH_NUM) {
        CopyPath(&valid_path[valid_path_num], path);
        valid_path_num++;
    }
}

bool Is_valid(const Path *path) {
    if (!path->valid) return false;
    if (path->R1_help_num > MAX_R1_HELP) return false;
    if (path->step_num > MAX_STEP) return false;
    if (path->turn_num > MAX_TURN) return false;
    return true;
}

// 到达10,11,12 (索引9,10,11)
bool Is_end(const Path *path) {
    if (path->step_num < 4) return false;

    int last_pos = path->step[path->step_num - 1];
    if (last_pos == 9 || last_pos == 10 || last_pos == 11) {
        if (path->kfs_num >= MIN_KFS) {
            AddPath(path);
        }
        return true;
    }
    return false;
}

KFS CheckKFS(const Path *path, int position) {
    if (position < 0 || position > 11) return INVALID;
    return path->now_kfs[position];
}

// 拿取四周的kfs
void CatchKFS(Path *path) {
    int current_pos = path->step[path->step_num - 1];

    // 检查前、后、左、右四个方向
    int positions[4];
    positions[0] = current_pos + 3;  // 前
    positions[1] = current_pos - 3;  // 后

    for (int i = 0; i < 2; i++) {
        if (positions[i] >= 0 && positions[i] <= 11) {
            if (CheckKFS(path, positions[i]) == R2___) {
                path->now_kfs[positions[i]] = EMPTY;
                path->kfs_num++;
            }
        }
    }


    positions[2] = current_pos - 1;  // 左
    if (current_pos % 3 != 0) {
        if (CheckKFS(path, positions[2]) == R2___) {
            path->now_kfs[positions[2]] = EMPTY;
            path->kfs_num++;
        }
    }


    positions[3] = current_pos + 1;  // 右
    if (current_pos % 3 != 2) {
        if (CheckKFS(path, positions[3]) == R2___) {
            path->now_kfs[positions[3]] = EMPTY;
            path->kfs_num++;
        }
    }

}

bool CanMoveTo(const Path *path, int new_pos) {
    if (new_pos < 0 || new_pos > 11) return false;

    KFS target = path->now_kfs[new_pos];
    return (target == EMPTY || target == R1___);
}


// 只准向前，左，右移动
bool MoveOneStep(Path *path, Direction dir) {
    int current_pos = path->step[path->step_num - 1];
    int new_pos = current_pos;

    switch (dir) {
        case FRONT:
            new_pos += 3;
            if (new_pos < 0 || new_pos > 11) {
                return false;
            }
            break;
        case LEFT:
            if (new_pos % 3 == 0) {
                return false;
            }
            new_pos -= 1;
            break;

        case RIGHT:
            if (new_pos % 3 == 2) {
                return false;
            }
            new_pos += 1;
            break;
        default: return false;
    }

    if (!CanMoveTo(path, new_pos)) {
        return false;
    }

    // 更新路径
    path->step_num++;
    path->step[path->step_num - 1] = new_pos;

    if (dir == LEFT || dir == RIGHT) {
        path->turn_num++;
    }

    // 处理新位置上的KFS
    if (path->now_kfs[new_pos] == R1___) {
        path->R1_help_num++;
        path->now_kfs[new_pos] = EMPTY;
    }

    return true;
}

void Move(Path *path) {
    count++;
    if (!Is_valid(path)) {
        return;
    }

    CatchKFS(path);

    if (Is_end(path)) {
        return;
    }

    // 递归搜索三个方向
    Path path_front, path_left, path_right;
    CopyPath(&path_front, path);
    CopyPath(&path_left, path);
    CopyPath(&path_right, path);

    if (MoveOneStep(&path_front, FRONT)) {
        Move(&path_front);
    }

    if (MoveOneStep(&path_left, LEFT)) {
        Move(&path_left);
    }

    if (MoveOneStep(&path_right, RIGHT)) {
        Move(&path_right);
    }
}

void StartMove(Path *path, int start_pos) {
    if (start_pos < 0 || start_pos > 2) {
        path->valid = false;
        return;
    }

    path->step_num = 1;
    path->step[0] = start_pos;  // 使用0-11索引

    // 处理起始位置的KFS
    switch (path->now_kfs[start_pos]) {
        case R1___:
            path->R1_help_num++;
            path->now_kfs[start_pos] = EMPTY;
            break;

        case R2___:
            path->kfs_num++;
            path->now_kfs[start_pos] = EMPTY;
            break;

        case EMPTY:
            break;

        default:
            path->valid = false;
            return;
    }

    Move(path);
}

void Print_Path(Path *path) {
    std::cout << "path:     ";
    for (int i = 0; i < path->step_num; i++) {
        std::cout << path->step[i] + 1 << ",";
    }
    std::cout << std::endl;
    std::cout << "kfs:      " << path->kfs_num << std::endl;
    std::cout << "step:     " << path->step_num << std::endl;
    std::cout << "R1_help:  " << path->R1_help_num << std::endl;
    std::cout << "turn:     " << path->turn_num << std::endl;
    std::cout << "score:    " << path->score << std::endl;
    std::cout << std::endl;
}


void CalcScroe(Path *path) {
    path->score =
        path->step_num * STEP_SCORE +
        path->R1_help_num * R1_HELP_SCORE +
        path->turn_num * TURN_SCORE +
        path->kfs_num * KFS_SCORE +
        path->step[0] * -DISTANCE_SCORE + path->step[path->step_num - 1] * DISTANCE_SCORE;// 开始3最近，结束10最近
}

void FindBestPath() {
    if (valid_path_num != 0) {
        for (int i = 0; i < valid_path_num; i++) {
            CalcScroe(&valid_path[i]);
        }

        best_path_num = 1;
        best_path[best_path_num - 1] = valid_path[0];

        for (int i = 1; i < valid_path_num; i++) {
            if (valid_path[i].score > best_path[best_path_num - 1].score) {
                best_path_num = 1;
                best_path[best_path_num - 1] = valid_path[i];
            } else if (valid_path[i].score == best_path[best_path_num - 1].score) {
                best_path_num++;
                best_path[best_path_num - 1] = valid_path[i];
            }
        }
    }
}




int main() {
    Path path_1, path_2, path_3;

    InitPath(&path_1);
    InitPath(&path_2);
    InitPath(&path_3);

    StartMove(&path_1, 0);  // 从位置1开始
    StartMove(&path_2, 1);  // 从位置2开始
    StartMove(&path_3, 2);  // 从位置3开始

    std::cout << "valid_path_num:" << valid_path_num << std::endl;
    std::cout << std::endl;

    FindBestPath();
    std::cout << "best_path_num:" << best_path_num << std::endl;
    std::cout << std::endl;

    for (int i = 0; i < best_path_num; i++) {
        Print_Path(&best_path[i]);
    }

    std::cout << "count:" << count;
    return 0;
}