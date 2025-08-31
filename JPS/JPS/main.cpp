#include "JPS.h"
vector<vector<int>> grid1 = {
    {0,0,0,0,0},
    {0,0,0,0,0},
    {0,0,0,0,0},
    {0,0,0,0,0},
    {0,0,0,0,0}
};
// 路径: (0,0) -> (4,4)
vector<vector<int>> grid2 = {
    {0,0,0,0,0},
    {0,1,1,1,0},
    {0,1,1,1,0},
    {0,1,1,1,0},
    {0,0,0,0,0}
};
// 路径: (0,0) -> (4,4) 应该绕开中间障碍物

vector<vector<int>> grid3 = {
    {0,1,0,0,0,0,0},
    {0,1,0,1,1,1,0},
    {0,1,0,1,0,0,0},
    {0,0,0,1,0,1,0},
    {1,1,0,1,0,1,0},
    {0,0,0,0,0,1,0}
};
// 路径: (0,0) -> (6,5)

vector<vector<int>> grid4 = {
    {1,1,1,1,1,1,1,0},
    {1,0,0,0,0,0,1,0},
    {1,0,0,0,1,0,1,0},
    {1,0,0,0,1,0,1,0},
    {1,0,0,0,1,0,0,0},
    {1,1,1,1,1,1,1,1}
};
// 路径: (1,4) -> (6,4)

vector<vector<int>> grid5 = {
    {0,0,0,0,0,0,0},
    {0,1,1,1,1,1,0},
    {0,1,0,0,0,1,0},
    {0,1,0,1,0,1,0},
    {0,1,0,0,0,1,0},
    {0,1,1,1,1,1,0},
    {0,0,0,0,0,0,0}
};
// 路径: (0,0) -> (6,6)

vector<vector<int>> grid6 = {
    {0,0,0,1,0,0,0},
    {0,1,0,1,0,1,0},
    {0,1,0,1,0,1,0},
    {0,1,0,1,0,1,0},
    {0,1,0,1,0,1,0},
    {0,1,0,0,0,1,0},
    {0,1,1,1,1,1,0}
};
// 路径: (0,0) -> (6,0) 应该无法到达

vector<vector<int>> grid7(20, vector<int>(20, 0));
// 路径: (0,0) -> (19,19)

vector<vector<int>> grid8 = {
    {0,1,0},
    {0,1,0},
    {0,0,0}
};
// 路径: (0,0) -> (2,2)

vector<vector<int>> grid9 = {
    {0,0,0,0,0,1,0,0,0,0},
    {0,1,1,1,0,1,0,1,1,0},
    {0,0,0,1,0,1,0,1,0,0},
    {0,1,0,1,0,1,0,1,0,1},
    {0,1,0,1,0,0,0,1,0,0},
    {0,1,0,1,1,1,1,1,1,0},
    {0,1,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,0,0,0}
};
// 路径: (0,0) -> (9,8)

// 可选：路径可视化函数
void visualizePath(const vector<vector<int>>& grid, const vector<Node*>& path) {
    vector<vector<char>> display(grid.size(), vector<char>(grid[0].size(), '.'));

    // 标记障碍物
    for (int y = 0; y < grid.size(); y++) {
        for (int x = 0; x < grid[0].size(); x++) {
            if (grid[y][x] == 1) {
                display[y][x] = '#';
            }
        }
    }

    // 标记路径
    for (Node* node : path) {
        display[node->y][node->x] = '*';
    }

    // 标记起点和终点
    if (!path.empty()) {
        display[path[0]->y][path[0]->x] = 'S';
        display[path.back()->y][path.back()->x] = 'E';
    }

    // 打印网格
    for (const auto& row : display) {
        for (char cell : row) {
            cout << cell << " ";
        }
        cout << endl;
    }
}

void testJPS(const vector<vector<int>>& grid, int startX, int startY, int endX, int endY, const string& testName) {
    cout << "=== " << testName << " ===" << endl;

    JPS jps(grid);
    auto path = jps.findPath(startX, startY, endX, endY);

    if (!path.empty()) {
        cout << "找到路径 (" << path.size() << " 个节点):" << endl;
        for (Node* node : path) {
            cout << "(" << node->x << "," << node->y << ") ";
        }
        cout << endl;

        // 可视化路径
        visualizePath(grid, path);
    }
    else {
        cout << "未找到路径!" << endl;
    }
    cout << "====================" << endl << endl;
}



int main() {

    // grid7添加一些随机障碍物
    grid7[5][5] = grid7[5][6] = grid7[5][7] = grid7[5][8] = 1;
    grid7[10][2] = grid7[10][3] = grid7[10][4] = grid7[10][5] = 1;
    grid7[15][8] = grid7[15][9] = grid7[15][10] = grid7[15][11] = 1;

    // 运行所有测试用例
    testJPS(grid1, 0, 0, 4, 4, "简单直线路径");
    testJPS(grid2, 0, 0, 4, 4, "有障碍物的直线路径");
    testJPS(grid3, 0, 0, 6, 5, "迷宫式路径");
    testJPS(grid4, 1, 4, 6, 4, "原始测试用例");
    testJPS(grid5, 0, 0, 6, 6, "多次对角线移动");
    testJPS(grid6, 0, 0, 6, 0, "死胡同测试");
    testJPS(grid7, 0, 0, 19, 19, "大型网格性能测试");
    testJPS(grid8, 0, 0, 2, 2, "边界情况测试");
    testJPS(grid9, 0, 0, 9, 8, "绕远路复杂路径");

    return 0;
}
