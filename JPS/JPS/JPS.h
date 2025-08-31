#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    float g, h, f;
    Node* parent;

    Node(int x, int y, Node* parent = nullptr)
        : x(x), y(y), g(0), h(0), f(0), parent(parent) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<> struct hash<Node> {
        size_t operator()(const Node& n) const {
            return hash<int>()(n.x) ^ (hash<int>()(n.y) << 1);
        }
    };
}

class JPS {
private:
    vector<vector<int>> grid;
    int width, height;

    // 启发式函数
    float heuristic(int x1, int y1, int x2, int y2) {
        return abs(x1 - x2) + abs(y1 - y2);
    }

    // 检查坐标是否有效
    bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height && grid[y][x] == 0;
    }

    // 检查是否是障碍物
    bool isObstacle(int x, int y) {
        return x < 0 || x >= width || y < 0 || y >= height || grid[y][x] == 1;
    }

    // 检查是否有强迫邻居
    bool hasForcedNeighbor(int x, int y, int dx, int dy) {
        // 水平移动
        if (dx != 0 && dy == 0) {
            // 检查上方和下方的强迫邻居
            if ((isObstacle(x, y + 1) && isValid(x + dx, y + 1)) ||
                (isObstacle(x, y - 1) && isValid(x + dx, y - 1))) {
                return true;
            }
        }
        // 垂直移动
        else if (dy != 0 && dx == 0) {
            // 检查左边和右边的强迫邻居
            if ((isObstacle(x + 1, y) && isValid(x + 1, y + dy)) ||
                (isObstacle(x - 1, y) && isValid(x - 1, y + dy))) {
                return true;
            }
        }
        // 对角线移动
        else if (dx != 0 && dy != 0) {
            // 检查水平方向的强迫邻居
            if ((isObstacle(x - dx, y) && isValid(x - dx, y + dy)) ||
                (isObstacle(x, y - dy) && isValid(x + dx, y - dy))) {
                return true;
            }
        }
        return false;
    }

    // 直线跳跃
    Node* jumpStraight(int x, int y, int dx, int dy, Node* goal) {
        int nx = x + dx;
        int ny = y + dy;

        while (isValid(nx, ny)) {
            // 到达目标
            if (nx == goal->x && ny == goal->y) {
                return new Node(nx, ny);
            }

            // 检查强迫邻居
            if (hasForcedNeighbor(nx, ny, dx, dy)) {
                return new Node(nx, ny);
            }

            // 继续跳跃
            nx += dx;
            ny += dy;
        }

        return nullptr;
    }

    // 对角线跳跃
    Node* jumpDiagonal(int x, int y, int dx, int dy, Node* goal) {
        int nx = x + dx;
        int ny = y + dy;

        while (isValid(nx, ny)) {
            // 到达目标
            if (nx == goal->x && ny == goal->y) {
                return new Node(nx, ny);
            }

            // 检查水平和垂直方向是否有跳点
            Node* horizontalJump = jumpStraight(nx, ny, dx, 0, goal);
            Node* verticalJump = jumpStraight(nx, ny, 0, dy, goal);

            if (horizontalJump != nullptr || verticalJump != nullptr) {
                delete horizontalJump;
                delete verticalJump;
                return new Node(nx, ny);
            }

            // 检查对角线方向是否有强迫邻居
            if (hasForcedNeighbor(nx, ny, dx, dy)) {
                return new Node(nx, ny);
            }

            // 继续对角线跳跃
            nx += dx;
            ny += dy;
        }

        return nullptr;
    }

    // 获取移动方向
    void getDirection(Node* from, Node* to, int& dx, int& dy) {
        if (from->x == to->x) {
            dx = 0;
            dy = (to->y > from->y) ? 1 : -1;
        }
        else if (from->y == to->y) {
            dx = (to->x > from->x) ? 1 : -1;
            dy = 0;
        }
        else {
            dx = (to->x > from->x) ? 1 : -1;
            dy = (to->y > from->y) ? 1 : -1;
        }
    }

    // 获取后继跳点
    vector<Node*> getSuccessors(Node* current, Node* goal) {
        vector<Node*> successors;

        if (current->parent == nullptr) {
            // 起点，检查所有8个方向
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;

                    Node* jumpPoint = nullptr;
                    if (dx != 0 && dy != 0) {
                        jumpPoint = jumpDiagonal(current->x, current->y, dx, dy, goal);
                    }
                    else {
                        jumpPoint = jumpStraight(current->x, current->y, dx, dy, goal);
                    }

                    if (jumpPoint) {
                        jumpPoint->parent = current;
                        successors.push_back(jumpPoint);
                    }
                }
            }
        }
        else {
            // 获取移动方向
            int dx, dy;
            getDirection(current->parent, current, dx, dy);

            // 对角线移动
            if (dx != 0 && dy != 0) {
                // 水平方向
                if (isValid(current->x + dx, current->y)) {
                    Node* jp = jumpStraight(current->x, current->y, dx, 0, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

                // 垂直方向
                if (isValid(current->x, current->y + dy)) {
                    Node* jp = jumpStraight(current->x, current->y, 0, dy, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

                // 对角线方向
                if (isValid(current->x + dx, current->y + dy)) {
                    Node* jp = jumpDiagonal(current->x, current->y, dx, dy, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

                // 对角线的强迫邻居
                if (isObstacle(current->x - dx, current->y) && isValid(current->x - dx, current->y + dy)) {
                    Node* jp = jumpDiagonal(current->x, current->y, -dx, dy, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

                if (isObstacle(current->x, current->y - dy) && isValid(current->x + dx, current->y - dy)) {
                    Node* jp = jumpDiagonal(current->x, current->y, dx, -dy, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

            }
            // 直线移动
            else {
                // 主要方向
                if (isValid(current->x + dx, current->y + dy)) {
                    Node* jp = jumpStraight(current->x, current->y, dx, dy, goal);
                    if (jp) {
                        jp->parent = current;
                        successors.push_back(jp);
                    }
                }

                // 检查强迫邻居导致的斜向搜索
                if (dx != 0) {
                    if (isObstacle(current->x, current->y + 1) && isValid(current->x + dx, current->y + 1)) {
                        Node* jp = jumpDiagonal(current->x, current->y, dx, 1, goal);
                        if (jp) {
                            jp->parent = current;
                            successors.push_back(jp);
                        }
                    }
                    if (isObstacle(current->x, current->y - 1) && isValid(current->x + dx, current->y - 1)) {
                        Node* jp = jumpDiagonal(current->x, current->y, dx, -1, goal);
                        if (jp) {
                            jp->parent = current;
                            successors.push_back(jp);
                        }
                    }
                }
                else {
                    if (isObstacle(current->x + 1, current->y) && isValid(current->x + 1, current->y + dy)) {
                        Node* jp = jumpDiagonal(current->x, current->y, 1, dy, goal);
                        if (jp) {
                            jp->parent = current;
                            successors.push_back(jp);
                        }
                    }
                    if (isObstacle(current->x - 1, current->y) && isValid(current->x - 1, current->y + dy)) {
                        Node* jp = jumpDiagonal(current->x, current->y, -1, dy, goal);
                        if (jp) {
                            jp->parent = current;
                            successors.push_back(jp);
                        }
                    }
                }
            }
        }

        return successors;
    }

public:
    JPS(vector<vector<int>> grid) : grid(grid) {
        height = grid.size();
        width = grid[0].size();
    }

    vector<Node*> findPath(int startX, int startY, int endX, int endY) {
        Node* start = new Node(startX, startY);
        Node* goal = new Node(endX, endY);

        // 检查起点和终点是否有效
        if (!isValid(startX, startY) || !isValid(endX, endY)) {
            return {};
        }

        auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
        priority_queue<Node*, vector<Node*>, decltype(cmp)> openList(cmp);
        unordered_map<Node, Node*> allNodes;

        start->g = 0;
        start->h = heuristic(start->x, start->y, goal->x, goal->y);
        start->f = start->g + start->h;

        openList.push(start);
        allNodes[*start] = start;

        while (!openList.empty()) {
            Node* current = openList.top();
            openList.pop();

            // 到达目标
            if (current->x == goal->x && current->y == goal->y) {
                vector<Node*> path;
                while (current) {
                    path.push_back(current);
                    current = current->parent;
                }
                reverse(path.begin(), path.end());
                return path;
            }

            // 获取后继跳点
            vector<Node*> successors = getSuccessors(current, goal);

            for (Node* successor : successors) {
                float newG = current->g +
                    sqrt(pow(successor->x - current->x, 2) + pow(successor->y - current->y, 2));

                if (allNodes.find(*successor) == allNodes.end() || newG < allNodes[*successor]->g) {
                    successor->g = newG;
                    successor->h = heuristic(successor->x, successor->y, goal->x, goal->y);
                    successor->f = successor->g + successor->h;
                    successor->parent = current;

                    if (allNodes.find(*successor) == allNodes.end()) {
                        Node* newNode = new Node(successor->x, successor->y, current);
                        newNode->g = successor->g;
                        newNode->h = successor->h;
                        newNode->f = successor->f;
                        allNodes[*newNode] = newNode;
                        openList.push(newNode);
                    }
                }
                delete successor;
            }
        }

        return {};
    }
};