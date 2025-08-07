#include <queue>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <functional>
#include <iostream>
enum InspireMethod
{
    Manhattan,
    Euclidean
};

class Point
{
private:
    double x_, y_;
    double score_ = 0;
    double dis_to_start_ = 0;
    double dis_to_end_ = 0;
    bool closed_ = false;
    bool blocked_ = false;
    Point *parent_ = nullptr;

public:
    Point(double x, double y) : x_(x), y_(y) {}
    Point() {};

    // 访问方法
    double x() const { return x_; }
    double y() const { return y_; }
    double score() const { return score_; }
    Point *parent() const { return parent_; }
    bool isClosed() const { return closed_; }
    bool isBlocked() const { return blocked_; }
    double dis_to_start() const { return dis_to_start_; };

    // 设置方法
    void setBlocked(bool blocked) { blocked_ = blocked; }
    void setParent(Point *parent) { parent_ = parent; }
    void close() { closed_ = true; }
    void set_x(double x) { x_ = x; };
    void set_y(double y) { y_ = y; };

    // 启发函数
    void calculateHeuristic(const Point &end, InspireMethod method)
    {
        switch (method)
        {
        case InspireMethod::Manhattan:
            dis_to_end_ = std::abs(x_ - end.x()) + std::abs(y_ - end.y());
            break;
        case InspireMethod::Euclidean:
            dis_to_end_ = std::sqrt((x_ - end.x()) * (x_ - end.x()) +
                                    (y_ - end.y()) * (y_ - end.y()));
            break;
        }
    }

    void updateScore(double cost)
    {
        dis_to_start_ += cost;
        score_ = dis_to_start_ + dis_to_end_;
    }

    bool operator==(const Point &other) const
    {
        return x_ == other.x_ && y_ == other.y_;
    }
};

class AStar
{
public:
    using Grid = std::vector<std::vector<Point>>;

    AStar(Grid &map) : map_(map)
    {
        initializeDirections();
    }

    bool findPath(Point &start, Point &end, std::vector<Point> &path,
                  InspireMethod method = InspireMethod::Euclidean);

private:
    void initializeDirections()
    {
        directions_ = {{-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}};
    }

    struct PointCompare
    {
        bool operator()(const Point *a, const Point *b) const
        {
            return a->score() > b->score();
        }
    };

    Grid &map_;
    std::priority_queue<Point *, std::vector<Point *>, PointCompare> open_queue_;
    std::unordered_set<Point *> open_set_;
    std::vector<std::pair<int, int>> directions_;
};

bool AStar::findPath(Point &start, Point &end, std::vector<Point> &path,
                     InspireMethod method)
{

    start.calculateHeuristic(end, method);
    start.updateScore(0); // start节点的dis_to_start为0

    if (start.isBlocked())
        return false;

    open_queue_.push(&start);
    open_set_.insert(&start);

    const int rows = map_.size();
    const int cols = map_[0].size();

    while (!open_queue_.empty())
    {
        Point *current = open_queue_.top();
        open_queue_.pop();
        open_set_.erase(current);
#ifdef DEBUG
        std::cout << "current_point:" << current->x() << "," << current->y() << ",score=" << current->score() << ",Position:" << current << ",closed=" << static_cast<double>(current->isClosed()) << std::endl;
#endif
        if (*current == end)
        {
            // 回溯路径
            Point *pathNode = current;
            while (pathNode)
            {
                path.push_back(*pathNode);
                pathNode = pathNode->parent();
            }
            return true;
        }

        current->close();

        for (const auto &[dx, dy] : directions_)
        {
            int new_x = static_cast<int>(current->x()) + dx;
            int new_y = static_cast<int>(current->y()) + dy;

            // 边界检查
            if (new_x < 0 || new_x >= rows || new_y < 0 || new_y >= cols)
                continue;

            Point *neighbor = &map_[new_x][new_y];

            // 节点有效性检查
            if (neighbor->isClosed() || neighbor->isBlocked() || open_set_.find(neighbor) != open_set_.end())
                continue;

            // 计算移动成本 (对角线成本更高)
            double cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
            double tentative_g_score = current->dis_to_start() + cost;

            // 新路径是否更优

            neighbor->setParent(current);
            neighbor->calculateHeuristic(end, method);
            neighbor->updateScore(cost);

            open_queue_.push(neighbor);
            open_set_.insert(neighbor);
        }
    }

    return false; // 路径未找到
}