#include "matplotlibcpp.h"
#include <queue>
#include <cmath>
#include <vector>
#include <unordered_set>
using namespace std;

enum inspire_method
{
    Manhattan,
    Euclidean
};
struct Point
{
    double x, y;
    double score = 0;
    double dis_to_start = 0;
    double dis_to_end = 0;
    bool closed = false;
    bool blocked = false;
    Point *Parent = nullptr;

    Point(int x, int y) : x(x), y(y) {};

    Point() {};

    bool operator<(const Point *other) const
    {
        return score < other->score;
    }
    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point &other) const
    {
        return x != other.x || y != other.y;
    }

    void inspire_func(Point end, inspire_method method = inspire_method::Euclidean)
    {
        switch (method)
        {
        case inspire_method::Manhattan:
            dis_to_end = abs(x - end.x) + abs(y - end.y);
            break;

        case inspire_method::Euclidean:
            dis_to_end = sqrt((x - end.x) * (x - end.x) + (y - end.y) * (y - end.y));
            break;
        }
    }
};

class AStarException : public std::runtime_error
{
public:
    using runtime_error::runtime_error;
};

class AStar
{
public:
    AStar(const vector<vector<Point>> map) : map(map)
    {
        if (map.empty())
        {
            throw AStarException("ap is empty");
        }
    };
    ~AStar() {};

    vector<vector<Point>> map;
    priority_queue<Point *> open_queue;
    unordered_set<Point *> open_set;
    bool findPath(Point &start, Point &end, vector<Point> &Path);
};
bool AStar::findPath(Point &start, Point &end, vector<Point> &Path)
{
    start.inspire_func(end);
    start.score = start.dis_to_end;
    if (start.blocked)
    {
        return false;
    }
    open_queue.push(&start);
    open_set.insert(open_queue.top());

    while (!open_queue.empty())
    {
        Point *current = open_queue.top();
#ifdef DEBUG
        cout << "current_point:" << current->x << "," << current->y << ",score=" << current->score << ",Position:" << current << ",closed=" << static_cast<double>(current->closed) << endl;
#endif
        if (*current == end)
        {
            while (*current != start)
            {
                Path.push_back(*current);
                if (current->Parent)
                    current = current->Parent;
                else
                {
                    cout << "path back err!" << endl;
                    return false;
                }
            }
            Path.push_back(*current);
            return true;
        }
        else
        {
            current->closed = true;
            if (open_set.find(open_queue.top()) != open_set.end())
                open_set.erase(open_queue.top());
            else
            {
                cout << "open_set error" << endl;
                return false;
            }
            open_queue.pop();

            const vector<vector<int>> dir = {{-1, -1},
                                            {-1, 0},
                                            {-1, 1},
                                            {0, 1},
                                            {1, 1},
                                            {1, 0},
                                            {1, -1},
                                            {0, -1}};
            
            if (map.empty())
            {
                cout << "map is empty" << endl;
                return false;
            }
            int rows = map.size();
            int cols = map[0].size();

            for (const auto& d:dir)
            {
                if ((current->x + d[0]) >= 0 &&
                    (current->x + d[0]) < rows &&
                    (current->y + d[1]) >= 0 &&
                    (current->y + d[1]) < cols)
                {
                    Point *next_point = &map[static_cast<int>(current->x + d[0])][static_cast<int>(current->y + d[1])];
                    if (!next_point->closed && !next_point->blocked && open_set.find(next_point) == open_set.end())
                    {
                        next_point->Parent = current;
                        next_point->inspire_func(end);
                        next_point->dis_to_start = current->dis_to_start + 1;
                        next_point->score = next_point->dis_to_start + next_point->dis_to_end;
                        open_queue.push(next_point);
                        open_set.insert(next_point);
#ifdef DEBUG
                        cout << "add_point:x=" << next_point->x << ",y=" << next_point->y << endl;
#endif
                    }
                }
            }
        }
    }
}
