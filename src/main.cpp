#include "AStar_improve.hpp"
#include "matplotlibcpp.h"
#include <random>
using namespace std;
namespace plt = matplotlibcpp;

pair<int, int> generate_my_random(int min, int max)
{
    static random_device rd;
    static mt19937 gen(rd());

    uniform_int_distribution<int> distrib(min, max);
    return make_pair<int, int>(distrib(gen), distrib(gen));
}

int main()
{

    int map_size = 33;
    Point start(0, 0), end(32, 32);

    vector<vector<Point>> map(map_size, vector<Point>(map_size));
    int rows = map.size();
    int cols = map[0].size();

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            map[i][j].set_x(i);
            map[i][j].set_y(j);
        }
    }
    vector<int> block_x;
    vector<int> block_y;
    for (int i = 0; i < 600; ++i)
    {
        pair<int, int> temp = generate_my_random(0, 32);
        map[temp.first][temp.second].setBlocked(true);
        block_x.push_back(temp.first);
        block_y.push_back(temp.second);
    }
    plt::scatter(block_x, block_y, 2);
    // plt::scatter_colored(block_x,block_y)
    map[0][0].setBlocked(false);
    map[32][32].setBlocked(false);

    AStar astar(map);
    vector<Point> Path;
    vector<int> Path_x;
    vector<int> Path_y;
    bool find = astar.findPath(start, end, Path);
    if (find)
    {
        for (auto it = Path.end() - 1; it != Path.begin() - 1; --it)
        {
            cout << it->x() << "," << it->y() << endl;
            Path_x.push_back(static_cast<int>(it->x()));
            Path_y.push_back(static_cast<int>(it->y()));
        }
        plt::scatter(Path_x, Path_y, 2);
        plt::show();
    }
    else
    {
        cout << "no find path" << endl;
    }
}