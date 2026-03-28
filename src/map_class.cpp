#include "mission_header.h"


/************************************************************************
Map类实现
*************************************************************************/
Map::Map(float _width, float _length, float _cellsize)
    : width(_width), length(_length), CellSize(_cellsize)
{
    Xnum = (ceil(length / CellSize) + 2);
    Ynum = (ceil(width / CellSize) + 2);
    delta_y = static_cast<int>(7.5 / CellSize);
    delta_x = static_cast<int>(1.0 / CellSize);
    Grid = new int *[Xnum];
    for (int i = 0; i < Xnum; ++i)
    {
        Grid[i] = new int[Ynum];
        memset(Grid[i], 0, Ynum * sizeof(int));
    }
    // 建墙,安全措施
    // memset(Grid[0],1,Ynum*sizeof(int));
    // memset(Grid[Xnum-1],1,Ynum*sizeof(int));
    for (int i = 0; i < Ynum; ++i)
    {
        Grid[0][i] = 1;
        Grid[Xnum - 1][i] = 1;
    }
    for (int i = 0; i < Xnum; ++i)
    {
        Grid[i][0] = 1;
        Grid[i][Ynum - 1] = 1;
    }
}

Map::~Map()
{
    for (int i = 0; i < Xnum; ++i)
    {
        delete[] Grid[i];
    }
    delete[] Grid;
    Grid = 0;
}

GridPoint Map::PointToGridPoint(const Point &P)
{
    return {static_cast<int>(ceil((P.x) / CellSize)) + delta_x, static_cast<int>(ceil((P.y) / CellSize)) + delta_y};
}

// 新坐标系下的pointtogrid
GridPoint Map::PointToGridPoint_World(const Point &P)
{
    return {static_cast<int>(ceil((P.x) / CellSize)), static_cast<int>(ceil((P.y) / CellSize))};
}

GridPoint Map::PointToGridPoint(const Point &P, int judge)
{
    return {static_cast<int>(round((P.x) / CellSize)) + delta_x, static_cast<int>(round((P.y) / CellSize)) + delta_y};
}

Point Map::GridPointToPoint(const GridPoint &p)
{
    return {(1.0 * p.x - 0.5 - delta_x) * CellSize, (1.0 * p.y - 0.5 - delta_y) * CellSize};
}

float Map::Manhattan(const GridPoint &start, const GridPoint &end)
{
    return abs(end.x - start.x) + abs(end.y - start.y);
    ;
}

// 非真实距离，为格栅距离
float Map::Euclidean(const GridPoint &p1, const GridPoint &p2)
{
    return sqrt(((p1.x - p2.x) * (p1.x - p2.x) * 1.0 + (p1.y - p2.y) * (p1.y - p2.y)) * 1.0);
}

bool Map::qualify(const GridPoint &p)
{
    if (p.x < M.Xnum && p.x >= 0 && p.y < M.Ynum && p.y >= 0)
        return true;
    return false;
}

void Map::update_map(int EXPAND_NUM)
{
    for (auto it = Obs.all_obs.begin(); it != Obs.all_obs.end(); ++it)
    {
        for (int i = it->start; i <= it->end; ++i)
        {
            GridPoint p = PointToGridPoint(rotation_yaw(yaw, AngleToPoint(i)));
            if (qualify({p.x, p.y}))
                Grid[p.x][p.y] = 1;
            // 扩张操作
            for (int j = 0; j < EXPAND_NUM; ++j)
            {
                if (qualify({p.x + dx[j], p.y + dy[j]}))
                    Grid[p.x + dx[j]][p.y + dy[j]] = 1;
            }
        }
    }
}

GridPoint Map::safe_Gpoint(const GridPoint &now, const GridPoint &end, int EXPAND_NUM, bool Fusion)
{
    GridPoint FinalPoint(0, 0, -1, FLOAT_MAX);
    int count = Fusion ? 0 : 1;
    for (int i = 0, x = 0, y = 0; i < EXPAND_NUM; ++i)
    {
        x = now.x + dx[i];
        y = now.y + dy[i];
        if (qualify({x, y}) && Grid[x][y] == 0)
        {
            if (Fusion)
            {
                FinalPoint += {x, y};
                ++count;
            }
            else
            {
                float new_h = Euclidean({x, y}, end);
                if (new_h < FinalPoint.h) // 找离终点最近的点
                    FinalPoint = {x, y, 0, new_h};
            }
        }
    }
    if (Fusion)
    {
        FinalPoint += now;
        ++count;
    }
    FinalPoint /= count;
    return FinalPoint;
}

bool Map::Astar(const GridPoint &start, const GridPoint &end)
{
    // 优先队列
    std::priority_queue<GridPoint, std::vector<GridPoint>, CompareF> min_heap;
    // 父节点数组,parent[x][y].x代表（x，y）位置的父节点x坐标，y同理
    GridPoint **Parent;
    Parent = new GridPoint *[Xnum];
    for (int i = 0; i < Xnum; ++i)
    {
        Parent[i] = new GridPoint[Ynum];
        for (int j = 0; j < Ynum; ++j)
        {
            Parent[i][j].g = -1; //-1表示未被访问
        }
    }
    // 初始化
    Parent[start.x][start.y].g = 0; // 非-1为已访问,注意此处不是表示父节点的gh值，而是当前坐标下的gh值
    Parent[start.x][start.y].h = Euclidean(start, end);
    min_heap.push(start);
    //
    GridPoint Gp;
    while (!min_heap.empty())
    {
        Gp = min_heap.top();
        min_heap.pop();
        if (Gp == end)
        {
            ROS_INFO("找到终点");
            break;
        }
        for (int i = 0, X = 0, Y = 0; i < EXPAND_ONE; ++i)
        {
            // 新节点的坐标
            X = Gp.x + Dx[i];
            Y = Gp.y + Dy[i];
            if (qualify({X, Y}) && Grid[X][Y] == 0 && Parent[X][Y].g == -1)
            {
                Parent[X][Y].g = Parent[Gp.x][Gp.y].g + weight[i];
                Parent[X][Y].h = Euclidean({X, Y}, end);
                Parent[X][Y].x = Gp.x;
                Parent[X][Y].y = Gp.y;
                min_heap.push({X, Y, Parent[X][Y].g, Parent[X][Y].h});
            }
        }
    }
    int count = 0;
    int x = Gp.x;
    int y = Gp.y;
    bool find = true;
    // 如果找到终点
    if (Gp == end)
    {
        while (Parent[x][y].x != start.x || Parent[x][y].y != start.y)
        {
            if (++count > 1000)
                break;
            int px = Parent[x][y].x;
            int py = Parent[x][y].y;
            x = px;
            y = py;
        }
        if (count <= 1000)
        {
            Path = std::vector<GridPoint>();
            x = Gp.x;
            y = Gp.y;
            while (Parent[x][y].x != start.x || Parent[x][y].y != start.y)
            {
                int px = Parent[x][y].x;
                int py = Parent[x][y].y;
                x = px;
                y = py;
                Path.push_back({x, y});
            }
        }
    }
    if (!(Gp == end) || count > 1000)
    {
        find = false;
    }

    for (int i = 0; i < Xnum; ++i)
        delete[] Parent[i];
    delete[] Parent;
    return find;
}