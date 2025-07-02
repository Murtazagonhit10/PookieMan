#include "AStar.h"
#include <set>

// ─── Local helpers ──────────────────────────────────────────────────────────
static bool isValid(int x, int y)
{
    return (x >= 0 && x < MAPSIZEX &&
        y >= 0 && y < MAPSIZEY);
}

// treat dots, pellets, and empty space as walkable
static bool isUnBlocked(const std::vector<std::string>& grid, int x, int y)
{
    char c = grid[y][x];
    return c == '.' || c == 'o' || c == ' ';
}

static bool isDestination(int x, int y, Pair dest)
{
    return x == dest.x && y == dest.y;
}

static double hValue(int x, int y, Pair dest)
{
    return std::sqrt((x - dest.x) * (x - dest.x) +
        (y - dest.y) * (y - dest.y));
}

static std::stack<Pair> tracePath(cell cellDetails[][MAPSIZEX], Pair dest)
{
    std::stack<Pair> path;
    int x = dest.x, y = dest.y;

    while (!(cellDetails[y][x].parent_x == x &&
        cellDetails[y][x].parent_y == y))
    {
        path.push({ x, y });
        int px = cellDetails[y][x].parent_x;
        int py = cellDetails[y][x].parent_y;
        x = px; y = py;
    }
    path.push({ x, y });                    // push the start cell
    return path;
}

// ─── Main entry: 4-neighbour A* ─────────────────────────────────────────────
std::stack<Pair> aStarSearch(const std::vector<std::string>& grid,
    Pair src, Pair dest)
{
    std::stack<Pair> empty;

    // sanity checks
    if (!isValid(src.x, src.y) || !isValid(dest.x, dest.y)) return empty;
    if (!isUnBlocked(grid, src.x, src.y) ||
        !isUnBlocked(grid, dest.x, dest.y))                 return empty;
    if (isDestination(src.x, src.y, dest)) { empty.push(src); return empty; }

    bool closed[MAPSIZEY][MAPSIZEX] = { {false} };
    cell cdet[MAPSIZEY][MAPSIZEX];

    // initialise start node
    int sx = src.x, sy = src.y;
    cdet[sy][sx] = { sx, sy, 0.0, 0.0, 0.0 };

    using openPair = std::pair<double, Pair>;     // (f-score, cell)
    std::set<openPair> open;
    open.insert({ 0.0, {sx, sy} });

    constexpr int dx[4] = { 1,-1, 0, 0 };
    constexpr int dy[4] = { 0, 0, 1,-1 };

    while (!open.empty())
    {
        Pair p = open.begin()->second;           // lowest f
        open.erase(open.begin());

        int x = p.x, y = p.y;
        closed[y][x] = true;

        // check the four neighbours
        for (int k = 0; k < 4; ++k)
        {
            int nx = x + dx[k], ny = y + dy[k];
            if (!isValid(nx, ny)) continue;

            // destination?
            if (isDestination(nx, ny, dest))
            {
                cdet[ny][nx].parent_x = x;
                cdet[ny][nx].parent_y = y;
                return tracePath(cdet, dest);
            }

            if (closed[ny][nx] || !isUnBlocked(grid, nx, ny)) continue;

            double gNew = cdet[y][x].g + 1.0;
            double hNew = hValue(nx, ny, dest);
            double fNew = gNew + hNew;

            if (std::isinf(cdet[ny][nx].f) || cdet[ny][nx].f > fNew)
            {
                open.insert({ fNew, {nx, ny} });
                cdet[ny][nx] = { x, y, fNew, gNew, hNew };
            }
        }
    }
    return empty;     // no path found
}