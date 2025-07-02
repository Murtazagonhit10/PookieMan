#ifndef ASTAR_H
#define ASTAR_H
// ─────────────────────────────────────────────────────────────────────────────
//  Very small 4-direction A* helper for Pac-Man
// ─────────────────────────────────────────────────────────────────────────────
#include <stack>
#include <vector>
#include <cmath>
#include <limits>
#include<string>

// ─── Maze dimensions (classic 28×31 grid) ───────────────────────────────────
constexpr int MAPSIZEX = 28;   // columns
constexpr int MAPSIZEY = 31;   // rows

// ─── Tiny POD structs ───────────────────────────────────────────────────────
struct Pair { int x, y; };           // a grid coordinate (column,row)
inline bool operator<(Pair a, Pair b) {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}
struct cell
{
    int    parent_x = -1, parent_y = -1;
    double f = 0, g = 0, h = 0;      // A* cost scores
};

// ─── Public helpers you’ll call from Ghost AI etc. ──────────────────────────
std::stack<Pair> aStarSearch(const std::vector<std::string>& grid,
    Pair src, Pair dest);

#endif // ASTAR_H