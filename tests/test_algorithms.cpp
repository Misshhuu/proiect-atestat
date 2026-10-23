

#include "../src/algorithms.hpp"
#include "../src/grid.hpp"
#include "../src/maze.hpp"

#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

#define CHECK(cond, msg)                                                          \
    do {                                                                          \
        if (!(cond)) {                                                            \
            ++g_failures;                                                         \
            if (g_failures <= 20) std::printf("FAIL: %s  [%s]\n", msg, #cond);    \
        }                                                                         \
    } while (0)


int referenceCost(const Grid& g) {
    const int n = g.size();
    std::vector<int>  dist(n, INT_MAX);
    std::vector<bool> done(n, false);
    dist[g.start] = 0;
    for (int iter = 0; iter < n; ++iter) {
        int u = -1;
        for (int i = 0; i < n; ++i)
            if (!done[i] && dist[i] != INT_MAX && (u == -1 || dist[i] < dist[u])) u = i;
        if (u == -1) break;
        done[u] = true;
        forEachNeighbor(g, u, [&](int v) {
            if (dist[u] + g.cost(v) < dist[v]) dist[v] = dist[u] + g.cost(v);
        });
    }
    return dist[g.goal] == INT_MAX ? -1 : dist[g.goal];
}


int referenceMoves(const Grid& g) {
    std::vector<int> d(g.size(), -1);
    std::vector<int> q{g.start};
    d[g.start] = 0;
    for (std::size_t h = 0; h < q.size(); ++h) {
        const int u = q[h];
        forEachNeighbor(g, u, [&](int v) {
            if (d[v] == -1) { d[v] = d[u] + 1; q.push_back(v); }
        });
    }
    return d[g.goal];
}

bool adjacent(const Grid& g, int a, int b) {
    return std::abs(g.rowOf(a) - g.rowOf(b)) + std::abs(g.colOf(a) - g.colOf(b)) == 1;
}


struct Outcome {
    bool found = false;
    int  moves = -1;
    int  cost  = -1;
    int  visited = 0;
    bool pathValid = true;
};

Outcome run(int algoIndex, Grid g) {
    auto algo = makeAlgorithm(algoIndex);
    algo->reset(g);
    int guard = g.size() * 20 + 100;
    while (algo->step(g) && --guard > 0) {}

    Outcome o;
    CHECK(algo->finished(), "algorithm must terminate");
    o.found   = algo->found();
    o.visited = algo->visited();
    if (!o.found) return o;

    const auto& p = algo->path();
    o.moves = static_cast<int>(p.size()) - 1;
    o.cost  = algo->pathCost();

    if (p.empty() || p.front() != g.start || p.back() != g.goal) o.pathValid = false;
    int recomputed = 0;
    for (std::size_t i = 1; i < p.size(); ++i) {
        if (!adjacent(g, p[i - 1], p[i]) || !g.passable(p[i])) o.pathValid = false;
        recomputed += g.cost(p[i]);
    }
    if (recomputed != o.cost) o.pathValid = false;
    return o;
}

Grid randomGrid(std::mt19937& rng) {
    std::uniform_int_distribution<int> dim(2, 24);
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    const double wallP  = std::uniform_real_distribution<double>(0.0, 0.45)(rng);
    const double swampP = std::uniform_real_distribution<double>(0.0, 0.5)(rng);

    Grid g(dim(rng), dim(rng));
    for (int i = 0; i < g.size(); ++i) {
        const double x = coin(rng);
        g.cells[i] = x < wallP ? Cell::Wall : (x < wallP + swampP ? Cell::Swamp : Cell::Empty);
    }
    std::uniform_int_distribution<int> any(0, g.size() - 1);
    g.start = any(rng);
    do { g.goal = any(rng); } while (g.size() > 1 && g.goal == g.start);
    g.cells[g.start] = g.cells[g.goal] = Cell::Empty;
    return g;
}

void testOne(const Grid& g, long long& astarVisited, long long& dijkVisited, int& reachable) {
    const int refCost  = referenceCost(g);
    const int refMoves = referenceMoves(g);

    const Outcome bfs = run(0, g), dfs = run(1, g), dij = run(2, g), ast = run(3, g);

    CHECK(bfs.found == (refCost != -1), "BFS existence matches reference");
    CHECK(dfs.found == (refCost != -1), "DFS existence matches reference");
    CHECK(dij.found == (refCost != -1), "Dijkstra existence matches reference");
    CHECK(ast.found == (refCost != -1), "A* existence matches reference");

    if (refCost == -1) return;
    ++reachable;

    CHECK(bfs.pathValid && dfs.pathValid && dij.pathValid && ast.pathValid, "all paths valid");
    CHECK(bfs.moves == refMoves, "BFS = fewest moves");
    CHECK(dij.cost == refCost,   "Dijkstra = cheapest cost");
    CHECK(ast.cost == refCost,   "A* = cheapest cost");
    CHECK(dfs.cost >= refCost,   "DFS never beats the optimum");
    CHECK(bfs.cost >= refCost,   "BFS never beats the optimum");

    astarVisited += ast.visited;
    dijkVisited  += dij.visited;
}

}  

int main() {
    std::mt19937 rng(12345);
    long long astarVisited = 0, dijkVisited = 0;
    int reachable = 0;
    const int kRandomTests = 4000;

    for (int t = 0; t < kRandomTests; ++t)
        testOne(randomGrid(rng), astarVisited, dijkVisited, reachable);

    for (int t = 0; t < 200; ++t) {
        Grid g(31, 45);
        generateMaze(g, rng);
        if (t % 2) sprinkleSwamp(g, rng, 0.3);
        testOne(g, astarVisited, dijkVisited, reachable);
    }

    {
        Grid g(5, 5);
        g.goal = g.start;
        const Outcome o = run(3, g);
        CHECK(o.found && o.moves == 0 && o.cost == 0, "start == goal gives empty path");
    }

    std::printf("grids with a path: %d\n", reachable);
    std::printf("total expansions  Dijkstra: %lld   A*: %lld\n", dijkVisited, astarVisited);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    else                 std::printf("%d CHECK(S) FAILED\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
