#pragma once
// grid.hpp - the world the algorithms search in.
//
// A node is just an int: index = row * cols + col. Using one flat vector
// (instead of vector<vector<>>) keeps memory contiguous and makes it trivial
// to store per-node data (parent, distance, ...) in parallel arrays.

#include <cstdint>
#include <vector>

enum class Cell : std::uint8_t {
    Empty,   // cost 1 to enter
    Wall,    // impassable
    Swamp    // cost kSwampCost to enter (makes weighted algorithms interesting)
};

// What the *search* currently thinks about a node. Purely for visualization.
enum class State : std::uint8_t {
    Unvisited,
    InFrontier,  // discovered, waiting to be expanded
    Visited,     // expanded (removed from the frontier)
    Path         // part of the final path
};

constexpr int kSwampCost = 5;

struct Grid {
    int rows;
    int cols;
    std::vector<Cell>  cells;
    std::vector<State> states;
    int start = 0;
    int goal  = 0;

    Grid(int r, int c)
        : rows(r), cols(c),
          cells(static_cast<std::size_t>(r) * c, Cell::Empty),
          states(static_cast<std::size_t>(r) * c, State::Unvisited) {
        resetEndpoints();
    }

    int  idx(int r, int c) const { return r * cols + c; }
    int  rowOf(int i)      const { return i / cols; }
    int  colOf(int i)      const { return i % cols; }
    int  size()            const { return rows * cols; }

    bool inBounds(int r, int c) const {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }
    bool passable(int i) const { return cells[i] != Cell::Wall; }

    // Cost of *entering* node i.
    int cost(int i) const { return cells[i] == Cell::Swamp ? kSwampCost : 1; }

    void resetEndpoints() {
        start = idx(rows / 2, cols / 4);
        goal  = idx(rows / 2, 3 * cols / 4);
    }

    // Forget everything the search did, keep walls/terrain.
    void clearSearch() {
        for (auto& s : states) s = State::Unvisited;
    }

    // Back to an empty board.
    void clearAll() {
        for (auto& c : cells) c = Cell::Empty;
        clearSearch();
        resetEndpoints();
    }
};

// Calls f(neighborIndex) for each passable 4-neighbour of node `id`.
// Order: up, right, down, left.
template <class F>
void forEachNeighbor(const Grid& g, int id, F&& f) {
    static constexpr int dr[4] = {-1, 0, 1, 0};
    static constexpr int dc[4] = { 0, 1, 0, -1};
    const int r = g.rowOf(id);
    const int c = g.colOf(id);
    for (int k = 0; k < 4; ++k) {
        const int nr = r + dr[k];
        const int nc = c + dc[k];
        if (!g.inBounds(nr, nc)) continue;
        const int n = g.idx(nr, nc);
        if (!g.passable(n)) continue;
        f(n);
    }
}
