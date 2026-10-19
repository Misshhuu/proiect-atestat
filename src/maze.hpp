#pragma once
// maze.hpp - helpers that fill a Grid with interesting test scenarios.

#include "grid.hpp"

#include <algorithm>
#include <random>
#include <vector>

// Randomized depth-first "recursive backtracker" maze, iterative version.
//
// Works on a lattice of odd coordinates: cells at (odd, odd) are rooms, cells
// with exactly one odd coordinate are the walls between two rooms. We start
// with everything walled, then repeatedly carve into an unvisited neighbouring
// room two cells away (removing the wall between). This produces a "perfect"
// maze (exactly one path between any two rooms), so we afterwards knock out a
// fraction of walls to create loops - otherwise every algorithm would find the
// same single path and the comparison would be boring.
//
// Requires odd rows and cols (>= 5).
inline void generateMaze(Grid& g, std::mt19937& rng, double loopChance = 0.08) {
    std::fill(g.cells.begin(), g.cells.end(), Cell::Wall);

    static constexpr int dr[4] = {-2, 2, 0, 0};
    static constexpr int dc[4] = { 0, 0, -2, 2};

    std::vector<int> stack;
    g.cells[g.idx(1, 1)] = Cell::Empty;
    stack.push_back(g.idx(1, 1));

    while (!stack.empty()) {
        const int cur = stack.back();
        const int r = g.rowOf(cur);
        const int c = g.colOf(cur);

        int order[4] = {0, 1, 2, 3};
        std::shuffle(order, order + 4, rng);

        bool carved = false;
        for (int k : order) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (nr < 1 || nr > g.rows - 2 || nc < 1 || nc > g.cols - 2) continue;
            if (g.cells[g.idx(nr, nc)] != Cell::Wall) continue;

            g.cells[g.idx(r + dr[k] / 2, c + dc[k] / 2)] = Cell::Empty;  // wall between
            g.cells[g.idx(nr, nc)] = Cell::Empty;                        // new room
            stack.push_back(g.idx(nr, nc));
            carved = true;
            break;
        }
        if (!carved) stack.pop_back();
    }

    // Add loops: open some interior walls that separate two open cells.
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    for (int r = 1; r < g.rows - 1; ++r) {
        for (int c = 1; c < g.cols - 1; ++c) {
            if ((r % 2) == (c % 2)) continue;              // rooms / pillars, not walls between rooms
            if (g.cells[g.idx(r, c)] != Cell::Wall) continue;
            if (coin(rng) < loopChance) g.cells[g.idx(r, c)] = Cell::Empty;
        }
    }

    g.start = g.idx(1, 1);
    g.goal  = g.idx(g.rows - 2, g.cols - 2);
    g.clearSearch();
}

// Turns a random fraction of empty cells into swamp (cost kSwampCost).
inline void sprinkleSwamp(Grid& g, std::mt19937& rng, double probability = 0.25) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    for (int i = 0; i < g.size(); ++i) {
        if (i == g.start || i == g.goal) continue;
        if (g.cells[i] == Cell::Empty && coin(rng) < probability) g.cells[i] = Cell::Swamp;
    }
    g.clearSearch();
}
