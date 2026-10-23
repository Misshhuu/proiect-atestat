#pragma once

#include "grid.hpp"

#include <algorithm>
#include <random>
#include <vector>


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

            g.cells[g.idx(r + dr[k] / 2, c + dc[k] / 2)] = Cell::Empty; 
            g.cells[g.idx(nr, nc)] = Cell::Empty;                       
            stack.push_back(g.idx(nr, nc));
            carved = true;
            break;
        }
        if (!carved) stack.pop_back();
    }


    std::uniform_real_distribution<double> coin(0.0, 1.0);
    for (int r = 1; r < g.rows - 1; ++r) {
        for (int c = 1; c < g.cols - 1; ++c) {
            if ((r % 2) == (c % 2)) continue;             
            if (g.cells[g.idx(r, c)] != Cell::Wall) continue;
            if (coin(rng) < loopChance) g.cells[g.idx(r, c)] = Cell::Empty;
        }
    }

    g.start = g.idx(1, 1);
    g.goal  = g.idx(g.rows - 2, g.cols - 2);
    g.clearSearch();
}

inline void sprinkleSwamp(Grid& g, std::mt19937& rng, double probability = 0.25) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    for (int i = 0; i < g.size(); ++i) {
        if (i == g.start || i == g.goal) continue;
        if (g.cells[i] == Cell::Empty && coin(rng) < probability) g.cells[i] = Cell::Swamp;
    }
    g.clearSearch();
}
