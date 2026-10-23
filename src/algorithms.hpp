#pragma once


#include "grid.hpp"

#include <algorithm>
#include <climits>
#include <cstdlib>
#include <memory>
#include <queue>
#include <vector>


class Algorithm {
public:
    virtual ~Algorithm() = default;
    virtual const char* name() const = 0;

    void reset(Grid& g) {
        g.clearSearch();
        parent_.assign(g.size(), -1);
        path_.clear();
        visited_  = 0;
        pathCost_ = 0;
        finished_ = false;
        found_    = false;
        onReset(g);
    }

    bool step(Grid& g) {
        if (finished_) return false;
        onStep(g);
        return !finished_;
    }

    bool finished() const { return finished_; }
    bool found()    const { return found_; }
    int  visited()  const { return visited_; }
    int  pathCost() const { return pathCost_; }
    const std::vector<int>& path() const { return path_; }

protected:
    virtual void onReset(Grid& g) = 0;
    virtual void onStep(Grid& g)  = 0;

    void finish(const Grid& g, bool found) {
        finished_ = true;
        found_    = found;
        if (!found) return;

        for (int at = g.goal; at != -1; at = parent_[at]) path_.push_back(at);
        std::reverse(path_.begin(), path_.end());

        pathCost_ = 0;
        for (std::size_t i = 1; i < path_.size(); ++i) pathCost_ += g.cost(path_[i]);
    }

    std::vector<int> parent_; 
    std::vector<int> path_;
    int  visited_  = 0;
    int  pathCost_ = 0;
    bool finished_ = false;
    bool found_    = false;
};


class BFS final : public Algorithm {
public:
    const char* name() const override { return "BFS"; }

protected:
    void onReset(Grid& g) override {
        frontier_ = std::queue<int>();
        frontier_.push(g.start);
        g.states[g.start] = State::InFrontier;
    }

    void onStep(Grid& g) override {
        if (frontier_.empty()) { finish(g, false); return; }

        const int cur = frontier_.front();
        frontier_.pop();
        g.states[cur] = State::Visited;
        ++visited_;
        if (cur == g.goal) { finish(g, true); return; }

        forEachNeighbor(g, cur, [&](int n) {
            if (g.states[n] != State::Unvisited) return;
            g.states[n] = State::InFrontier;
            parent_[n]  = cur;
            frontier_.push(n);
        });
    }

private:
    std::queue<int> frontier_;
};


class DFS final : public Algorithm {
public:
    const char* name() const override { return "DFS"; }

protected:
    void onReset(Grid& g) override {
        stack_.clear();
        stack_.push_back(g.start);
        g.states[g.start] = State::InFrontier;
    }

    void onStep(Grid& g) override {
       
        while (!stack_.empty() && g.states[stack_.back()] == State::Visited)
            stack_.pop_back();
        if (stack_.empty()) { finish(g, false); return; }

        const int cur = stack_.back();
        stack_.pop_back();
        g.states[cur] = State::Visited;
        ++visited_;
        if (cur == g.goal) { finish(g, true); return; }

        forEachNeighbor(g, cur, [&](int n) {
            if (g.states[n] == State::Visited) return;
            g.states[n] = State::InFrontier;
            parent_[n]  = cur;
            stack_.push_back(n);
        });
    }

private:
    std::vector<int> stack_;
};


class BestFirst : public Algorithm {
protected:
    virtual int heuristic(const Grid& g, int node) const = 0;

    void onReset(Grid& g) override {
        pq_ = Queue();
        dist_.assign(g.size(), INT_MAX);
        dist_[g.start] = 0;
        pq_.push({heuristic(g, g.start), 0, g.start});
        g.states[g.start] = State::InFrontier;
    }

    void onStep(Grid& g) override {
        while (!pq_.empty() && g.states[pq_.top().node] == State::Visited) pq_.pop();
        if (pq_.empty()) { finish(g, false); return; }

        const Entry e = pq_.top();
        pq_.pop();
        const int cur = e.node;
        g.states[cur] = State::Visited;
        ++visited_;
        if (cur == g.goal) { finish(g, true); return; }

        forEachNeighbor(g, cur, [&](int n) {
            if (g.states[n] == State::Visited) return;
            const int nd = dist_[cur] + g.cost(n);
            if (nd < dist_[n]) {           
                dist_[n]   = nd;
                parent_[n] = cur;
                g.states[n] = State::InFrontier;
                pq_.push({nd + heuristic(g, n), nd, n});
            }
        });
    }

private:
    struct Entry { int f; int g; int node; };
    struct Cmp {
        bool operator()(const Entry& a, const Entry& b) const {
            if (a.f != b.f) return a.f > b.f;   
            return a.g < b.g;                   
        }
    };
    using Queue = std::priority_queue<Entry, std::vector<Entry>, Cmp>;

    Queue            pq_;
    std::vector<int> dist_;
};

class Dijkstra final : public BestFirst {
public:
    const char* name() const override { return "Dijkstra"; }
protected:
    int heuristic(const Grid&, int) const override { return 0; }
};

class AStar final : public BestFirst {
public:
    const char* name() const override { return "A*"; }
protected:
    int heuristic(const Grid& g, int node) const override {
        return std::abs(g.rowOf(node) - g.rowOf(g.goal)) +
               std::abs(g.colOf(node) - g.colOf(g.goal));
    }
};

inline constexpr int kAlgorithmCount = 4;

inline std::unique_ptr<Algorithm> makeAlgorithm(int index) {
    switch (index) {
        case 0:  return std::make_unique<BFS>();
        case 1:  return std::make_unique<DFS>();
        case 2:  return std::make_unique<Dijkstra>();
        default: return std::make_unique<AStar>();
    }
}
