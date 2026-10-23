

#include "raylib.h"

#include "algorithms.hpp"
#include "grid.hpp"
#include "maze.hpp"

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

namespace {

constexpr int kRows   = 31;
constexpr int kCols   = 45;
constexpr int kCellPx = 22;
constexpr int kGridW  = kCols * kCellPx;
constexpr int kGridH  = kRows * kCellPx;
constexpr int kPanelW = 290;
constexpr int kWinW   = kGridW + kPanelW;
constexpr int kWinH   = kGridH;

constexpr int kSpeeds[]   = {5, 10, 20, 40, 80, 160, 320, 640, 1280, 2560};
constexpr int kSpeedCount = static_cast<int>(sizeof(kSpeeds) / sizeof(kSpeeds[0]));
constexpr double kPathNodesPerSecond = 120.0;  


const Color kPanelBg    = {28, 32, 44, 255};
const Color kCellEmpty  = {236, 239, 244, 255};
const Color kGridLine   = {206, 212, 222, 255};
const Color kWall       = {38, 43, 58, 255};
const Color kSwamp      = {150, 176, 108, 255};
const Color kFrontier   = {150, 232, 222, 255};
const Color kVisited    = {96, 150, 235, 255};
const Color kPathColor  = {250, 200, 40, 255};
const Color kStart      = {46, 190, 110, 255};
const Color kGoal       = {230, 60, 70, 255};
const Color kText       = {232, 236, 244, 255};
const Color kTextDim    = {150, 158, 176, 255};


bool Button(Rectangle r, const char* label, bool active = false) {
    const bool hover = CheckCollisionPointRec(GetMousePosition(), r);
    Color bg = active ? Color{70, 110, 200, 255}
             : hover  ? Color{66, 74, 98, 255}
                      : Color{48, 54, 74, 255};
    DrawRectangleRec(r, bg);
    DrawRectangleLinesEx(r, 1.0f, Color{84, 92, 120, 255});
    const int tw = MeasureText(label, 16);
    DrawText(label, static_cast<int>(r.x + (r.width - tw) / 2),
             static_cast<int>(r.y + (r.height - 16) / 2), 16, kText);
    return hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

class Visualizer {
public:
    Visualizer() : grid_(kRows, kCols), rng_(std::random_device{}()) {
        selectAlgorithm(3);  
    }

    void update() {
        handleKeyboard();
        handleMousePainting();
        advanceSearch();
        advancePathAnimation();
    }

    void draw() {
        ClearBackground(kPanelBg);
        drawGrid();
        drawPanel();
    }

private:

    Grid grid_;
    std::unique_ptr<Algorithm> algo_;
    int  algoIndex_   = 0;
    int  speedLevel_  = 4;
    bool running_     = false;
    double stepAcc_   = 0.0;   
    std::size_t revealed_ = 0;   
    double revealAcc_ = 0.0;
    std::mt19937 rng_;


    void selectAlgorithm(int index) {
        algoIndex_ = index;
        algo_ = makeAlgorithm(index);
        resetSearch();
    }

    void resetSearch() {
        running_   = false;
        stepAcc_   = 0.0;
        revealed_  = 0;
        revealAcc_ = 0.0;
        algo_->reset(grid_);
    }

    void toggleRun() {
        if (algo_->finished()) resetSearch();  
        running_ = !running_;
    }

    void singleStep() {
        if (algo_->finished()) return;
        running_ = false;
        algo_->step(grid_);
    }

    void handleKeyboard() {
        if (IsKeyPressed(KEY_ONE))   selectAlgorithm(0);
        if (IsKeyPressed(KEY_TWO))   selectAlgorithm(1);
        if (IsKeyPressed(KEY_THREE)) selectAlgorithm(2);
        if (IsKeyPressed(KEY_FOUR))  selectAlgorithm(3);

        if (IsKeyPressed(KEY_SPACE)) toggleRun();
        if (IsKeyPressed(KEY_N))     singleStep();
        if (IsKeyPressed(KEY_R))     resetSearch();

        if (IsKeyPressed(KEY_UP))   speedLevel_ = std::min(speedLevel_ + 1, kSpeedCount - 1);
        if (IsKeyPressed(KEY_DOWN)) speedLevel_ = std::max(speedLevel_ - 1, 0);

        if (IsKeyPressed(KEY_C)) { grid_.clearAll(); resetSearch(); }
        if (IsKeyPressed(KEY_M)) { generateMaze(grid_, rng_); resetSearch(); }
        if (IsKeyPressed(KEY_T)) { sprinkleSwamp(grid_, rng_); resetSearch(); }

        const int cell = cellUnderMouse();
        if (cell >= 0) {
            if (IsKeyPressed(KEY_S) && cell != grid_.goal)  moveEndpoint(grid_.start, cell);
            if (IsKeyPressed(KEY_G) && cell != grid_.start) moveEndpoint(grid_.goal, cell);
        }
    }

    void moveEndpoint(int& endpoint, int cell) {
        grid_.cells[cell] = Cell::Empty;   
        endpoint = cell;
        resetSearch();
    }


    int cellUnderMouse() const {
        const Vector2 m = GetMousePosition();
        const int x = static_cast<int>(m.x);
        const int y = static_cast<int>(m.y);
        if (x < 0 || y < 0 || x >= kGridW || y >= kGridH) return -1;
        return grid_.idx(y / kCellPx, x / kCellPx);
    }

    void handleMousePainting() {
        const int cell = cellUnderMouse();
        if (cell < 0 || cell == grid_.start || cell == grid_.goal) return;

        const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        Cell wanted;
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))       wanted = shift ? Cell::Swamp : Cell::Wall;
        else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) wanted = Cell::Empty;
        else return;

        if (grid_.cells[cell] != wanted) {  
            grid_.cells[cell] = wanted;
            resetSearch();
        }
    }

    void advanceSearch() {
        if (!running_) return;

        stepAcc_ += GetFrameTime() * kSpeeds[speedLevel_];
        int n = static_cast<int>(stepAcc_);
        stepAcc_ -= n;
        n = std::min(n, 5000);

        for (int i = 0; i < n; ++i) {
            if (!algo_->step(grid_)) { running_ = false; break; }
        }
    }

    void advancePathAnimation() {
        if (!algo_->finished() || !algo_->found()) return;
        const auto& path = algo_->path();
        if (revealed_ >= path.size()) return;

        revealAcc_ += GetFrameTime() * kPathNodesPerSecond;
        while (revealAcc_ >= 1.0 && revealed_ < path.size()) {
            grid_.states[path[revealed_++]] = State::Path;
            revealAcc_ -= 1.0;
        }
    }

    void drawGrid() const {
        for (int r = 0; r < kRows; ++r) {
            for (int c = 0; c < kCols; ++c) {
                const int i = grid_.idx(r, c);
                const int x = c * kCellPx;
                const int y = r * kCellPx;

                if (grid_.cells[i] == Cell::Wall) {
                    DrawRectangle(x, y, kCellPx, kCellPx, kWall);
                    continue;
                }

                DrawRectangle(x, y, kCellPx, kCellPx,
                              grid_.cells[i] == Cell::Swamp ? kSwamp : kCellEmpty);

                switch (grid_.states[i]) {
                    case State::InFrontier: DrawRectangle(x, y, kCellPx, kCellPx, Fade(kFrontier, 0.85f)); break;
                    case State::Visited:    DrawRectangle(x, y, kCellPx, kCellPx, Fade(kVisited, 0.75f));  break;
                    case State::Path:       DrawRectangle(x, y, kCellPx, kCellPx, kPathColor);            break;
                    case State::Unvisited:  break;
                }
                DrawRectangleLines(x, y, kCellPx, kCellPx, kGridLine);
            }
        }
        drawEndpoint(grid_.start, kStart, "S");
        drawEndpoint(grid_.goal,  kGoal,  "G");
    }

    void drawEndpoint(int node, Color color, const char* letter) const {
        const int x = grid_.colOf(node) * kCellPx;
        const int y = grid_.rowOf(node) * kCellPx;
        DrawRectangle(x + 1, y + 1, kCellPx - 2, kCellPx - 2, color);
        const int tw = MeasureText(letter, 16);
        DrawText(letter, x + (kCellPx - tw) / 2, y + (kCellPx - 16) / 2, 16, WHITE);
    }

    const char* statusText() const {
        if (algo_->finished()) return algo_->found() ? "Path found" : "No path";
        return running_ ? "Running" : "Paused / idle";
    }

    void legendItem(int x, int y, Color c, const char* label) const {
        DrawRectangle(x, y, 14, 14, c);
        DrawRectangleLines(x, y, 14, 14, Color{84, 92, 120, 255});
        DrawText(label, x + 22, y - 1, 14, kText);
    }

    void drawPanel() {
        const int px = kGridW + 16;
        const float bw = 126.0f, bh = 30.0f, gap = 8.0f;
        const float x1 = static_cast<float>(px), x2 = x1 + bw + gap;

        DrawText("Pathfinding", px, 14, 24, kText);
        DrawText("Visualizer", px, 40, 24, kText);

        DrawText("Algorithm", px, 78, 14, kTextDim);
        const char* names[kAlgorithmCount] = {"1  BFS", "2  DFS", "3  Dijkstra", "4  A*"};
        for (int i = 0; i < kAlgorithmCount; ++i) {
            const Rectangle r = {i % 2 == 0 ? x1 : x2, 98.0f + (i / 2) * (bh + gap), bw, bh};
            if (Button(r, names[i], i == algoIndex_)) selectAlgorithm(i);
        }

        DrawText("Controls", px, 180, 14, kTextDim);
        if (Button({x1, 200, bw, bh}, running_ ? "Pause" : "Run  (Space)")) toggleRun();
        if (Button({x2, 200, bw, bh}, "Step  (N)"))        singleStep();
        if (Button({x1, 238, bw, bh}, "Reset  (R)"))       resetSearch();
        if (Button({x2, 238, bw, bh}, "Clear  (C)"))     { grid_.clearAll(); resetSearch(); }
        if (Button({x1, 276, bw, bh}, "Maze  (M)"))      { generateMaze(grid_, rng_); resetSearch(); }
        if (Button({x2, 276, bw, bh}, "Swamp  (T)"))     { sprinkleSwamp(grid_, rng_); resetSearch(); }

     
        DrawText(TextFormat("Speed: %d steps/s", kSpeeds[speedLevel_]), px, 322, 16, kText);
        if (Button({x1, 344, bw, 26}, "Slower (Down)")) speedLevel_ = std::max(speedLevel_ - 1, 0);
        if (Button({x2, 344, bw, 26}, "Faster (Up)"))   speedLevel_ = std::min(speedLevel_ + 1, kSpeedCount - 1);

        
        DrawText("Statistics", px, 388, 14, kTextDim);
        DrawText(TextFormat("Algorithm: %s", algo_->name()), px, 408, 16, kText);
        DrawText(TextFormat("Status: %s", statusText()),     px, 430, 16, kText);
        DrawText(TextFormat("Visited nodes: %d", algo_->visited()), px, 452, 16, kText);
        if (algo_->finished() && algo_->found()) {
            DrawText(TextFormat("Path moves: %d", static_cast<int>(algo_->path().size()) - 1), px, 474, 16, kText);
            DrawText(TextFormat("Path cost:  %d", algo_->pathCost()), px, 496, 16, kText);
        }

       
        legendItem(px,       528, kStart,    "Start");
        legendItem(px + 140, 528, kGoal,     "Goal");
        legendItem(px,       548, kWall,     "Wall");
        legendItem(px + 140, 548, kSwamp,    "Swamp (x5)");
        legendItem(px,       568, Fade(kFrontier, 0.85f), "Frontier");
        legendItem(px + 140, 568, Fade(kVisited, 0.75f),  "Visited");
        legendItem(px,       588, kPathColor, "Path");

     
        DrawText("LMB wall   Shift+LMB swamp   RMB erase", px, 614, 12, kTextDim);
        DrawText("S / G over a cell: move start / goal",   px, 630, 12, kTextDim);
    }
};

}  

int main() {
    InitWindow(kWinW, kWinH, "Pathfinding Visualizer - BFS / DFS / Dijkstra / A*");
    SetTargetFPS(60);

    Visualizer app;
    while (!WindowShouldClose()) {
        app.update();
        BeginDrawing();
        app.draw();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
