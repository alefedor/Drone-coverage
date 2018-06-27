#pragma once

struct task {
    double start;
    double end;
    int column;
    int drone;
};

inline bool operator < (const task &a, const task &b) {
    if (a.start < b.start)
        return true;
    if (a.start > b.start)
        return false;
    if (a.end < b.end)
        return true;
    if (a.end > b.end)
        return false;
    if (a.column < b.column)
        return true;
    if (a.column > b.column)
        return false;
    if (a.drone < b.drone)
        return true;
    else
        return false;
}
