#pragma once

#include <stdlib.h>

template<typename T>
struct point {
    T x;
    T y;

    inline bool operator ==(const point<T> a) {
        return a.x == x && a.y == y;
    }
};

template<typename T>
struct segment {
    point<T> a;
    point<T> b;
};

template<typename T>
inline double intersectionV(const point<T> &a, const point<T> &b, T x) {
    if (fabs(a.x - b.x) < 1e-7)
        return a.y; // vertical segments are ignored

    return a.y + (x - a.x) / (double)(b.x - a.x) * (b.y - a.y);
}

template<typename T>
inline double cross(const point<T> &a, const point<T> &b, const point<T> &o) {
    return (a.x - o.x) * (double)(b.y - o.y) - (b.x - o.x) * (double)(a.y - o.y);
}

template<typename T>
inline bool intersects(const segment<T> &p, const segment<T> &q) {
    if (std::max(p.a.x, p.b.x) < std::min(q.a.x, q.b.x) ||
        std::max(q.a.x, q.b.x) < std::min(p.a.x, p.b.x) ||
        std::max(p.a.y, p.b.y) < std::min(q.a.y, q.b.y) ||
        std::max(q.a.y, q.b.y) < std::min(p.a.y, p.b.y))
        return false;

    return cross(p.b, q.a, p.a) * cross(p.b, q.b, p.a) <= 0 &&
           cross(q.b, p.a, q.a) * cross(q.b, p.b, q.a) <= 0;
}

template<typename T>
inline bool inside(const std::vector<point<double>> &points, const point<T> &p) {
    double rx = ((double)rand() / (double)RAND_MAX) * 1e8 + 1e9;
    double ry = ((double)rand() / (double)RAND_MAX) * 1e8 + 1e9;

    segment<double> seg {point<double> {p.x, p.y}, point<double> {rx, ry}};

    bool result = false;

    for (int i = 0; i < points.size(); i++) {
        int j = i + 1;
        if (j == points.size())
            j = 0;

        result ^= intersects(segment<double> {points[i], points[j]}, seg);
    }

    return result;
}