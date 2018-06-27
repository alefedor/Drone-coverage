#include <fstream>
#include <iostream>
#include <vector>
#include <limits>
#include <algorithm>
#include <set>
#include <queue>
#include <map>
#include <iomanip>
#include "geometry.h"
#include "task.h"

using namespace std;

const double eps = 1e-5;

point<double> getIntersection(double l, double r, const point<double> &a, const point<double> &b) {
    if (l <= a.x && a.x < r)
        return a;

    if (a.x < l)
        return point<double>{l, intersectionV(a, b, l)};
    else
        return point<double>{r, intersectionV(a, b, r)};
}


double getDist(const point<double> &a, const point<double> &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

int main() {
    freopen("input.txt", "r", stdin);
    freopen("output.txt", "w", stdout);
    int n;
    double cx, cy;
    int drones;
    cin >> drones;
    vector<point<double>> starts;

    for (int i = 0; i < drones; i++) {
        cin >> cx >> cy;
        starts.push_back(point<double> {cx, cy});
    }

    double mx, R;

    cin >> mx >> R;
    cin >> n;

    double l, r;
    l = numeric_limits<double>::max();
    r = numeric_limits<double>::min();

    vector<point<double>> points;
    for (int i = 0; i < n; i++) {
        double x, y;
        cin >> x >> y;
        points.push_back(point<double> {x, y});

        l = min(l, x);
        r = max(r, x);
    }

    int num = (int)((r - l + 2 * R - eps) / (2 * R));

    vector<pair<int, int>> events;
    for (int i = 0; i < n; i++) {
        int add = (int)((min(points[i].x, points[(i + 1) % n].x) - l) / (2 * R));
        int del = (int)((max(points[i].x, points[(i + 1) % n].x) - l) / (2 * R)) + 1;
        events.emplace_back(add, i + 1);
        events.emplace_back(del, -i - 1);
    }

    sort(events.begin(), events.end());
    auto it = events.begin();
    set<int> edges;

    vector<set<pair<double, double>>> intervals;
    intervals.resize(num);

    for (int i = 0; i < num; i++) {
        while (it != events.end() && it -> first == i && it -> second < 0) {
            int v = -it -> second - 1;
            edges.erase(v);
            it++;
        }
        while (it != events.end() && it -> first == i && it -> second > 0) {
            int v = it -> second - 1;
            edges.insert(v);
            it++;
        }

        vector<pair<double, bool>> borders;

        vector<point<double>> vertices;

        const double left = l + i * 2 * R;
        const double right = l + (i + 1) * 2 * R;

        for (int e : edges) {
            vertices.push_back(getIntersection(left, right, points[e], points[(e + 1) % n]));
            vertices.push_back(getIntersection(left, right, points[(e + 1) % n], points[e]));
        }

        auto cmpr = [](const point<double> &a, const point<double> &b) {
            return a.y < b.y;
        };

        sort(vertices.begin(), vertices.end(), cmpr);

        for (auto v : vertices) {
            if (!borders.empty() && v.y - eps <= borders.back().first)
                continue; // skip, because point already was handled

            for (double dy : {-eps, eps}) {
                double y = v.y + dy;

                bool shouldcover = false;
                segment<double> seg {point<double>{left, y}, point<double>{right, y}};
                for (int j = 0; j < n; j++) {
                    segment<double> edge {points[j], points[(j + 1) % n]};
                    if (intersects(seg, edge)) {
                        shouldcover = true;
                        break;
                    }
                }

                if (!shouldcover && inside(points, point<double> {v.x, y}))
                    shouldcover = true;

                borders.emplace_back(y, shouldcover);
            }
        }

        double last = numeric_limits<double>::min();
        double first = numeric_limits<double>::min();
        bool started = false;
        for (auto p : borders) {
            if (started) {
                if (!p.second) {
                    last = p.first;
                    intervals[i].insert({first, last});
                    first = p.first;
                    started = false;
                }
            } else {
                if (p.second) {
                    started = true;
                } else {
                    first = p.first;
                }
            }
        }
    }

    priority_queue<pair<double, task>, vector<pair<double, task>>, greater<pair<double, task>>> tasks;
    vector<vector<point<double>>> result;
    result.resize(drones);

    vector<double> dist;
    dist.resize(drones, 0);

    for (int i = 0; i < drones; i++)
        tasks.push(make_pair(0, task{-1, -1, -1, i}));

    double totalTime = 0;

    while (!tasks.empty()) {
        double time = tasks.top().first;
        task t = tasks.top().second;
        tasks.pop();

        double x;

        point<double> p;

        if (t.column == -1) {
            x = starts[t.drone].x;
            p = starts[t.drone];
        } else {
            x = l + 2 * R * t.column + R;
            p = {x, t.start};
        }
        result[t.drone].push_back(p);

        if (t.column == -1)
            dist[t.drone] = 0;

        if (fabs(t.start - t.end) > eps) { // has task

            double end;

            if (fabs(t.start - t.end) + getDist(starts[t.drone], point<double> {x, t.end}) + dist[t.drone] <= mx) {
                end = t.end;
            } else {
                double l = t.start;
                double r = t.end;
                for (int i = 0; i < 40; i++) { // binary search
                    double m = (l + r) / 2;
                    if (fabs(t.start - m) + getDist(starts[t.drone], point<double> {x, m}) + dist[t.drone] > mx)
                        r = m;
                    else
                        l = m;
                }

                end = l;

                intervals[t.column].insert({min(t.end, end), max(t.end, end)});
            }
            dist[t.drone] += fabs(end - t.start);
            tasks.push(make_pair(fabs(t.start - end) + time, task{end, end, t.column, t.drone}));
        } else { // need new task
            task newTask = {-1, -1, -1, t.drone};
            double cost = numeric_limits<double>::max();
            set<pair<double, double>>::iterator iterator;
            point<double> nextPoint;
            double y = (t.column == -1 ? starts[t.drone].y : t.start);

            auto update = [&](int column, set<pair<double, double>>::iterator it) {
                pair<double, double> interval = *it;
                double newX = l + 2 * R * column + R;

                if (fabs(x - newX) > cost)
                    return;

                point<double> newPoint;
                double nextY;

                if (y < interval.first + 2 * eps) {
                    newPoint = {newX, interval.first};
                    nextY = interval.second;
                } else if (interval.second - 2 * eps < y) {
                    newPoint = {newX, interval.second};
                    nextY = interval.first;
                } else {
                    newPoint = {newX, y};
                    if (rand() & 1)
                        nextY = interval.first;
                    else
                        nextY = interval.second;
                }

                nextPoint = newPoint;

                double d = getDist(p, newPoint);
                double fullDist = d + getDist(starts[t.drone], newPoint);

                if (fullDist + dist[t.drone] + eps <= mx && d < cost) {
                    cost = d;
                    iterator = it;
                    newTask = task{newPoint.y, nextY, column, t.drone};
                }
            };

            for (int i = 0; i < num; i++) {
                auto it = intervals[i].lower_bound({y, y});

                if (it != intervals[i].end())
                    update(i, it);

                if (it != intervals[i].begin()) {
                    it--;
                    update(i, it);
                }
            }

            if (newTask.column == -1) {
                if (t.column == -1) {
                    totalTime = time;
                    continue; // drone finished his work
                }
                tasks.push(make_pair(time + getDist(p, starts[t.drone]), newTask));
            } else {
                tasks.push(make_pair(time + cost, newTask));
                if (fabs(newTask.start - iterator -> first) < 2 * eps || fabs(newTask.start - iterator -> second) < 2 * eps) {
                    // nothing to do
                } else {
                    double a = iterator -> first + iterator -> second - newTask.end;
                    intervals[newTask.column].insert({min(a, y), max(a, y)});
                }
                intervals[newTask.column].erase(iterator);
            }

            dist[t.drone] += getDist(p, nextPoint);
        }
    }

    cout << fixed << setprecision(3);
    cout << totalTime << "\n";
    for (int i = 0; i < drones; i++) {
        cout << result[i].size() << "\n";
        for (auto p : result[i])
            cout << p.x << " " << p.y << "\n";
    }

    return 0;
}