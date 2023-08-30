#include "obstacle_avoidance/SplitAndMerge.h"

#include <cmath>
#include <vector>

SplitAndMerge::SplitAndMerge() { threshold = 5; }

std::vector<Point> SplitAndMerge::ir2Cartesian(std::vector<float> ir_in) {
  std::vector<Point> raw_points;  // raw data converted into cartesian
  for (int i = 0; i < ir_in.size(); i++) {
    float r = ir_in[i];
    float theta = i * 22.5 - 90;
    float x = r * cos(theta);
    float y = r * sin(theta);
    Point point;
    point.x = x;
    point.y = y;
    raw_points.push_back(point);
  }
  return raw_points;
}

std::vector<std::vector<Point>> SplitAndMerge::grabData(std::vector<float> ir_in) {
  return runSplitAndMerge(ir2Cartesian(ir_in));
}

float SplitAndMerge::getDistance(Point P, Point Ps, Point Pe) {
  float dist;
  if (Ps.x == Pe.x && Ps.y == Pe.y) {
    dist = sqrt(pow(P.x - Ps.x, 2) + pow(P.y - Ps.y, 2));
  } else {
    float parallelogram_area = abs((Pe.x - Ps.x) * (P.y - Ps.y) - (P.x - Ps.x) * (Pe.y - Ps.y));
    float base = sqrt(pow(Ps.x - Pe.x, 2) + pow(Ps.y - Pe.y, 2));
    dist = parallelogram_area / base;
  }
  return dist;
}

std::pair<float, int> SplitAndMerge::GetMostDistant(std::vector<Point> points) {
  float dmax = 0.0;
  int index = -1;
  for (int i = 1; i < points.size() - 1; i++) {
    // no need to check the first and last points
    float d = getDistance(points[i], points[0], points[points.size() - 1]);
    if (d > dmax) {
      index = i;
      dmax = d;
    }
  }
  std::pair<float, int> result(dmax, index);
  return result;
}

std::vector<std::vector<Point>> SplitAndMerge::runSplitAndMerge(std::vector<Point> points) {
  std::pair<float, int> most_distant = GetMostDistant(points);
  float d = most_distant.first;
  int ind = most_distant.second;
  std::vector<std::vector<Point>> result;
  if (d > threshold) {
    // e.g. if d = 3, group {0, 1, 2, 3}, {3, 4, 5, 6, ...}, So, duplicate 3
    auto start_itr = points.begin();
    auto end_itr = points.end();
    std::vector<Point> points_left(ind + 1);
    std::vector<Point> points_right(points.size() - ind);
    copy(start_itr, start_itr + ind + 1, points_left.begin());
    copy(start_itr + ind, end_itr, points_right.begin());

    std::vector<std::vector<Point>> result_left = runSplitAndMerge(points_left);
    std::vector<std::vector<Point>> result_right = runSplitAndMerge(points_right);

    result = result_left;
    result.insert(result.end(), result_right.begin(), result_right.end());
  } else {
    // one line segment
    result.push_back(points);
  }
  return result;
}
