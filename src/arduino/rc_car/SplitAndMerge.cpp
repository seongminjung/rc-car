#include "SplitAndMerge.h"

#include <Arduino_AVRSTL.h>

#include <cmath>
#include <vector>

#define IR_OFFSET 7

SplitAndMerge::SplitAndMerge() { threshold = 5; }

std::vector<std::vector<Point>> SplitAndMerge::grabData(std::vector<float> ir_in) {
  std::vector<Point> group;
  std::vector<std::vector<Point>> groups;
  for (int i = 0; i < ir_in.size(); i++) {
    if (ir_in[i] < 70) {
      Point point;
      point.x = (ir_in[i] + IR_OFFSET) * cos((i * 22.5 - 90) * 3.141592 / 180);
      point.y = (ir_in[i] + IR_OFFSET) * sin((i * 22.5 - 90) * 3.141592 / 180);
      group.push_back(point);
    } else {
      if (group.size() > 0) {
        groups.push_back(group);
        group.clear();
      }
    }
  }
  // put final group into groups
  if (group.size() > 0) {
    groups.push_back(group);
    group.clear();
  }
  std::vector<std::vector<Point>> result;
  for (int i = 0; i < groups.size(); i++) {
    std::vector<std::vector<Point>> temp = runSplitAndMerge(groups[i]);
    result.insert(result.end(), temp.begin(), temp.end());
  }
  return result;
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
  // if (points.size() <= 1) {
  //   // if a line segment has only one point, return
  //   return std::vector<std::vector<Point>>();
  // }
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

    result.insert(result.end(), result_left.begin(), result_left.end());
    result.insert(result.end(), result_right.begin(), result_right.end());
  } else {
    // one line segment
    result.push_back(points);
  }
  return result;
}
