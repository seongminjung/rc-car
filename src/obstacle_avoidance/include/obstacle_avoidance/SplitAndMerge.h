#include <vector>

struct PolarPoint {
  float r;
  float theta;
};

struct Point {
  int idx;  // for debugging
  float x;
  float y;
};

class SplitAndMerge {
 private:
  int threshold;  // centimeter

 public:
  SplitAndMerge();

  std::vector<std::vector<Point>> grabData(std::vector<float> ir_in);

  float getDistance(Point P, Point Ps, Point Pe);

  std::pair<float, int> GetMostDistant(std::vector<Point> points);

  std::vector<std::vector<Point>> runSplitAndMerge(std::vector<Point> points);
};