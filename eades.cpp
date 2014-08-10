#include <iostream>
#include <vector>
#include <random>

const double MAX_N = 1e3;
const size_t NODE_SIZE = 30;

class Velocity {
public:
  double x_;
  double y_;

  Velocity() : x_(0.0), y_(0.0) {}
  Velocity(double x, double y) : x_(x), y_(y) {}
};

class Node {
public:
  std::string label;
  double x;
  double y;
  Velocity v;

};

void eades(std::uniform_real_distribution<double>& pos) {
  std::vector<Node> nodes(NODE_SIZE);
}

int main() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> pos(-MAX_N, MAX_N);
  eades(pos);
  return 0;
}
