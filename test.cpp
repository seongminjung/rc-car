#include <iostream>
#include <vector>

int main() {
  std::vector<std::vector<int>> groups;
  std::vector<std::vector<int>> a;
  std::vector<int> b;
  a = std::vector<std::vector<int>>();

  groups.insert(groups.end(), a.begin(), a.end());
  for (int i = 0; i < groups.size(); i++) {
    std::cout << groups[i].size() << std::endl;
  }
  std::cout << groups.size() << std::endl;
  groups.push_back(b);
  std::cout << groups.size() << std::endl;
}