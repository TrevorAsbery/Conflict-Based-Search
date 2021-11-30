
#pragma once
#include "AStarPlanner.h"


class PrioritizedSearch {
public:
  vector<Path> find_solution();
  PrioritizedSearch(const MAPFInstance& ins): a_star(ins) {}

private:
  AStarPlanner a_star;

};
