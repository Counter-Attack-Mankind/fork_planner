//
// Created by yenkn on 20-7-27.
//
#pragma once

#include <vector>
#include <Eigen/Dense>

namespace planning_core {
enum class Padding { Null, Front, End };

template <typename T>
std::vector<T> diff(const std::vector<T> &vec, Padding padding = Padding::Null) {
  std::vector<T> result(vec.size());
  for(int i = 0; i < vec.size() - 1; i++) {
    result[i] = vec[i+1] - vec[i];
  }

  if(padding == Padding::Front) {
    result.insert(result.begin(), result[0]);
  } else if(padding == Padding::End) {
    result.push_back(result.back());
  }

  return result;
}

template<typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
  std::vector<T> values;
  for (T value = start; value < stop; value += step)
    values.push_back(value);
  return values;
}

}