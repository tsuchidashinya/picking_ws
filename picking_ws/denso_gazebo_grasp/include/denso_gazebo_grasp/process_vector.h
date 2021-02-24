#ifndef PROCESS_VECTOR_H
#define PROCESS_VECTOR_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

namespace process_vector
{
class ProcessVector
{
public:
  ProcessVector();
  double getVectorNorm(const std::vector<double>& v);
  double getDegFromVectors(const std::vector<double>& v1, const std::vector<double>& v2);

private:
  std::vector<double> normalizeVector(const std::vector<double>& v);
};
}  // namespace process_vector

#endif  // PROCESS_VECTOR_H
