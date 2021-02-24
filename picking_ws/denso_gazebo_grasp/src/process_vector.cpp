#include <denso_gazebo_grasp/process_vector.h>

using process_vector::ProcessVector;

ProcessVector::ProcessVector()
{
}

double ProcessVector::getVectorNorm(const std::vector<double>& v)
{
  double norm_square{ 0.0 };
  for (int i = 0; i < v.size(); i++)
  {
    norm_square += v[i] * v[i];
  }
  return std::sqrt(norm_square);
}

double ProcessVector::getDegFromVectors(const std::vector<double>& v1, const std::vector<double>& v2)
{
  if (v1.size() != v2.size())
  {
    std::cerr << "Vector Size Error" << std::endl;
  }
  std::vector<double> v1_normalize = normalizeVector(v1);
  std::vector<double> v2_normalize = normalizeVector(v2);
  return std::acos(std::inner_product(v1_normalize.begin(), v1_normalize.end(), v2_normalize.begin(), 0.0f)) * 180 /
         M_PI;
}

std::vector<double> ProcessVector::normalizeVector(const std::vector<double>& v)
{
  double v_norm = getVectorNorm(v);
  std::vector<double> normalize_vector;

  std::for_each(v.begin(), v.end(), [&](double elem)
  {
    normalize_vector.push_back(elem / v_norm);
  });

  return normalize_vector;
}
