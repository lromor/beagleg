#ifndef __BEZIER_H_
#define __BEZIER_H_

#include <iostream>
#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1
#include <cmath>
#include <cstddef>
#include <array>
#include <vector>

#include "vector.h"
#include "quad.h"

inline double binom(int n, int k) {
  return 1 / ((n + 1) * std::beta(n - k + 1, k + 1));
}

template<typename T, int N>
Vector<T, N + 1> BernsteinBasis(double t) {
  Vector<T, N + 1> out;
  for (int i = 0; i <= N; ++i)
    out[i] = binom(N, i) * std::pow(t, i) * std::pow(1 - t, N - i);
  return out;
}

// N: Dimensionality.
// O: Order of the bezier.
template<typename T, size_t N, unsigned O>
struct Bezier {
  using CPType = std::array<Vector<T, N>, O + 1>;
  std::array<Vector<T, N>, O + 1> cps;

  Bezier<T, N, O-1> Derivative() const {
    std::array<Vector<T, N>, O> dcps;
    for (unsigned i = 0; i < O; ++i)
      dcps[i] = (cps[i + 1] - cps[i]) * static_cast<T>(O);
    Bezier<T, N, O-1> d = {dcps};
    return d;
  }

  double length() const {
    auto d = Derivative();
    double result;
    Integrate([&](double t) {
      return d(t).norm();
    }, 0, 1, &result);
    return result;
  }

  Vector<T, N> operator()(double t) {
    Vector<T, N> out = {};
    auto b = BernsteinBasis<T, O>(t);

    for (size_t i = 0; i < N; ++i)
      for (unsigned j = 0; j < O + 1; ++j)
        out[i] += cps[j][i] * b[j];
    return out;
  }
};

template<size_t N, typename T>
using CubicBezier = Bezier<T, N, 3>;
#endif // __BEZIER_H_
