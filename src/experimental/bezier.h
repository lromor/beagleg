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

template<int N>
Vector<N + 1> BernsteinBasis(double t) {
  Vector<N + 1> out;
  for (int i = 0; i <= N; ++i)
    out[i] = binom(N, i) * std::pow(t, i) * std::pow(1 - t, N - i);
  return out;
}

// O: Order of the bezier.
// N: Dimensionality.
template<unsigned O, size_t N>
struct Bezier {
  using CPType = std::array<Vector<N>, O + 1>;
  std::array<Vector<N>, O + 1> cps;

  Bezier<O-1, N> Derivative() const {
    std::array<Vector<N>, O> dcps;
    for (unsigned i = 0; i < O; ++i)
      dcps[i] = (cps[i + 1] - cps[i]) * O;
    Bezier<O-1, N> d = {dcps};
    return d;
  }

  double length() const {
    auto d = Derivative();
    double result;
    Integrate([&](double t) {
      return norm(d(t));
    }, 0, 1, &result);
    return result;
  }

  Vector<N> operator()(double t) {
    Vector<N> out = {};
    auto b = BernsteinBasis<O>(t);

    for (size_t i = 0; i < N; ++i)
      for (unsigned j = 0; j < O + 1; ++j)
        out[i] += cps[j][i] * b[j];
    return out;
  }
};

template<size_t N>
using CubicBezier = Bezier<3, N>;

#endif // __BEZIER_H_
