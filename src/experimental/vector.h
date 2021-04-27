#ifndef __VECTOR_H_
#define __VECTOR_H_
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <ostream>
#include <cassert>

#include "../common/container.h"

template <typename T, size_t N>
class Vector : public FixedArray<T, N> {
public:
  double sum() const {
    double sum = 0;
    for (size_t i = 0; i < N; ++i)
      sum += (*this)[i];
    return sum;
  }

  double norm() const {
    const Vector &v = *this;
    return std::sqrt((v * v).sum());
  }
};

template <typename T, size_t N>
Vector<T, N> operator+(const Vector<T, N> &a, const Vector<T, N> &b) {
  Vector<T, N> out;
  for (int i = 0; i < N; ++i)
    out[i] = a[i] + b[i];
  return out;
}

template <typename T, size_t N>
Vector<T, N> &operator+=(const Vector<T, N> &a, const Vector<T, N> &b) {
  return a + b;
}

template <typename T, size_t N>
Vector<T, N> operator-(const Vector<T, N> &a, const Vector<T, N> &b) {
  Vector<T, N> out;
  for (size_t i = 0; i < N; ++i)
    out[i] = a[i] - b[i];
  return out;
}

template <typename T, size_t N>
Vector<T, N> &operator-=(const Vector<T, N> &a, const Vector<T, N> &b) {
  return a - b;
}

template <typename T, size_t N>
Vector<T, N> operator*(const Vector<T, N> &a, const Vector<T, N> &b) {
  Vector<T, N> out;
  for (size_t i = 0; i < N; ++i)
    out[i] = a[i] * b[i];
  return out;
}

template <typename T, size_t N>
Vector<T, N> operator*(const Vector<T, N> &a, const T b) {
  Vector<T, N> out;
  for (size_t i = 0; i < N; ++i)
    out[i] = a[i] * b;
  return out;
}

template <typename T, size_t N>
Vector<T, N> operator*(const T b, const Vector<T, N> &a) {
  return a * b;
}

template <typename T, size_t N>
std::ostream& operator<< (std::ostream& os, const Vector<T, N> &s) {
  os << "[";
  for (size_t i = 0; i < s.size(); ++i) {
    os << s[i];
    if (i == s.size() - 1) break;
    os << ", ";
  }
  os << "]";
  return os;
}
#endif // __VECTOR_H_
