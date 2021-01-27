#ifndef __VECTOR_H_
#define __VECTOR_H_
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <ostream>

template<size_t N>
struct Vector : public std::array<double, N> {
  constexpr double sum() {
    double sum = 0;
    for (size_t i = 0; i < N; ++i)
      sum += this->at(i);
    return sum;
  }
};

template<size_t N>
Vector<N> operator+(const Vector<N> &a, const Vector<N> &b) {
  Vector<N> out;
  for (int i = 0; i < N; ++i)
    out[i] = a[i] + b[i];
  return out;
}

template<size_t N>
Vector<N> &operator+=(const Vector<N> &a, const Vector<N> &b) {
  return a + b;
}

template<size_t N>
Vector<N> operator-(const Vector<N> &a, const Vector<N> &b) {
  Vector<N> out;
  for (int i = 0; i < N; ++i)
    out[i] = a[i] - b[i];
  return out;
}

template<size_t N>
Vector<N> &operator-=(const Vector<N> &a, const Vector<N> &b) {
  return a - b;
}

template<size_t N>
Vector<N> operator*(const Vector<N> &a, const Vector<N> &b) {
  Vector<N> out;
  for (int i = 0; i < N; ++i)
    out[i] = a[i] * b[i];
  return out;
}

template<size_t N, typename T>
Vector<N> operator*(const Vector<N> &a, const T b) {
  Vector<N> out;
  for (size_t i = 0; i < N; ++i)
    out[i] = a[i] * b;
  return out;
}

template<size_t N, typename T>
Vector<N> operator*(const T b, const Vector<N> &a) {
  return a * b;
}

template<size_t N>
inline double norm(const Vector<N> v) {
  double norm = 0;
  for (int i = 0; i < N; ++i)
    norm += v[i] * v[i];
  return std::sqrt(norm);
}

template<size_t N>
std::ostream& operator<< (std::ostream& os, const Vector<N> &s) {
  os << "[";
  for (int i = 0; i < s.size(); ++i) {
    os << s[i];
    if (i == s.size() - 1) break;
    os << ", ";
  }
  os << "]";
  return os;
}
#endif // __VECTOR_H_
