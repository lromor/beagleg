#ifndef __QUAD_H_
#define __QUAD_H_

#include <functional>

bool Integrate(const std::function<double(double)> &f, double a, double b,
               double *result, double *abserr = NULL, double epsabs = 1e-6f,
               double epsrel = 1e-6f, size_t limit = 1e3, int key = 6);

#endif // __QUAD_H_
