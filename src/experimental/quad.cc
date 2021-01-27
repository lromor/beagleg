#include "quad.h"
#include <gsl/gsl_integration.h>

typedef std::function<double(double)> Function;

static double wrapper(double x, void * p) {
  Function &f = *(Function *) p;
  return f(x);
}

bool Integrate(const std::function<double(double)> &f, double a, double b,
               double *result, double *abserr, double epsabs,
               double epsrel, size_t limit, int key) {
  gsl_integration_workspace *workspace = gsl_integration_workspace_alloc(limit);
  gsl_function gsl_f;
  double abserr_local;
  gsl_f.function = &wrapper;
  gsl_f.params = (void *)&f;
  gsl_integration_qag(&gsl_f, a, b, epsabs, epsrel, limit, key, workspace, result, &abserr_local);
  gsl_integration_workspace_free(workspace);
  if (abserr)
    *abserr = abserr_local;

  if (abserr_local > epsabs)
    return false;
  return true;
}
