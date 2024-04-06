#pragma once

#include <complex>

#include <vector>

using cplex = std::complex<double>;

// for both routines, the vector a contains the polynomial coefficients of the
// polynomial f(x) = sum_{i=0}^{m} a[i]*x^i where m = a.size() - 1 is the order
// of the polynomial

// x is the initial guess to a single root, and laguerre refines that root
bool laguerre(const std::vector<cplex> &a, cplex &x, int &its);

// iteratively deflate the polynomial, using laguerre to refine each root
void lroots(const std::vector<cplex> &a, std::vector<cplex> &roots,
            bool polish = false);
