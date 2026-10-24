#include "vendor/quadprogpp/QuadProg++.hh"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using quadprogpp::Matrix;
using quadprogpp::Vector;

static void check(bool value, const char* what) {
  if (!value) throw std::runtime_error(what);
}

static Matrix<double> identity(unsigned n) {
  Matrix<double> G(n, n);
  for (unsigned i=0; i<n; ++i)
    for (unsigned j=0; j<n; ++j) G[i][j] = i==j ? 1. : 0.;
  return G;
}

template<typename Function> static void reject(Function action, const char* what) {
  bool threw=false;
  try {action();} catch (const std::exception&) {threw=true;}
  check(threw,what);
}

static void compare(unsigned n, bool equality, bool guarded, double linear_scale) {
  auto G=identity(n), H=G;
  Matrix<double> CE(n, equality?1:0), CI(n, 2*n+2);
  Vector<double> ce0(equality?1:0), ci0(2*n+2), g(n), a(n), b(n), tol(2*n+2);
  if (equality) ce0[0]=-.1;
  for (unsigned j=0; j<n; ++j) {
    g[j] = linear_scale*(j%2 ? .7 : -1.3);
    if (equality) CE[j][0] = j==n-1 ? 1. : 0.;
    for (unsigned i=0; i<2*n+2; ++i) CI[j][i]=0.;
    CI[j][j]=1.; CI[j][n+j]=-1.;
  }
  for (unsigned i=0; i<2*n+2; ++i) {ci0[i]=.4;tol[i]=1e-10;}
  // Duplicated active planes exercise unchanged working-set decisions.
  CI[0][2*n]=-1.; CI[0][2*n+1]=-1.;
  ci0[2*n]=ci0[2*n+1]=.2;
  const auto* t=guarded?&tol:nullptr;
  auto ga=g, gb=g;
  const double ca=quadprogpp::solve_quadprog(G,ga,CE,ce0,CI,ci0,a,1000,t);
  const double cb=quadprogpp::solve_quadprog_identity(H,gb,CE,ce0,CI,ci0,b,1000,t);
  check(std::isfinite(ca) && ca==cb, "identity objective differs from generic");
  for (unsigned j=0; j<n; ++j)
    check(a[j]==b[j], "identity solution differs from generic");
  for (unsigned i=0; i<2*n+2; ++i) {
    double slack=ci0[i];
    for (unsigned j=0; j<n; ++j) slack+=CI[j][i]*b[j];
    check(slack>=-1e-10,"original inequality violated");
  }
}

int main() {
  try {
    for (unsigned n: {2u,5u,75u})
      for (bool equality: {false,true})
        for (bool guarded: {false,true})
          for (double linear: {0.,.01,1.}) compare(n,equality,guarded,linear);
    // Single-variable unconstrained path, including nonzero g0.
    {
      auto A=identity(1),B=A;Matrix<double> E(1,0),I(1,0);
      Vector<double> e(0),i(0),g1(1),x1(1),x2(1);g1[0]=-.3;
      const double a=quadprogpp::solve_quadprog(A,g1,E,e,I,i,x1);
      const double b=quadprogpp::solve_quadprog_identity(B,g1,E,e,I,i,x2);
      check(a==b && x1[0]==x2[0] && x2[0]==.3,"single unconstrained differs");
    }
    Matrix<double> CE(2,0),CI(2,0);
    Vector<double> ce0(0),ci0(0),g(2),x(2);
    g[0]=-.4;g[1]=.6;
    for (double invalid: {std::nextafter(1.,2.),2.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      auto G=identity(2);G[0][0]=invalid;bool rejected=false;
      try {quadprogpp::solve_quadprog_identity(G,g,CE,ce0,CI,ci0,x);}
      catch (const std::logic_error&) {rejected=true;}
      check(rejected,"nonidentity diagonal accepted");
    }
    auto G=identity(2);G[0][1]=std::numeric_limits<double>::denorm_min();
    bool rejected=false;
    try {quadprogpp::solve_quadprog_identity(G,g,CE,ce0,CI,ci0,x);}
    catch (const std::logic_error&) {rejected=true;}
    check(rejected,"nonzero offdiagonal accepted");
    // The generic API still accepts and solves a non-identity SPD Hessian.
    G=identity(2);G[0][0]=2.;
    quadprogpp::solve_quadprog(G,g,CE,ce0,CI,ci0,x);
    check(std::abs(x[0]-.2)<1e-15 && x[1]==-.6,"generic SPD path changed");
    G=identity(2);
    auto invoke=[&](const Matrix<double>& E,const Vector<double>& e,const Matrix<double>& I,const Vector<double>& i,const Vector<double>* tolerance=nullptr) {
      quadprogpp::solve_quadprog_identity(G,g,E,e,I,i,x,100,tolerance);
    };
    reject([&]{invoke(Matrix<double>(1,0),ce0,CI,ci0);},"bad CE rows accepted");
    reject([&]{invoke(CE,Vector<double>(1),CI,ci0);},"bad equality offsets accepted");
    reject([&]{invoke(CE,ce0,Matrix<double>(1,0),ci0);},"bad CI rows accepted");
    reject([&]{invoke(CE,ce0,CI,Vector<double>(1));},"bad inequality offsets accepted");
    Vector<double> badtol(1);badtol[0]=1e-10;
    reject([&]{invoke(CE,ce0,CI,ci0,&badtol);},"bad tolerance dimensions accepted");
    Matrix<double> plane(2,1);plane[0][0]=1.;plane[1][0]=0.;Vector<double> offset(1);offset[0]=0.;
    for (double bad: {0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      badtol[0]=bad;
      reject([&]{invoke(CE,ce0,plane,offset,&badtol);},"bad tolerance value accepted");
    }
    g.resize(1);
    reject([&]{invoke(CE,ce0,CI,ci0);},"bad linear dimension accepted");
    g.resize(2);g[0]=std::numeric_limits<double>::quiet_NaN();g[1]=0.;
    reject([&]{invoke(CE,ce0,CI,ci0);},"nonfinite linear term accepted");
    G=identity(0);g.resize(0);Matrix<double> empty(0,0);Vector<double> zero(0);
    reject([&]{quadprogpp::solve_quadprog_identity(G,g,empty,zero,empty,zero,x);},"empty Hessian accepted");
    // Both routes retain the bounded active-set failure, with no returned fallback.
    for (bool fast: {false,true}) {
      G=identity(2);g.resize(2);g[0]=0.;g[1]=0.;offset[0]=-.1;
      reject([&]{
        if (fast) quadprogpp::solve_quadprog_identity(G,g,CE,ce0,plane,offset,x,1);
        else quadprogpp::solve_quadprog(G,g,CE,ce0,plane,offset,x,1);
      },"iteration limit accepted an unfinished solve");
    }
    std::cout << "Identity/generic exact equivalence and strict-entry tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';return 1;
  }
}
