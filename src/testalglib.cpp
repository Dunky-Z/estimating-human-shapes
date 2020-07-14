//#include "../alglib/cpp/src/stdafx.h"
//#include <stdlib.h>
//#include <stdio.h>
//#include <math.h>
//#include "../alglib/cpp/src/optimization.h"
//
//using namespace alglib;
//void function1_grad(const real_1d_array &x, double &func, real_1d_array &grad, void *ptr)
//{
//	//
//	// this callback calculates f(x0,x1) = 100*(x0+3)^4 + (x1-3)^4
//	// and its derivatives df/d0 and df/dx1
//	//
//	func = 100 * pow(x[0] + 3, 4) + pow(x[1] - 3, 4);
//	grad[0] = 400 * pow(x[0] + 3, 3);
//	grad[1] = 4 * pow(x[1] - 3, 3);
//}
//
//int main(int argc, char **argv)
//{
//	//
//	// This example demonstrates minimization of
//	//
//	//     f(x,y) = 100*(x+3)^4+(y-3)^4
//	//
//	// using LBFGS method, with:
//	// * initial point x=[0,0]
//	// * unit scale being set for all variables (see minlbfgssetscale for more info)
//	// * stopping criteria set to "terminate after short enough step"
//	// * OptGuard integrity check being used to check problem statement
//	//   for some common errors like nonsmoothness or bad analytic gradient
//	//
//	// First, we create optimizer object and tune its properties
//	//
//	real_1d_array x = "[0,0]";
//	real_1d_array s = "[1,1]";
//	double epsg = 0;
//	double epsf = 0;
//	double epsx = 0.0000000001;
//	ae_int_t maxits = 0;
//	minlbfgsstate state;
//	minlbfgscreate(1, x, state);
//	minlbfgssetcond(state, epsg, epsf, epsx, maxits);
//	minlbfgssetscale(state, s);
//
//	//
//	// Activate OptGuard integrity checking.
//	//
//	// OptGuard monitor helps to catch common coding and problem statement
//	// issues, like:
//	// * discontinuity of the target function (C0 continuity violation)
//	// * nonsmoothness of the target function (C1 continuity violation)
//	// * erroneous analytic gradient, i.e. one inconsistent with actual
//	//   change in the target/constraints
//	//
//	// OptGuard is essential for early prototyping stages because such
//	// problems often result in premature termination of the optimizer
//	// which is really hard to distinguish from the correct termination.
//	//
//	// IMPORTANT: GRADIENT VERIFICATION IS PERFORMED BY MEANS OF NUMERICAL
//	//            DIFFERENTIATION. DO NOT USE IT IN PRODUCTION CODE!!!!!!!
//	//
//	//            Other OptGuard checks add moderate overhead, but anyway
//	//            it is better to turn them off when they are not needed.
//	//
//	minlbfgsoptguardsmoothness(state);
//	minlbfgsoptguardgradient(state, 0.001);
//
//	//
//	// Optimize and examine results.
//	//
//	minlbfgsreport rep;
//	alglib::minlbfgsoptimize(state, function1_grad);
//	minlbfgsresults(state, x, rep);
//	printf("%s\n", x.tostring(2).c_str()); // EXPECTED: [-3,3]
//
//	//
//	// Check that OptGuard did not report errors
//	//
//	// NOTE: want to test OptGuard? Try breaking the gradient - say, add
//	//       1.0 to some of its components.
//	//
//	optguardreport ogrep;
//	minlbfgsoptguardresults(state, ogrep);
//	printf("%s\n", ogrep.badgradsuspected ? "true" : "false"); // EXPECTED: false
//	printf("%s\n", ogrep.nonc0suspected ? "true" : "false"); // EXPECTED: false
//	printf("%s\n", ogrep.nonc1suspected ? "true" : "false"); // EXPECTED: false
//	return 0;
//}