//
// Created by chkwon on 3/24/22.
//

#include <stdio.h>
#include "C_Interface.h"
#include "AlgorithmParameters.h"
#include <assert.h>
#include <stdlib.h>
#include <math.h>

void print_solution(struct Solution * sol) {

	// Print the result
	printf("Total Route Cost = %f\n", sol->cost);
	printf("Total CPU Time = %f\n", sol->time);
	printf("Number of Routes = %d\n", sol->n_routes);
	for (int i = 0; i < sol->n_routes; i++) {
		struct SolutionRoute r = sol->routes[i];
		printf("Route #%d: ", i);
		printf("[");
		for (int j = 0; j < r.length; j++) {
			if (j < r.length - 1) {
				printf("%d, ", r.path[j]);
			}
			else {
				printf("%d]", r.path[j]);
			}
		}
		printf("\n");
	}
}
int main()
{

	// Preparing algorithm parameters
	struct AlgorithmParameters ap;
	ap = default_algorithm_parameters();
	ap.timeLimit = 1.73; // seconds

	// Problem Data
	int n = 10;
	double x[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	double y[] = {5, 4, 3, 2, 1, 9, 8, 7, 6, 5};
	double s[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	double d[] = {0, 2, 3, 1, 2, 3, 1, 2, 3, 1};
	double v_cap = 10;
	double duration_limit = 100000000;
	char isRoundingInteger = ap.isRoundingInteger;
	char isDurationConstraint = 0;
	int max_nbVeh = 2;
	char verbose = 1;

	// Solve
	struct Solution *sol = solve_cvrp(
		n, x, y, s, d,
		v_cap, duration_limit, isRoundingInteger, isDurationConstraint,
		max_nbVeh, &ap, verbose);
	print_solution(sol);

	// Test if the solution is correct
	assert(sol->cost == 29);

	// Test #2: solve by dist_mtx
	double dist_mtx[n][n];
	for (int i=0; i < n; i++) {
		for (int j=0; j< n; j++) {
			dist_mtx[i][j] = sqrt( (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]));
			dist_mtx[i][j] *= ( 1 + 0.05 * i - 0.03 * j);
		}
	}

	struct Solution *sol2 = solve_cvrp_dist_mtx(
		n, x, y, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol2);
	assert(round(sol2->cost) == 32);

	double zero[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	struct Solution *sol3 = solve_cvrp_dist_mtx(
		n, zero, zero, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol3);
	assert(round(sol2->cost) == round(sol3->cost));


	struct Solution *sol4 = solve_cvrp_dist_mtx(
		n, NULL, NULL, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol4);
	assert(round(sol2->cost) == round(sol4->cost));


	delete_solution(sol);
	delete_solution(sol2);
	delete_solution(sol3);
	delete_solution(sol4);

	return 0;
}