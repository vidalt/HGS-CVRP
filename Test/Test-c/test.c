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
	struct AlgorithmParameters ap = default_algorithm_parameters();
	ap.timeLimit = 1.73; // seconds
	ap.nbIter = 10000; // iterations

	// Problem Data
	int n = 10;
	double x[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	double y[] = {5, 4, 3, 2, 1, 9, 8, 7, 6, 5};
	double s[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	double d[] = {0, 2, 3, 1, 2, 3, 1, 2, 3, 1};
	double v_cap = 10;
	double duration_limit = 100000000;
	char isRoundingInteger = 1;
	char isDurationConstraint = 0;
	int max_nbVeh = 2;
	char verbose = 1;


	////////////////////////////////////////////////////////////////////////////////////////////////////////
	printf("-------- test.c #1 -----\n");
	// Solve
	struct Solution *sol = solve_cvrp(
		n, x, y, s, d,
		v_cap, duration_limit, isRoundingInteger, isDurationConstraint,
		max_nbVeh, &ap, verbose);
	print_solution(sol);

	// Test if the solution is correct
	assert(sol->cost == 29);

	// The default value of useSwapStart should have not changed.
	assert(ap.useSwapStar == 1);

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	printf("-------- test.c #2 -----\n");
	// Test #2: solve by dist_mtx
	double dist_mtx[10][10];
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

	// The default value of useSwapStart should have not changed.
	assert(ap.useSwapStar == 1);




	////////////////////////////////////////////////////////////////////////////////////////////////////////
	printf("-------- test.c #3 -----\n");

	double zero[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	struct Solution *sol3 = solve_cvrp_dist_mtx(
		n, zero, zero, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol3);
	assert(round(sol2->cost) == round(sol3->cost));

	// The default value of useSwapStart should have not changed.
	assert(ap.useSwapStar == 1);

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	printf("-------- test.c #4 -----\n");

	struct Solution *sol4 = solve_cvrp_dist_mtx(
		n, NULL, NULL, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol4);
	assert(round(sol2->cost) == round(sol4->cost));

	// The default value of useSwapStart should have not changed.
	assert(ap.useSwapStar == 1);

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	printf("-------- test.c #5 -----\n");

	ap.useSwapStar = 0;
	struct Solution *sol5 = solve_cvrp_dist_mtx(
		n, NULL, NULL, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol5);
	assert(round(sol2->cost) == round(sol4->cost));

	assert(ap.useSwapStar == 0);

	////////////////////////////////////////////////////////////////////////////////////////////////////////

	printf("-------- test.c #6 (redundant duration constraint) -----\n");

	ap.useSwapStar = 0;
	duration_limit = 1000;
	isDurationConstraint = 1;
	struct Solution *sol6 = solve_cvrp_dist_mtx(
		n, NULL, NULL, (double*)dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol6);
	assert(round(sol5->cost) == round(sol6->cost));

	////////////////////////////////////////////////////////////////////////////////////////////////////////

	printf("-------- test.c #7 (tight duration constraint) -----\n");

	double rounded_dist_mtx[10][10];
	for (int i=0; i < n; i++) {
		for (int j=0; j< n; j++) {
			rounded_dist_mtx[i][j] = sqrt( (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]));
			rounded_dist_mtx[i][j] = round(rounded_dist_mtx[i][j]);
			printf("%d ", (int) rounded_dist_mtx[i][j]);
		}
		printf("\n");
	}

	ap.useSwapStar = 1;
	ap.seed = 12;
	max_nbVeh = 5;
	duration_limit = 18;
	isDurationConstraint = 1;
	struct Solution *sol7 = solve_cvrp_dist_mtx(
		n, NULL, NULL, (double*)rounded_dist_mtx, s, d,
		v_cap, duration_limit, isDurationConstraint,
		max_nbVeh, &ap, verbose);

	print_solution(sol7);

	int tail, head;
	for (int r = 0; r < sol7->n_routes; r ++) {
		struct SolutionRoute route = sol7->routes[r];
		double duration = 0.0;
		for (int k = 0; k < route.length - 1; k ++) {
			tail = route.path[k];
			head = route.path[k+1];
			duration += rounded_dist_mtx[tail][head];
		}
		assert(duration <= duration_limit);
		printf("route #%d duration = %f\n", r, duration);
	}
	printf("duration_limit = %f\n", duration_limit);
	assert(sol7->cost == 42);

	////////////////////////////////////////////////////////////////////////////////////////////////////////

	delete_solution(sol);
	delete_solution(sol2);
	delete_solution(sol3);
	delete_solution(sol4);
	delete_solution(sol5);
	delete_solution(sol6);
	delete_solution(sol7);

	return 0;
}