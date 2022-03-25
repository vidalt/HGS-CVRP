//
// Created by chkwon on 3/23/22.
//

#include "C_Interface.h"
#include "Population.h"
#include "Params.h"
#include "Genetic.h"
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

Solution *prepare_solution(Population &population, Params &params)
{

	// Preparing the best solution
	auto *sol = new Solution;
	sol->time = (double)(clock() - params.startTime) / (double)CLOCKS_PER_SEC;

	if (population.getBestFound() != nullptr) {
		// Best individual
		auto best = population.getBestFound();

		// setting the cost
		sol->cost = best->myCostSol.penalizedCost;

		// finding out the number of routes in the best individual
		int n_routes = 0;
		for (int k = 0; k < best->params->nbVehicles; k++)
			if (!best->chromR[k].empty()) ++n_routes;

		// filling out the route information
		sol->n_routes = n_routes;
		sol->routes = new SolutionRoute[n_routes];
		for (int k = 0; k < n_routes; k++) {
			sol->routes[k].length = (int)best->chromR[k].size();
			sol->routes[k].path = new int[sol->routes[k].length];
			std::copy(best->chromR[k].begin(), best->chromR[k].end(), sol->routes[k].path);
		}
	}
	else {
		sol->cost = 0.0;
		sol->n_routes = 0;
		sol->routes = nullptr;
	}
	return sol;
}

Solution *run_hgs_cvrp(Params &params, AlgorithmParameters &ap)
{
	// Creating the Split and local search structures
	Split split(&params);
	LocalSearch localSearch(&params);

	bool verbose = params.verbose;

	// Initial population
	if (verbose) {
		std::cout << "----- INSTANCE LOADED WITH " << params.nbClients << " CLIENTS AND " << params.nbVehicles
				  << " VEHICLES" << std::endl;
		std::cout << "----- BUILDING INITIAL POPULATION" << std::endl;
	}
	Population population(&params, &split, &localSearch);

	// Genetic algorithm
	if (verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
	Genetic solver(&params, &split, &population, &localSearch);
	solver.run(ap.nbIter, ap.timeLimit);
	if (verbose)
		std::cout << "----- GENETIC ALGORITHM FINISHED, TIME SPENT: " << (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC << " s" << std::endl;

	return prepare_solution(population, params);
}

extern "C" Solution *solve_cvrp(
	int n, double *x, double *y, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isRoundingInteger, char isDurationConstraint,
	int max_nbVeh, AlgorithmParameters *ap, char verbose)
{
	Solution *result;

	try {
		std::vector<double> x_coords(x, x + n);
		std::vector<double> y_coords(y, y + n);
		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector < std::vector< double > > dist_mtx = std::vector < std::vector< double > >(n + 1, std::vector <double>(n + 1));
		for (int i = 0; i <= n; i++)
		{
			for (int j = 0; j <= n; j++)
			{
				dist_mtx[i][j] = std::sqrt(
					(x_coords[i] - x_coords[j])*(x_coords[i] - x_coords[j])
					+ (y_coords[i] - y_coords[j])*(y_coords[i] - y_coords[j])
				);
				if (isRoundingInteger)
					dist_mtx[i][j] = std::round(dist_mtx[i][j]);
			}
		}

		Params params(
			x_coords,
			y_coords,
			dist_mtx,
			service_time,
			demands,
			vehicleCapacity,
			durationLimit,
			max_nbVeh,
			isDurationConstraint,
			verbose,
			*ap
		);
		result = run_hgs_cvrp(params, *ap);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}

extern "C" Solution *solve_cvrp_dist_mtx(
	int n, double *x, double *y, double *dist_mtx, double *serv_time, double *dem,
	double vehicleCapacity, double durationLimit, char isDurationConstraint,
	int max_nbVeh, AlgorithmParameters *ap, char verbose)
{
	Solution *result;

	try {
		std::vector<double> x_coords(x, x + n);
		std::vector<double> y_coords(y, y + n);
		std::vector<double> service_time(serv_time, serv_time + n);
		std::vector<double> demands(dem, dem + n);

		std::vector<std::vector<double> > distance_matrix(n, std::vector<double>(n));
		for (int i = 0; i < n; i++) { // row
			for (int j = 0; j < n; j++) { // column
				distance_matrix[i][j] = dist_mtx[n * i + j];
			}
		}

		Params params(
			x_coords,
			y_coords,
			distance_matrix,
			service_time,
			demands,
			vehicleCapacity,
			durationLimit,
			max_nbVeh,
			isDurationConstraint,
			verbose,
			*ap
		);
		result = run_hgs_cvrp(params, *ap);
	}
	catch (const std::string &e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception &e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }

	return result;
}




extern "C" void delete_solution(Solution *sol)
{
	for (int i = 0; i < sol->n_routes; ++i)
		delete[] sol->routes[i].path;

	delete[] sol->routes;
	delete sol;
}