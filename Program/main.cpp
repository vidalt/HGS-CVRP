#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "CVRPLIB.h"
using namespace std;

int main(int argc, char *argv[])
{
	try
	{
		// Reading the arguments of the program
		CommandLine commandline(argc, argv);
		bool verbose = commandline.verbose;

		// Print all algorithm parameter values
		if (verbose) print_algorithm_parameters(commandline.ap);

		// Reading the data file and initializing some data structures
		if (verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		CVRPLIB cvrp(commandline.pathInstance, commandline.isRoundingInteger);

		Params params(
			cvrp.x_coords,
			cvrp.y_coords,
			cvrp.dist_mtx,
			cvrp.service_time,
			cvrp.demands,
			cvrp.vehicleCapacity,
			cvrp.durationLimit,
			commandline.nbVeh,
			cvrp.isDurationConstraint,
			verbose,
			commandline.ap
		);
		if (verbose) std::cout << "----- INSTANCE LOADED WITH " << params.nbClients << " CLIENTS AND " << params.nbVehicles << " VEHICLES" << std::endl;

		// Creating the Split and local search structures
		Split split(&params);
		LocalSearch localSearch(&params);

		// Initial population
		if (verbose) std::cout << "----- BUILDING INITIAL POPULATION" << std::endl;
		Population population(&params, &split, &localSearch);

		// Genetic algorithm
		if (verbose) std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
		Genetic solver(&params, &split, &population, &localSearch);
		solver.run();
		if (verbose) std::cout << "----- GENETIC ALGORITHM FINISHED, TIME SPENT: " << (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC << std::endl;

		// Exporting the best solution
		if (population.getBestFound() != NULL)
		{
			population.getBestFound()->exportCVRPLibFormat(commandline.pathSolution);
			population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance, commandline.ap.seed);
			if (commandline.pathBKS != "") population.exportBKS(commandline.pathBKS);
		}
	}
	catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
	return 0;
}
