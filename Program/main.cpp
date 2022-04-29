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

		// Print all algorithm parameter values
		if (commandline.verbose) print_algorithm_parameters(commandline.ap);

		// Reading the data file and initializing some data structures
		if (commandline.verbose) std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
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
			commandline.verbose,
			commandline.ap
		);

		// Initializing the different building blocks of the HGS algorithm
		Split split(&params);
		LocalSearch localSearch(&params);
		Population population(&params, &split, &localSearch);
		Genetic solver(&params, &split, &population, &localSearch);

		// Running the algorithm
		solver.run();
		
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
