#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
using namespace std;

int main(int argc, char *argv[])
{
	try
	{
		// Reading the arguments of the program
		CommandLine commandline(argc, argv);

		// Reading the data file and initializing some data structures
		std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		Params params(commandline.pathInstance, commandline.nbVeh, commandline.seed);
		std::cout << "----- INSTANCE LOADED WITH " << params.nbClients << " CLIENTS AND " << params.nbVehicles << " VEHICLES" << std::endl;

		// Creating the Split and local search structures
		Split split(&params);
		LocalSearch localSearch(&params);

		// Initial population
		std::cout << "----- BUILDING INITIAL POPULATION" << std::endl;
		Population population(&params, &split, &localSearch);

		// Genetic algorithm
		std::cout << "----- STARTING GENETIC ALGORITHM" << std::endl;
		Genetic solver(&params, &split, &population, &localSearch);
		solver.run(commandline.nbIter, commandline.timeLimit);
		std::cout << "----- GENETIC ALGORITHM FINISHED, TIME SPENT: " << (double)(clock()-params.startTime)/(double)CLOCKS_PER_SEC << std::endl;

		// Exporting the best solution
		if (population.getBestFound() != NULL)
		{
			population.getBestFound()->exportCVRPLibFormat(commandline.pathSolution);
			population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance, commandline.seed);
			if (commandline.pathBKS != "") population.exportBKS(commandline.pathBKS);
		}
	}
	catch (const string& e) { std::cout << "EXCEPTION | " << e << std::endl; }
	catch (const std::exception& e) { std::cout << "EXCEPTION | " << e.what() << std::endl; }
	return 0;
}
