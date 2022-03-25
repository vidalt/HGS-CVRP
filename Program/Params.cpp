#include "Params.h"

Params::Params(std::string pathToInstance, int nbVeh, int seedRNG) : nbVehicles(nbVeh)
{
	std::string content, content2, content3;
	double serviceTimeData = 0.;
	nbClients = 0;
	totalDemand = 0.;
	maxDemand = 0.;
	durationLimit = 1.e30;
	vehicleCapacity = 1.e30;
	isRoundingInteger = true;
	isDurationConstraint = false;

	// Initialize RNG
	srand(seedRNG);					

	// Read INPUT dataset
	std::ifstream inputFile(pathToInstance);
	if (inputFile.is_open())
	{
		getline(inputFile, content);
		getline(inputFile, content);
		getline(inputFile, content);
		for (inputFile >> content ; content != "NODE_COORD_SECTION" ; inputFile >> content)
		{
			if (content == "DIMENSION") { inputFile >> content2 >> nbClients; nbClients--; } // Need to substract the depot from the number of nodes
			else if (content == "EDGE_WEIGHT_TYPE")	inputFile >> content2 >> content3;
			else if (content == "CAPACITY")	inputFile >> content2 >> vehicleCapacity;
			else if (content == "DISTANCE") { inputFile >> content2 >> durationLimit; isDurationConstraint = true; }
			else if (content == "SERVICE_TIME")	inputFile >> content2 >> serviceTimeData;
			else throw std::string("Unexpected data in input file: " + content);
		}
		if (nbClients <= 0) throw std::string("Number of nodes is undefined");
		if (vehicleCapacity == 1.e30) throw std::string("Vehicle capacity is undefined");
		
		// Reading client coordinates
		cli = std::vector<Client>(nbClients + 1);
		for (int i = 0; i <= nbClients; i++)
		{
			inputFile >> cli[i].custNum >> cli[i].coordX >> cli[i].coordY;
			cli[i].custNum--;
			cli[i].polarAngle = CircleSector::positive_mod(32768.*atan2(cli[i].coordY - cli[0].coordY, cli[i].coordX - cli[0].coordX) / PI);
		}

		// Reading demand information
		inputFile >> content;
		if (content != "DEMAND_SECTION") throw std::string("Unexpected data in input file: " + content);
		for (int i = 0; i <= nbClients; i++)
		{
			inputFile >> content >> cli[i].demand;
			cli[i].serviceDuration = (i == 0) ? 0. : serviceTimeData ;
			if (cli[i].demand > maxDemand) maxDemand = cli[i].demand;
			totalDemand += cli[i].demand;
		}

		// Reading depot information (in all current instances the depot is represented as node 1, the program will return an error otherwise)
		inputFile >> content >> content2 >> content3 >> content3;
		if (content != "DEPOT_SECTION") throw std::string("Unexpected data in input file: " + content);
		if (content2 != "1") throw std::string("Expected depot index 1 instead of " + content2);
		if (content3 != "EOF") throw std::string("Unexpected data in input file: " + content3);
	}
	else
		throw std::invalid_argument("Impossible to open instance file: " + pathToInstance);		
	
	// Default initialization if the number of vehicles has not been provided by the user
	if (nbVehicles == INT_MAX)
	{
		nbVehicles = std::ceil(1.3*totalDemand/vehicleCapacity) + 3;  // Safety margin: 30% + 3 more vehicles than the trivial bin packing LB
		std::cout << "----- FLEET SIZE WAS NOT SPECIFIED: DEFAULT INITIALIZATION TO " << nbVehicles << " VEHICLES" << std::endl;
	}
	else
	{
		std::cout << "----- FLEET SIZE SPECIFIED IN THE COMMANDLINE: SET TO " << nbVehicles << " VEHICLES" << std::endl;
	}

	// Calculation of the distance matrix
	maxDist = 0.;
	timeCost = std::vector < std::vector< double > >(nbClients + 1, std::vector <double>(nbClients + 1));
	for (int i = 0; i <= nbClients; i++)
	{
		for (int j = 0; j <= nbClients; j++)
		{
			double d = std::sqrt((cli[i].coordX - cli[j].coordX)*(cli[i].coordX - cli[j].coordX) + (cli[i].coordY - cli[j].coordY)*(cli[i].coordY - cli[j].coordY));
			if (isRoundingInteger) { d += 0.5; d = (double)(int)d; } // integer rounding
			if (d > maxDist) maxDist = d;
			timeCost[i][j] = d;
		}
	}

	// Calculation of the correlated vertices for each customer (for the granular restriction)
	correlatedVertices = std::vector < std::vector < int > >(nbClients + 1);
	std::vector < std::set < int > > setCorrelatedVertices = std::vector < std::set <int> >(nbClients + 1);
	std::vector < std::pair <double, int> > orderProximity;
	for (int i = 1; i <= nbClients; i++)
	{
		orderProximity.clear();
		for (int j = 1; j <= nbClients; j++)
			if (i != j) orderProximity.push_back(std::pair <double, int>(timeCost[i][j], j));
		std::sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(nbGranular, nbClients - 1); j++)
		{
			// If i is correlated with j, then j should be correlated with i
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
	}

	// Filling the vector of correlated vertices
	for (int i = 1; i <= nbClients; i++)
		for (int x : setCorrelatedVertices[i])
			correlatedVertices[i].push_back(x);

	// Safeguards to avoid possible numerical instability in case of instances containing arbitrarily small or large numerical values
	if (maxDist < 0.1 || maxDist > 100000)   throw std::string("The distances are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (maxDemand < 0.1 || maxDemand > 100000) throw std::string("The demand quantities are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (nbVehicles < std::ceil(totalDemand / vehicleCapacity)) throw std::string("Fleet size is insufficient to service the considered clients.");

	// A reasonable scale for the initial values of the penalties
	penaltyDuration = 1;
	penaltyCapacity = std::max<double>(0.1, std::min<double>(1000., maxDist / maxDemand));
}