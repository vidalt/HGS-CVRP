//
// Created by chkwon on 3/23/22.
//

#include "AlgorithmParameters.h"
#include <cfloat>
#include <iostream>

extern "C"
struct AlgorithmParameters default_algorithm_parameters() {
	struct AlgorithmParameters ap{};

	ap.nbGranular = 20;
	ap.mu = 25;
	ap.lambda = 40;
	ap.nbElite = 4;
	ap.nbClose = 5;
	ap.targetFeasible = 0.2;

	ap.seed = 0;
	ap.nbIter = 20000;
	ap.timeLimit = DBL_MAX;
	ap.isRoundingInteger = 1;

	return ap;
}

void print_algorithm_parameters(const AlgorithmParameters & ap)
{
	std::cout << "=========== Algorithm Parameters =================" << std::endl;
	std::cout << "---- nbGranular        is set to " << ap.nbGranular << std::endl;
	std::cout << "---- mu                is set to " << ap.mu << std::endl;
	std::cout << "---- lambda            is set to " << ap.lambda << std::endl;
	std::cout << "---- nbElite           is set to " << ap.nbElite << std::endl;
	std::cout << "---- nbClose           is set to " << ap.nbClose << std::endl;
	std::cout << "---- targetFeasible    is set to " << ap.targetFeasible << std::endl;
	std::cout << "---- seed              is set to " << ap.seed << std::endl;
	std::cout << "---- nbIter            is set to " << ap.nbIter << std::endl;
	std::cout << "---- timeLimit         is set to " << ap.timeLimit << std::endl;
	std::cout << "---- isRoundingInteger is set to " << static_cast<unsigned>(ap.isRoundingInteger) << std::endl;
	std::cout << "==================================================" << std::endl;
}