//
// Created by chkwon on 3/23/22.
//

// This header file must be readable in C.

#ifndef ALGORITHMPARAMETERS_H
#define ALGORITHMPARAMETERS_H

struct AlgorithmParameters {
	int nbGranular;			// Granular search parameter, limits the number of moves in the RI local search
	int mu;					// Minimum population size
	int lambda;				// Number of solutions created before reaching the maximum population size (i.e., generation size)
	int nbElite;			// Number of elite individuals (reduced in HGS-2020)
	int nbClose;			// Number of closest solutions/individuals considered when calculating diversity contribution
	double targetFeasible;	// Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters

	int seed;				// Random seed. Default value: 0
	int nbIter;				// Number of iterations without improvement until termination. Default value: 20,000 iterations
	double timeLimit;		// CPU time limit until termination in seconds. Default value: infinity
	char isRoundingInteger; // rounding the distances or not. Default value: 1. This is char, not bool, for C compatibility
};


#ifdef __cplusplus
extern "C"
#endif
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
void print_algorithm_parameters(const AlgorithmParameters & ap);
#endif



#endif //ALGORITHMPARAMETERS_H
