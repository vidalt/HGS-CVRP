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
	int nbElite;			// Number of elite individuals
	int nbClose;			// Number of closest solutions/individuals considered when calculating diversity contribution
	double targetFeasible;	// Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters

	int seed;				// Random seed. Default value: 0
	int nbIter;				// Nb iterations without improvement until termination (or restart if a time limit is specified). Default value: 20,000 iterations
	double timeLimit;		// CPU time limit until termination in seconds. Default value: 0 (i.e., inactive)
	int useSwapStar;		// Use SWAP* local search or not. Default value: 1. Only available when coordinates are provided.

  int useDecomposition;   // If 1, large (> 300 customers) problems are decomposed using Barycentre Clustering Decomposition.
                          // See: https://santini.in/files/papers/santini-schneider-vidal-vigo-2022.pdf
  int decoIterations;     // Decompose the problem each decoIterations iterations.
  int decoTargetSz;       // Try to create subproblems with this many customers.
  int decoNbIter;         // Param nbIter, but for the sub-problem.
};


#ifdef __cplusplus
extern "C"
#endif
struct AlgorithmParameters default_algorithm_parameters();

#ifdef __cplusplus
void print_algorithm_parameters(const AlgorithmParameters & ap);
#endif



#endif //ALGORITHMPARAMETERS_H
