/*MIT License

Copyright(c) 2022 Alberto Santini

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef HGS_CVRP_DECOMPOSITION_H
#define HGS_CVRP_DECOMPOSITION_H

#include <map>
#include <memory>
#include <vector>
#include <cassert>

#include "Params.h"
#include "Individual.h"
#include "Population.h"

class SubProblem;

// Implements Barycenter Clustering decomposition, as explained in:
// https://santini.in/files/papers/santini-schneider-vidal-vigo-2022.pdf
class Decomposition {
  const Params& params;
  Population& population;

  // Gives a list of subproblems to solve.
  std::vector<std::unique_ptr<SubProblem>> getSubproblems(const Individual& eliteIndividual, double timeElapsed) const;

  // Merges an SP solution spSolution into an MP solution mpSolution.
  // It also needs access to the subproblem, via param sp.
  void mergeSolutionIntoMpIndividual(const Individual& spSolution, Individual& mpIndividual, const SubProblem& sp);

public:
  // Performs decomposition starting from the routes of an elite individual
  // chosen from the MP population.
  // It ensures that sub-problems do not exceed the remaining time.
  // This method also takes care of adding to the master problem's population
  // the new solution created by the decomposed sub-problems.
  void decompose(double timeElapsed);

  explicit Decomposition(const Params& params, Population& population) :
    params{params}, population{population} {}
};

// Represent one of the sub-problems solved by decomposition.
class SubProblem {
  // Master Problem original params.
  const Params& mpParams;

  // Sub-Problem newly created params.
  Params spParams;

  // Elite individual used to build the sub-problem.
  const Individual& eliteIndividual;

  // Bidirectional mapping of MP <-> SP customer numbers.
  std::map<int, int> mpToSpCustomers;
  std::map<int, int> spToMpCustomers;

  // Indices of eliteIndividual's routes to include in
  // the sub-problem. I.e., the sub-problem's customers will
  // be exactly those visited by these routes.
  std::vector<int> eliteRoutes;

  // Zero-initialises SP params.
  void initSubproblem();

  // Adds customers from a route into the SP.
  void addRoute(int routeId);

  // Builds the distance matrix for the SP.
  void calculateDistances();

  // Builds the proximity matrix for the SP.
  void calculateProximity();

public:
  explicit SubProblem(
      const Params& mpParams,
      const Individual& eliteIndividual,
      std::vector<int> eliteRoutes,
      double timeElapsed);

  // Solves the sub-problem and returns the best SP individual.
  Individual solve();

  const std::vector<int>& getEliteRoutes() const { return eliteRoutes; }
  const Individual& getEliteIndividual() const { return eliteIndividual; }
  int spToMpCustomer(int spCust) const { return spToMpCustomers.at(spCust); }
  void decreaseTimeout(double t) { spParams.ap.timeLimit = std::max(spParams.ap.timeLimit - t, 0.0); }
  double getTimeLimit() const { return spParams.ap.timeLimit; }
};


#endif //HGS_CVRP_DECOMPOSITION_H
