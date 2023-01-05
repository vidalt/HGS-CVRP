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

#include "Decomposition.h"
#include "DecompositionClustering.h"
#include "Genetic.h"
#include <algorithm>
#include <cassert>
#include <chrono>

void Decomposition::decompose(double timeElapsed) {
  using std::chrono::steady_clock, std::chrono::duration_cast, std::chrono::seconds;

  if(params.nbClients <= params.ap.decoTargetSz) {
    // Instance is not large enough for decomposition.
    return;
  }

  const auto& eliteIndividual = population.getBinaryTournament();
  auto subproblems = getSubproblems(eliteIndividual, timeElapsed);

  if(subproblems.size() < 2u) {
    // Only 1 subproblem!
    return;
  }

  // New MP individual we want to build, starting from the SP solutions.
  Individual mpIndividual{params};
  mpIndividual.chromR.clear();
  mpIndividual.chromT.clear();

  double timeSpentInDeco = 0.0;

  for(auto& sp : subproblems) {
    sp->decreaseTimeout(timeSpentInDeco);

    const auto startTime = steady_clock::now();
    const auto solution = sp->solve();
    const auto endTime = steady_clock::now();

    timeSpentInDeco += duration_cast<seconds>(endTime - startTime).count();

    mergeSolutionIntoMpIndividual(solution, mpIndividual, *sp);
  }

  // Add empty routes in case the number of used vehicles decreased.
  for(auto r = mpIndividual.chromR.size(); r < params.nbVehicles; ++r) {
    mpIndividual.chromR.emplace_back();
  }

  // Evaluate the costs of the new MP individual.
  mpIndividual.evaluateCompleteCost(params);

  // Add the new MP individual to the population.
  population.addIndividual(mpIndividual, true);
}

std::vector<std::unique_ptr<SubProblem>> Decomposition::getSubproblems(
    const Individual &eliteIndividual,
    double timeElapsed) const
{
  using namespace kmeans;

  const auto k = (std::size_t) std::ceil((double) params.nbClients / (double) params.ap.decoTargetSz);

  auto emptyRoutes = std::vector<int>{};
  for(auto i = 0u; i < eliteIndividual.chromR.size(); ++i) {
    if(eliteIndividual.chromR[i].empty()) {
      emptyRoutes.push_back(i);
    }
  }

  // Get the clustering of the routes' barycentres via k-means.
  const auto clustering = kMeans(k, eliteIndividual.barycentres, emptyRoutes);
  assert(clustering.size() == k);

  // We will now proceed to create k subproblems, one for each cluster.
  auto sps = std::vector<std::unique_ptr<SubProblem>>{};
  sps.reserve(k);

  // Cycle through each cluster. A cluster is a collection of indices
  // which index routes in base_individual.
  for(const auto& cluster : clustering) {
    if(!cluster.empty()) {
      // Create a subproblem with the routes indexed by the cluster.
      sps.push_back(std::make_unique<SubProblem>(params, eliteIndividual, cluster, timeElapsed));
    }
  }

  return sps;
}

void Decomposition::mergeSolutionIntoMpIndividual(const Individual &spSolution,
                                                  Individual &mpIndividual,
                                                  const SubProblem &sp)
{
  const auto& eliteIndividual = sp.getEliteIndividual();
  double eliteCost = 0.0;

  // We calculate eliteCost, i.e. the cost of the routes used to build the
  // sub-problem sp. If the SP produced a better solution, we use it. Otherwise,
  // we just re-add the elite indvidual's routes to mpIndividual.
  for(auto routeId : sp.getEliteRoutes()) {
    const auto &route = eliteIndividual.chromR[routeId];

    if(route.empty()) {
      continue;
    }

    // Depot -> first client.
    eliteCost += params.timeCost[0][route[0]];
    // Client -> client.
    for(auto custPos = 0u; custPos < route.size() - 1u; ++custPos) {
      eliteCost += params.timeCost[route[custPos]][route[custPos + 1u]];
    }
    // Last client -> Depot.
    eliteCost += params.timeCost[route.back()][0];
  }

  if(eliteCost > spSolution.eval.penalizedCost) {
    // SP solution is better!
    for(const auto& route : spSolution.chromR) {
      mpIndividual.chromR.emplace_back();
      for(auto spCust : route) {
        auto mpCust = sp.spToMpCustomer(spCust);
        mpIndividual.chromR.back().push_back(mpCust);
        mpIndividual.chromT.push_back(mpCust);
      }
    }
  } else {
    // Routes from Elite Individual are better!
    for(auto routeId : sp.getEliteRoutes()) {
      mpIndividual.chromR.emplace_back();
      for(auto mpCust : eliteIndividual.chromR[routeId]) {
        mpIndividual.chromR.back().emplace_back(mpCust);
        mpIndividual.chromT.push_back(mpCust);
      }
    }
  }
}

void SubProblem::initSubproblem() {
  spParams.nbClients = 0;
  spParams.totalDemand = 0.0;
  spParams.maxDemand = 0.0;

  // The only node which is kept for sure is the depot.
  spParams.cli = {mpParams.cli.at(0u)};

  // The SP has as many vehicles as there are routes.
  spParams.nbVehicles = (int) eliteRoutes.size();

  // The depot maps to itself.
  spToMpCustomers[0] = 0;
  mpToSpCustomers[0] = 0;
}

void SubProblem::addRoute(int routeId) {
  const auto& route = eliteIndividual.chromR.at(routeId);

  for(auto customer : route) {
    const auto& client = mpParams.cli.at(customer);

    spParams.nbClients++;
    mpToSpCustomers[customer] = spParams.nbClients;
    spToMpCustomers[spParams.nbClients] = customer;
    spParams.cli.push_back(client);
    spParams.totalDemand += client.demand;
    spParams.maxDemand = std::max(spParams.maxDemand, client.demand);
  }
}

void SubProblem::calculateDistances() {
  spParams.maxDist = 0.0;

  const auto sz = spParams.nbClients + 1;
  spParams.timeCost = std::vector<std::vector<double>>(sz, std::vector<double>(sz));

  // Go through the customers (and depot) added to the subproblem, and get
  // the subproblem distance matrix from the master problem one, being careful
  // that we need to use the correct mapped indices.
  for(auto i = 0; i < sz; ++i) {
    // Distance to self is zero.
    spParams.timeCost.at(i).at(i) = 0.0;

    for(auto j = i + 1; j < sz; ++j) {
      // Map the subproblem customers to the corresponding
      // master problem customers.
      const auto mpI = spToMpCustomers.at(i);
      const auto mpJ = spToMpCustomers.at(j);

      // Get the distance from the master problem.
      const auto dist = mpParams.timeCost.at(mpI).at(mpJ);

      // Since we deal with symmetric problems, we can just
      // update both distances at the same time.
      spParams.timeCost.at(i).at(j) = dist;
      spParams.timeCost.at(j).at(i) = dist;

      // Update the maximum distance encountered in the subproblem.
      spParams.maxDist = std::max(spParams.maxDist, dist);
    }
  }
}

void SubProblem::calculateProximity() {
  // Calculate the number of proximal vertices for the subproblem. This number is given by the parameter
  // nbGranular, or just by the number of customers in the subproblem if it is lower.
  const auto nProximity = std::min(spParams.ap.nbGranular, spParams.nbClients - 1);

  // Clear the proximal vertices list and only add an empty entry for the depot.
  spParams.correlatedVertices = {{}};

  // For each customer (not the depot, notice the loop starts from 1) in the subproblem...
  for(auto i = 1; i <= spParams.nbClients; ++i) {
    // Use this sorting criterion to sort the customers, relative to
    // customer i. The criterion is simply: if j1 is closer to i than
    // j2 is, then j1 precedes j2.
    auto distanceSort = [this, i] (int j1, int j2) -> bool {
      return spParams.timeCost.at(i).at(j1) < spParams.timeCost.at(i).at(j2);
    };

    // Get a list of all vertices in the subproblem excluding the depot (notice that iota starts from 1).
    auto allVertices = std::vector<int>(spParams.nbClients);
    std::iota(allVertices.begin(), allVertices.end(), 1);

    // Sort them by proximity to customer i.
    std::sort(allVertices.begin(), allVertices.end(), distanceSort);

    // Nothing should be closer to i than i itself.
    assert(allVertices.at(0) == i);

    // Add the first nProximity customers to the proximity list for i. Notice that we start from
    // .begin() + 1 to exclude i itself, which is always the first element (see assert above).
    // Also notice that by definition of nProximity, .begin() + nProximity cannot be out of bounds:
    // at worst, it is equal to .end() when nProximity == sp_params.nb_clients and all customers
    // go into the proximity list.
    if(nProximity > 0) {
      spParams.correlatedVertices.emplace_back(allVertices.begin() + 1, allVertices.begin() + nProximity);
    } else {
      spParams.correlatedVertices.emplace_back();
    }
  }
}

SubProblem::SubProblem(const Params &mpParams,
                       const Individual &eliteIndividual,
                       std::vector<int> eliteRoutes,
                       double timeElapsed) :
    mpParams{mpParams},
    eliteIndividual{eliteIndividual},
    eliteRoutes{std::move(eliteRoutes)}
{
  // Zero-intialise spParams.
  initSubproblem();

  // Add customers from the relevant routes.
  for(auto routeId : this->eliteRoutes) {
    addRoute(routeId);
  }

  // Recompute distance and proximity matrices.
  calculateDistances();
  calculateProximity();

  // Sets the termination criterion.
  spParams.ap.nbIter = mpParams.ap.decoNbIter;
  spParams.ap.timeLimit = std::max(0.0, mpParams.ap.timeLimit - timeElapsed);

  // Solving the SP requires reducing some MP params.
  spParams.ap.nbElite = mpParams.ap.nbElite / 2;
  spParams.ap.lambda = mpParams.ap.lambda / 2;
  spParams.ap.mu = mpParams.ap.mu / 2;

  // Copy other params
  spParams.ap.nbGranular = mpParams.ap.nbGranular;
  spParams.ap.nbClose = mpParams.ap.nbClose;
  spParams.ap.targetFeasible = mpParams.ap.targetFeasible;
  spParams.ap.useSwapStar = mpParams.ap.useSwapStar;
  spParams.ap.nbIterPenaltyManagement = mpParams.ap.nbIterPenaltyManagement;
  spParams.ap.penaltyDecrease = mpParams.ap.penaltyDecrease;
  spParams.ap.penaltyIncrease = mpParams.ap.penaltyIncrease;
  spParams.ap.nbIterTraces = mpParams.ap.nbIterTraces;
  spParams.isDurationConstraint = mpParams.isDurationConstraint;
  spParams.areCoordinatesProvided = mpParams.areCoordinatesProvided;
  spParams.durationLimit = mpParams.durationLimit;
  spParams.vehicleCapacity = mpParams.vehicleCapacity;
  spParams.penaltyDuration = mpParams.penaltyDuration;
  spParams.penaltyCapacity = mpParams.penaltyCapacity;

  // Disable output printing
  spParams.verbose = false;

  // Re-init the clock
  spParams.startTime = clock();

  // Turn off nested decomposition
  spParams.ap.useDecomposition = 0;
}

Individual SubProblem::solve() {
  if(mpParams.verbose) {
    std::cerr << "Solving a SP with " << spParams.nbClients << " customers and " << spParams.nbVehicles << " vehicles\n";
  }

  if(spParams.nbClients == 0 || spParams.nbVehicles == 0) {
    // Empty sub-problem: nothing to do!
    return Individual{spParams};
  }

  Genetic solver{spParams};
  solver.run();

  const Individual* best = solver.population.getBestFound();

  if(best) {
    // Returns the best sub-problem individual.
    return *best;
  }

  // Sub-problem could not find a solution!
  return Individual{spParams};
}