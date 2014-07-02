/*! 
 *  \author    PraveenKumar Vasudevan
 *  \date      2014
 */
#ifndef SimulationEngine_H
#define SimulationEngine_H

#include <vector>
#include <string>
#include "MOMDP.h"
using namespace std;
using namespace momdp;

namespace momdp
{
    class AlphaVectorPolicy;
    class SolverParams;

    /**
     * @brief Policy Simulation Engine class
     **/
    class SimulationEngine
    {
        private:
            SharedPointer<MOMDP> problem;
            SharedPointer<AlphaVectorPolicy> policy;
            SolverParams * solverParams;


        public:
	  /**
	   * @brief Constructor
	   *
	   **/
	  SimulationEngine();
	  /**
	   * @brief Setup the simulator
	   *
	   * @param problem Reference to the problem
	   * @param policy Reference to the policy
	   * @param solverParams Solver parameters
	   * @return void
	   **/
	  void setup(SharedPointer<MOMDP> problem, SharedPointer<AlphaVectorPolicy> policy, SolverParams * solverParams);
	  /**
	   * @brief Perform action on full observable system
	   *
	   * @param outBelObs Output Belief state
	   * @param action Action index
	   * @param belSt Belief state
	   * @return void
	   **/
	  void performActionObs(belief_vector& outBelObs, int action, const BeliefWithState& belSt) const;
	  /**
	   * @brief Perform action with no observablility
	   *
	   * @param outBelUnobs Out belief state
	   * @param action Action index
	   * @param belSt Belief state
	   * @param currObsState Current observed state
	   * @return void
	   **/
	  void performActionUnobs(belief_vector& outBelUnobs, int action, const BeliefWithState& belSt, int currObsState) const;
	  /**
	   * @brief Get possible observations
	   *
	   * @param possObs Possible obsevation vector
	   * @param action Action index
	   * @param belSt Belief state
	   * @return void
	   **/
	  void getPossibleObservations(belief_vector& possObs, int action, const BeliefWithState& belSt) const;
	  /**
	   * @brief String representation
	   *
	   * @return :string - string value
	   **/
	  string toString();

	  /**
	   * @brief Return reward given a Belief and action
	   *
	   * @param belst Belief state
	   * @param action Action
	   * @return double - Reward value
	   **/
	  double getReward(const BeliefWithState& belst, int action);

	  /**
	   * @brief Check terminal
	   *
	   * @param o ...
	   * @param s ...
	   * @param bhout ...
	   * @param fhout ...
	   * @return void
	   **/
	  void checkTerminal(string o, string s, vector<int> &bhout, vector<int> &fhout);
	  /**
	   * @brief Get greedy action
	   *
	   * @param  ...
	   * @param  ...
	   * @return int
	   **/
	  int getGreedyAction(vector<int> &, vector<int> &);



	  /**
	   * @brief Display belief vector on an output stream
	   *
	   * @param b Belief vector
	   * @param s Output stream
	   * @return void
	   **/
	  void display(belief_vector& b, ostream& s);
	  /**
	   * @brief Run the simulator for a given number of steps
	   *
	   * @param iters Iterations count
	   * @param streamOut output stream
	   * @param reward reward value
	   * @param expReward expected reward value
	   * @return int - BEst action
	   **/
	  int runFor(int iters, ofstream* streamOut, double& reward, double& expReward);
	  /**
	   * @brief Destructor
	   *
	   * @param  ...
	   **/
	  virtual ~SimulationEngine(void);
    };



}
#endif

