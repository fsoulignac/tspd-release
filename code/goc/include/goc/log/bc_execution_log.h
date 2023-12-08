//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_BC_EXECUTION_LOG_H
#define GOC_LOG_BC_EXECUTION_LOG_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/linear_programming/model/valuation.h"
#include "goc/log/log.h"
#include "goc/time/duration.h"

namespace goc
{
// All the status that can result from a branch and cut execution.
enum class BCStatus {
	DidNotStart, Infeasible, Unbounded, TimeLimitReached, MemoryLimitReached, Optimum, NodeLimitReached
};

// This class stores information about the execution of a branch-and-cut solver.
// - It is JSON serializable and compatible with the Kaleidoscope kd_type "bc".
class BCExecutionLog : public Log
{
public:
	std::string screen_output; // output of the algorithm in the screen.
	Duration time; // total time spent solving the problem.
	BCStatus status; // the status of the execution.
	int nodes_open; // number of open nodes in the BB tree at the end of the execution.
	int nodes_closed; // number of nodes closed during the BB.
	double root_lp_value; // objective value of the root node relaxation (after cuts).
	double root_int_value; // value of the integer solution found at the root node.
	double best_bound; // best dual bound found for the problem.
	double best_int_value; // value of the best integer solution found.
	Duration cut_time; // total time spent separating cuts.
	
	BCExecutionLog();
	
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, BCStatus status);
} // namespace goc

#endif //GOC_LOG_BC_EXECUTION_LOG_H
