//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_CG_EXECUTION_LOG_H
#define GOC_LOG_CG_EXECUTION_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/linear_programming/model/valuation.h"
#include "goc/log/log.h"
#include "goc/time/duration.h"

namespace goc
{
// All the status that can result from a column generation execution.
enum class CGStatus { DidNotStart, Infeasible, Unbounded, TimeLimitReached, MemoryLimitReached, Optimum };

// This class stores information about the execution of a column generation algorithm.
// It is compatible with the Kaleidoscope kd_type "cg".
class CGExecutionLog : public Log
{
public:
	std::string screen_output; // output of the algorithm in the screen.
	Duration time; // total time spent solving.
	CGStatus status; // the status of the execution
	Valuation incumbent; // best solution found.
	double incumbent_value; // value of the best solution found.
	int columns_added; // total number of columns added in the colgen.
	int iteration_count; // number of pricing iterations solved.
	Duration pricing_time; // time spent solving the pricing problem.
	Duration lp_time; // time spent solving the lp relaxation.
	std::vector<nlohmann::json> iterations; // logs of the pricing iterations.
	
	CGExecutionLog();
	
	// Serialize log.
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, CGStatus status);
} // namespace goc

#endif //GOC_LOG_CG_EXECUTION_LOG_H
