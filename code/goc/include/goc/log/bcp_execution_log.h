//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_BCP_EXECUTION_LOG_H
#define GOC_LOG_BCP_EXECUTION_LOG_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/linear_programming/model/valuation.h"
#include "goc/log/bc_execution_log.h"
#include "goc/time/duration.h"

namespace goc
{
// This class stores information about the execution of a branch-cut-price solver.
// - It is JSON serializable and compatible with the Kaleidoscope kd_type "bcp".
class BCPExecutionLog : public BCExecutionLog
{
public:
	Duration lp_time; // time spent solving linear relaxations.
	Duration pricing_time; // time spent solving pricing problems.
	Duration branching_time; // time spent branching.
	
	BCPExecutionLog();
	
	virtual nlohmann::json ToJSON() const;
};
} // namespace goc

#endif //GOC_LOG_BCP_EXECUTION_LOG_H
