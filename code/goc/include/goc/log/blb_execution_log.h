//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_BLB_EXECUTION_LOG_H
#define GOC_LOG_BLB_EXECUTION_LOG_H

#include <iostream>
#include <string>

#include "goc/lib/json.hpp"
#include "goc/log/mlb_execution_log.h"
#include "goc/log/log.h"
#include "goc/time/duration.h"

namespace goc
{
// All the status that can result from a monodirectional labeling algorithm.
enum class BLBStatus { DidNotStart, TimeLimitReached, SolutionLimitReached, Finished };

// This class stores information about the execution of a BIDIRECTIONAL labeling algorithm.
// It is compatible with the Kaleidoscope kd_type "blb".
class BLBExecutionLog : public Log
{
public:
	BLBStatus status; // status at the end of the execution.
	Duration time; // total execution time.
	std::string screen_output; // output of the algorithm in the screen.
	MLBExecutionLog forward_log; // log of the forward labeling.
	MLBExecutionLog backward_log; // log of the backward labeling.
	Duration merge_time; // time spent merging labels.

	BLBExecutionLog();
	
	// Serialize log.
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, BLBStatus status);
} // namespace goc

#endif //GOC_LOG_BLB_EXECUTION_LOG_H
