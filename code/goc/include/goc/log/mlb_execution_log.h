//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_MLB_EXECUTION_LOG_H
#define GOC_LOG_MLB_EXECUTION_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/log/log.h"
#include "goc/time/duration.h"

namespace goc
{
// All the status that can result from a monodirectional labeling algorithm.
enum class MLBStatus { DidNotStart, TimeLimitReached, ProcessLimitReached, Finished };

// This class stores information about the execution of a MONODIRECTIONAL labeling algorithm.
// We consider that a label in the labeling algorithm has to pass through the following stages:
//	1 - enumeration: label is enumerated.
//  2 - queuing: enumerated labels that are not bounded or dominated are put into a queue.
//	3 - domination: check if a label is dominated.
//  4 - bounded: check if the completion bound is below an upper bound.
//	5 - process: the label is not dominated and thus it is inserted to the undominated structure.
// It is compatible with the Kaleidoscope kd_type "mlb".
class MLBExecutionLog : public Log
{
public:
	std::string screen_output; // output of the algorithm in the screen.
	Duration time; // total time spent solving the problem.
	MLBStatus status = MLBStatus::DidNotStart; // the status of the execution.
	int enumerated_count = 0; // number of labels enumerated.
	int dominated_count = 0; // number of labels dominated.
	int processed_count = 0; // number of labels processed.
	int bounded_count = 0; // number of labels bounded.
	std::vector<int> count_by_length; // count_by_length[i] indicates how many labels of length i were processed.
	Duration queuing_time; // time pushing and popping from the queue, including time to dominate within the queue.
	Duration enumeration_time; // time spent in the enumeration phase.
	Duration domination_time; // time spent in the domination phase, excluding the time to dominate within the queue.
	Duration process_time; // time spent in the process phase.
	Duration bounding_time; // time spent in the bounding phase.

	MLBExecutionLog();
	
	// Serialize log.
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, MLBStatus status);
} // namespace goc

#endif //GOC_LOG_MLB_EXECUTION_LOG_H
