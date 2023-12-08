//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/log/mlb_execution_log.h"

#include <unordered_map>

#include "goc/string/string_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
MLBExecutionLog::MLBExecutionLog()
{
	screen_output = "";
	time = 0.0_hr;
	status = MLBStatus::DidNotStart;
	bounded_count = enumerated_count = dominated_count = processed_count = 0;
	bounding_time = queuing_time = enumeration_time = domination_time = process_time = 0.0_hr;
}

json MLBExecutionLog::ToJSON() const
{
	json j;
	j["kd_type"] = "mlb"; // ID of the log type.
	j["screen_output"] = screen_output;
	j["time"] = time;
	j["status"] = STR(status);
	j["enumerated_count"] = enumerated_count;
	j["dominated_count"] = dominated_count;
	j["processed_count"] = processed_count;
	j["bounded_count"] = bounded_count;
	j["count_by_length"] = count_by_length;
	j["queuing_time"] = queuing_time;
	j["enumeration_time"] = enumeration_time;
	j["domination_time"] = domination_time;
	j["process_time"] = process_time;
	j["bounding_time"] = bounding_time;
	return j;
}

ostream& operator<<(ostream& os, MLBStatus status)
{
	unordered_map<MLBStatus, string> mapper = {{MLBStatus::DidNotStart, "DidNotStart"},
											  {MLBStatus::TimeLimitReached, "TimeLimitReached"},
											  {MLBStatus::ProcessLimitReached, "ProcessLimitReached"},
											  {MLBStatus::Finished, "Finished"}};
	return os << mapper[status];
}
} // namespace goc