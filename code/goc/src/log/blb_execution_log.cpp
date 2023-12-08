//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/log/blb_execution_log.h"

#include <unordered_map>

#include "goc/string/string_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
BLBExecutionLog::BLBExecutionLog()
{
	status = BLBStatus::DidNotStart;
	time = merge_time = 0.0_sec;
	screen_output = "";
}

json BLBExecutionLog::ToJSON() const
{
	json j;
	j["kd_type"] = "blb"; // ID of the log type.
	j["screen_output"] = screen_output;
	j["time"] = time;
	j["status"] = STR(status);
	j["forward"] = forward_log;
	j["backward"] = backward_log;
	j["merge_time"] = merge_time;
	
	return j;
}

ostream& operator<<(ostream& os, BLBStatus status)
{
	unordered_map<BLBStatus, string> mapper = {{BLBStatus::DidNotStart, "DidNotStart"},
											   {BLBStatus::TimeLimitReached, "TimeLimitReached"},
											   {BLBStatus::SolutionLimitReached, "SolutionLimitReached"},
											   {BLBStatus::Finished, "Finished"}};
	return os << mapper[status];
}
} // namespace goc