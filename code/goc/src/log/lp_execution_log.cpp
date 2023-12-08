//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/log/lp_execution_log.h"

#include <unordered_map>

#include "goc/string/string_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
LPExecutionLog::LPExecutionLog()
{
	screen_output = "";
	time = 0.0_sec;
	status = LPStatus::DidNotStart;
	simplex_iterations = 0;
	incumbent_value = 0.0;
	variable_count = 0;
}

json LPExecutionLog::ToJSON() const
{
	json j;
	j["kd_type"] = "lp"; // ID of the log type.
	j["screen_output"] = screen_output;
	j["time"] = time.Amount(DurationUnit::Seconds);
	j["status"] = STR(status);
	j["simplex_iterations"] = simplex_iterations;
	j["incumbent"] = incumbent;
	j["incumbent_value"] = incumbent_value;
	j["constraint_count"] = constraint_count;
	j["variable_count"] = variable_count;
	j["duals"] = duals;
	return j;
}

ostream& operator<<(ostream& os, LPStatus status)
{
	unordered_map<LPStatus, string> mapper = {{LPStatus::DidNotStart, "DidNotStart"},
											  {LPStatus::Infeasible, "Infeasible"},
											  {LPStatus::Unbounded, "Unbounded"},
											  {LPStatus::TimeLimitReached, "TimeLimitReached"},
											  {LPStatus::MemoryLimitReached, "MemoryLimitReached"},
											  {LPStatus::Optimum, "Optimum"}};
	return os << mapper[status];
}
} // namespace goc