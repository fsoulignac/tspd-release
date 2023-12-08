//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/log/cg_execution_log.h"

#include "goc/string/string_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
CGExecutionLog::CGExecutionLog()
{
	screen_output = "";
	time = 0.0_sec;
	status = CGStatus::DidNotStart;
	incumbent_value = 0.0;
	columns_added = iteration_count = 0;
	pricing_time = lp_time = 0.0_sec;
}

json CGExecutionLog::ToJSON() const
{
	json j;
	j["kd_type"] = "cg"; // ID of the log type.
	j["screen_output"] = screen_output;
	j["time"] = time.Amount(DurationUnit::Seconds);
	j["status"] = STR(status);
	j["incumbent"] = incumbent;
	j["incumbent_value"] = incumbent_value;
	j["columns_added"] = columns_added;
	j["iteration_count"] = iteration_count;
	j["pricing_time"] = pricing_time;
	j["lp_time"] = lp_time;
	j["iterations"] = iterations;
	
	return j;
}

ostream& operator<<(ostream& os, CGStatus status)
{
	unordered_map<CGStatus, string> mapper = {{CGStatus::DidNotStart, "DidNotStart"},
											  {CGStatus::Infeasible, "Infeasible"},
											  {CGStatus::Unbounded, "Unbounded"},
											  {CGStatus::TimeLimitReached, "TimeLimitReached"},
											  {CGStatus::MemoryLimitReached, "MemoryLimitReached"},
											  {CGStatus::Optimum, "Optimum"}};
	return os << mapper[status];
}
} // namespace goc