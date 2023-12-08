//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "log/solver_problem_log.h"

using namespace std;
using namespace nlohmann;
using namespace goc;

namespace tspd
{

json SolverProblemLog::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_solver_problem";
	j["time"] = time;
	j["lb"] = lb;
	j["start_ub"] = start_ub;
	j["ub"] = ub;
	j["start_ng"] = start_ng;
    j["final_ng"] = final_ng;
	j["dna_time"] = DNA_time;
	j["lp_time"] = lp_time;
	j["pricing_time"] = pricing_time;
	j["fix_time"] = variable_fixing_time;
    j["iterative_fix_time"] = iterative_fixing_time;
	j["labelings"] = labelings;
	j["exact_labelings"] = exact_labelings;
	j["exact_labelings_retries"] = exact_labelings_plus;
	return j;
}

} // namespace tspd
