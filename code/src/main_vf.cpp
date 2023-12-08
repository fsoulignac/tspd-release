//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include <iostream>
#include <vector>
#include <goc/goc.h>

#include "tspd_instance.h"
#include "solver.h"
#include "ng_utils.h"
#include "time_guardian.h"

using namespace std;
using namespace goc;
using namespace nlohmann;
using namespace tspd;


int main()
{
    json output;
    simulate_runner_input("../instances/poi/", "poi-20-1-1", "../experiments/vf.json", "var-fix");

    json experiment, instance_json, solutions;
    cin >> experiment >> instance_json >> solutions;

    auto time_limit_factor = value_or_default(experiment, "time_limit_factor", 20.0);
    auto init_ng = value_or_default(experiment, "init_ng", 3);
    auto gaps = value_or_default(experiment, "gaps", std::vector<double>());
    auto drone_speed = value_or_default(experiment, "drone_speed", 1.0);
    
    clog << "Experiment parameters" << endl 
         << "Time limit factor: " << time_limit_factor
         << ", Init NG size: " << init_ng 
         << ", Gaps: " << gaps;

    clog << "Parsing instance..." << endl;
    time_guardian.SetTimeLimit(Duration(3600, DurationUnit::Seconds));
    TSPDInput tsp_input = instance_json;
    TSPDInstance tsp(tsp_input, drone_speed);

    auto b_tsp = tsp.Reverse();
    
    try{
        Subinstance instance(&tsp, &b_tsp);
        Solver solver(instance, NearbyNG(instance, init_ng));
        auto log = solver.VariableFixingExperiment(gaps, time_limit_factor);
        
        output["solution"] = log.duals;
        output["solver"] = log;
	}
    catch (TimeLimitExceeded& tle)
    {
        clog << "Error, esto no deberia haber pasado." << endl;
        return 1;
    }
	catch (std::bad_alloc& e)
	{
		return 3; // Memory limit exceeded code for runner.
	}
        
    cout << std::setw(4) << output << endl;

	return 0;
}
