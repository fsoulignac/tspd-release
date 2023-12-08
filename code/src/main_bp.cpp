//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include <iostream>
#include <vector>
#include <goc/goc.h>

#include "tspd_instance.h"
#include "route.h"
#include "bp_solver.h"
#include "solver.h"
#include "ng_utils.h"
#include "time_guardian.h"

using namespace std;
using namespace goc;
using namespace nlohmann;
using namespace tspd;

int main()
{
    simulate_runner_input("../instances/poi/", "poi-10-1-2", "../experiments/all.json", "full");

    json experiment, instance_json, solutions;
    cin >> experiment >> instance_json >> solutions;

    auto time_limit = Duration(value_or_default(experiment, "time_limit", 600), DurationUnit::Seconds);
    auto init_ng = value_or_default(experiment, "init_ng", 3);
    auto cg_algorithm_id = value_or_default(experiment, "cg_algorithm", 0);
    auto bidirectional = value_or_default(experiment, "bidirectional", true);
    auto drone_speed = value_or_default(experiment, "drone_speed", 1.0);
    auto dominance_level_id = value_or_default(experiment, "dominance_level", 0);
    auto only_root = value_or_default(experiment, "only_root", false);
    auto loops = value_or_default(experiment, "loops", false);
    auto compatible = value_or_default(experiment, "compatible", 0);
    auto truck_serves = value_or_default(experiment, "truck_serves", true);
    auto launch_time = value_or_default(experiment, "launch_time", 0.0);
    auto rendezvous_time = value_or_default(experiment, "rendezvous_time", 0.0);
    auto range = value_or_default(experiment, "range", numeric_limits<double>::infinity());
    auto log_level_id = value_or_default(experiment, "log_level", 0);

    DominanceLevel dominance_level = DominanceLevel::Fork;
    if(dominance_level_id == 1) dominance_level = DominanceLevel::NoFork;

    CGAlgorithm cg_algorithm = CGAlgorithm::Basic;
    if(cg_algorithm_id == 1) cg_algorithm = CGAlgorithm::Plus;

    BPLogLevel log_level = BPLogLevel::None;
    if(log_level_id == 1) log_level = BPLogLevel::Root;
    if(log_level_id == 2) log_level = BPLogLevel::All;
    
    clog << "Experiment parameters" << endl 
         << "Time limit: " << time_limit
         << ", Init NG size: " << init_ng 
         << ", Loops: " << loops
         << ", Launch time: " << launch_time
         << ", Rendezvous time: " << rendezvous_time
         << ", Flying range: " << range
         << ", Drone Compatibility Level: " << compatible
         << ", Truck serves alone?: " << truck_serves
         << ", CG Algorithm: " << cg_algorithm 
         << ", Only root? " << only_root
         << (bidirectional ? string(", Bidirectional search") : string(", Forward search"))
         << ", Dominance Level: " << dominance_level << endl;

    clog << "Parsing instance..." << endl;
    time_guardian.SetTimeLimit(time_limit);
    TSPDInput tsp_input = instance_json;
    TSPDInstance tsp(tsp_input, drone_speed);


    json output;
    auto b_tsp = tsp.Reverse();

    Subinstance instance(&tsp, &b_tsp, loops, compatible, truck_serves, launch_time, rendezvous_time, range);
    BPSolver bp_solver(instance, NearbyNG(instance, init_ng));
	try
	{
        bp_solver.solve_only_root = only_root;
        bp_solver.log_level = log_level;
        bp_solver.Run(cg_algorithm, bidirectional, dominance_level);

        clog << bp_solver.BestSolution() << endl;
        output["solution"] = bp_solver.BestSolution();
        output["solver"] = bp_solver.Log();
    }
    catch (TimeLimitExceeded& tle)
    {
        clog << "Time limit reached." << endl;
        output["solver"] = bp_solver.Log();
    }
	catch (std::bad_alloc& e)
	{
		return 3; // Memory limit exceeded code for runner.
	}

	cout << std::setw(4) << output << endl;
    return 0;
}
