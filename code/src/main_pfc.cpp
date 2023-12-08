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
    simulate_runner_input("../instances/poi/", "poi-20-1", "../experiments/best.json", "pfc-5-speed-1");

    json experiment, instance_json, solutions;
    cin >> experiment >> instance_json >> solutions;

    auto time_limit = Duration(value_or_default(experiment, "time_limit", 600), DurationUnit::Seconds);
    auto init_ng = value_or_default(experiment, "init_ng", 3);
    auto drone_speed = value_or_default(experiment, "drone_speed", 1.0);
    auto loops = value_or_default(experiment, "loops", false);
    auto compatible = value_or_default(experiment, "compatible", 0);
    auto truck_serves = value_or_default(experiment, "truck_serves", true);
    auto launch_time = value_or_default(experiment, "launch_time", 0.0);
    auto rendezvous_time = value_or_default(experiment, "rendezvous_time", 0.0);
    auto range = value_or_default(experiment, "range", numeric_limits<double>::infinity());
    auto land_and_wait = value_or_default(experiment, "land_and_wait", true);
    if(range == numeric_limits<double>::infinity()) land_and_wait = true;
    auto log_level_id = value_or_default(experiment, "log_level", 0);

    LogLevel log_level = LogLevel::None;
    if(log_level_id == 1) log_level = LogLevel::Main;
    if(log_level_id == 2) log_level = LogLevel::All;

    clog << "Experiment parameters" << endl 
         << "Time limit: " << time_limit
         << ", Init NG size: " << init_ng 
         << ", Drone speed: " << drone_speed
         << ", Loops: " << loops
         << ", Launch time: " << launch_time
         << ", Rendezvous time: " << rendezvous_time
         << ", Fly range: " << range
         << ", Land and wait: " << land_and_wait
         << ", Drone Comptibility Level: " << compatible
         << ", Truck serves alone?: " << truck_serves << endl;

    clog << "Parsing instance..." << endl;
    time_guardian.SetTimeLimit(time_limit);
    TSPDInput tsp_input = instance_json;
    TSPDInstance tsp(tsp_input, drone_speed);

    auto b_tsp = tsp.Reverse();

    Subinstance instance(&tsp, &b_tsp, loops, compatible, truck_serves, launch_time, rendezvous_time, range, land_and_wait);
    Solver solver(instance, NearbyNG(instance, init_ng));
    try {
        solver.log_level = log_level;
        solver.Run();
        
        clog << solver.BestSolution() << endl;
        output["solution"] = solver.BestSolution();
        output["solver"] = solver.Log();
	}
    catch (TimeLimitExceeded& tle)
    {
        clog << "Time limit reached." << endl;
        output["solver"] = solver.Log();
    }
	catch (std::bad_alloc& e)
	{
		return 3; // Memory limit exceeded code for runner.
	}
        
    cout << std::setw(4) << output << endl; // Return the output for Runner.

	return 0;
}
