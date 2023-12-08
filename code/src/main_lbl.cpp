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
    simulate_runner_input("../instances/poi/", "poi-10-1", "../experiments/lbl.json", "mono");

    json experiment, instance_json, solutions;
    cin >> experiment >> instance_json >> solutions;

    auto time_limit = Duration(value_or_default(experiment, "time_limit", 600), DurationUnit::Seconds);
    auto init_ng = value_or_default(experiment, "init_ng", 3);
    auto bidirectional = value_or_default(experiment, "bidirectional", true);  
    auto drone_speed = value_or_default(experiment, "drone_speed", 1.0);
    auto dominance_level_id = value_or_default(experiment, "dominance_level", 0);
    
    DominanceLevel dominance_level = DominanceLevel::Fork;
    if(dominance_level_id == 1) dominance_level = DominanceLevel::NoFork;
    
    clog << "Experiment parameters" << endl 
         << "Time limit: " << time_limit
         << ", Init NG size: " << init_ng 
         << (bidirectional ? string(", Bidirectional search") : string(", Forward search"))
         << ", Dominance Level: " << dominance_level << endl;

    clog << "Parsing instance..." << endl;
    time_guardian.SetTimeLimit(Duration(3600, DurationUnit::Seconds));
    TSPDInput tsp_input = instance_json;
    TSPDInstance tsp(tsp_input, drone_speed);

    auto b_tsp = tsp.Reverse();
    
    Subinstance instance(&tsp, &b_tsp);
    auto ng = NearbyNG(instance, init_ng);
    Solver solver(instance, ng);
    try{
        clog << "Computing best duals of RMP" << endl;
        auto [_, duals] = solver.ColGen(goc::INFTY, false, true, DominanceLevel::Fork);

        clog << endl << "Running the labeling on the best duals of the RMP" << endl;
        auto ext = bidirectional ? instance.VertexCount()/2 : instance.VertexCount()-1;
        Labeling lbl("Labeling on an optimal dual vector", &instance, &ng, duals, dominance_level);

        time_guardian.SetTimeLimit(time_guardian.Now() + time_limit);
        lbl.Run(ext);
        auto best_route = lbl.Merge(numeric_limits<Time>::max(), 1)[0];
        
        //infeasible is not an option
        clog << best_route << endl;
        output["solution"] = best_route; // Add solution to the output.
        output["solver"] = lbl.Log(0); // Add execution information to the output.
	}
    catch (TimeLimitExceeded& tle)
    {
        clog << "Time limit reached." << endl;
    }
	catch (std::bad_alloc& e)
	{
		return 3; // Memory limit exceeded code for runner.
	}
        
    // Output result information.
    cout << std::setw(4) << output << endl; // Return the output for Runner.

	return 0;
}
