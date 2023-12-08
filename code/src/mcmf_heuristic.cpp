//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "mcmf_heuristic.h"

using namespace std;
using namespace goc;

namespace tspd {
    
namespace {

Route RunHeuristic(const Subinstance& instance, const Route& route) {
    
    VertexSet in_truck;
    for(auto v : route.truck) in_truck.set(v);
    vector<Vertex> drones;
    for(auto v : instance.Vertices()) if(not in_truck.test(v)) drones.push_back(v);

    vector<double> leg_cost;
    for(auto [f, j] : route.bifurcation) {
        leg_cost.push_back(0.0);
        for(; f < j; ++f) leg_cost.back() += instance.TruckTime(Direction::Forward, route.truck[f], route.truck[f+1]);
    }
    
	auto formulation = unique_ptr<Formulation>(LPSolver::NewFormulation());
	formulation->Minimize(Expression());
    
    for(auto _ : drones)
        formulation->AddConstraint(Expression().EQ(0)), (void)_;
    auto lidx = drones.size();
    for(auto _ : route.bifurcation)
        formulation->AddConstraint(Expression().EQ(0)), (void)_;
    auto sidx = lidx + route.bifurcation.size();
    formulation->AddConstraint(Expression().EQ(static_cast<int>(drones.size())));
    formulation->AddConstraint(Expression().EQ(static_cast<int>(route.bifurcation.size())));
    
        
    for(auto d = 0ul; d < drones.size(); ++d) {
        Variable f = formulation->AddVariable("sd_" + STR(d), VariableDomain::Real, 0, 1);
        formulation->SetObjectiveCoefficient(f, 0);
        formulation->SetConstraintCoefficient(static_cast<int>(d), f, 1);
        formulation->SetConstraintCoefficient(static_cast<int>(sidx), f, 1);
    }
    
    for(auto l = 0ul; l < route.bifurcation.size(); ++l) {
        Variable f = formulation->AddVariable("lt_" + STR(l), VariableDomain::Real, 0, 1);
        formulation->SetObjectiveCoefficient(f, 0);
        formulation->SetConstraintCoefficient(static_cast<int>(lidx + l), f, -1);
        formulation->SetConstraintCoefficient(static_cast<int>(sidx+1), f, 1);
    }
    
    unordered_map<int, pair<int, Vertex>> var_to_drone_leg;
    for(auto d = 0ul; d < drones.size(); ++d) for(auto l = 0ul; l < route.bifurcation.size(); ++l) { 
        auto [u,_,w] = route.Leg(l); auto v = drones[d];
        if(not instance.HasLeg(Direction::Forward, route.bifurcation[l].first, route.bifurcation[l].second, u, v, w)) continue;
     	Variable f = formulation->AddVariable("f(" + STR(d) + "," + STR(l) + ")", VariableDomain::Real, 0.0, 1.0);
        formulation->SetObjectiveCoefficient(f, max(0.0, instance.LegTime(Direction::Forward, u, v, w) - leg_cost[l]));
        formulation->SetConstraintCoefficient(static_cast<int>(d), f, -1);
        formulation->SetConstraintCoefficient(static_cast<int>(lidx + l), f, 1);
        var_to_drone_leg[f.Index()] = {l, v};
    }
    
    LPSolver lp_solver;
    lp_solver.time_limit = Duration::Max();
    auto lp_log = lp_solver.Solve(formulation.get(), {LPOption::Incumbent});
    if (lp_log.status != LPStatus::Optimum or not lp_log.incumbent.IsInteger()) {
        return Route(goc::INFTY);
    }

    tspd::Route res = route;
    res.drone.resize(route.drone.size());
    for (auto& f: lp_log.incumbent) if(f.first.Index() >= (int)(drones.size() + route.bifurcation.size()) and f.second > 0.1) {
        auto [pos, v] = var_to_drone_leg[f.first.Index()];
        res.drone[pos] = v;
    }
    res.time = instance.RouteTime(Direction::Forward, res);
    
    return res;
}


Route MakeTruckElementary(const Subinstance& instance, const Route& route) {

    VertexSet in_truck;
    for(auto v : route.truck) in_truck.set(v);
    std::list<Vertex> cand;
    for(auto v : instance.Vertices()) if(not in_truck[v]) 
        cand.push_back(v);

    tspd::Route res = route;

    in_truck.reset();
    auto tt = [&](size_t t, Vertex v) {
        auto& T = res.truck; auto d = Direction::Forward;
        return instance.TruckTime(d, T[t-1], v) + instance.TruckTime(d, v, T[t+1]);
    };
    for(auto t = 0ul; t < res.truck.size(); ++t) {
        if(in_truck.test(res.truck[t])) {
            auto c = min_element(cand.begin(), cand.end(), [&](Vertex w, Vertex z){return epsilon_smaller(tt(t, w), tt(t, z));});
            res.truck[t] = *c;
            cand.erase(c);
        }
        in_truck.set(res.truck[t]);
    }
        
    res.time = instance.RouteTime(Direction::Forward, res);
    return res;
}
    
} // namespace anonymous

Route MinCostMaxFlowHeuristic(const Subinstance& instance, const Route& route) {
    
    auto r = MakeTruckElementary(instance, route);
    
    if(not instance.LandAndWait()) {
        for(auto [f,j] : r.bifurcation) {
            double dist = 0;
            for(; f < j; ++f) 
                dist += instance.TruckTime(Direction::Forward, r.truck[f], r.truck[f+1]);
                
            if(epsilon_bigger(dist, instance.Range())) return Route(goc::INFTY);
        }
    }
    
    return RunHeuristic(instance, r);
}

} // namespace tspd
