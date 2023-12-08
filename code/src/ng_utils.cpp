//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "ng_utils.h"

using namespace std;
using namespace goc;

namespace tspd {

BitGraph NearbyNG(const Subinstance& instance, size_t ng_size) {
    BitGraph res(instance.VertexCount());
    for(auto i = 1ul; i < instance.VertexCount()-1; i++){
        vector<pair<double, size_t>> dists;
        for(auto j = 1ul; j < instance.VertexCount()-1; j++){
            if(i==j) continue;
            dists.push_back({instance.TruckTime(Direction::Forward, i,j), j});
        }
        sort(dists.begin(), dists.end());
        for(auto j = 0ul; j < ng_size and j < dists.size(); j++){
            res.AddArc(i, dists[j].second);
        }
    } 
    return res;
}

BitGraph ElementaryNG(const Subinstance& tsp) {
    return BitGraph(tsp.VertexCount(), tsp.VertexBitset());
}

vector<RouteCycle> Cycles(const tspd::Route& route, size_t max_size) 
{
    vector<RouteCycle> res;
    auto operations = route.Operations();
    auto positions = route.Positions();
    
    operations.insert(operations.begin(), {route.truck[0], Move::Combined});

    constexpr size_t NIL = numeric_limits<size_t>::max();
    vector<size_t> last(MAX_N, NIL);

    auto bifurcation = 0ul;
    auto last_truck_move = 0ul;
    for(auto op = 1ul; op < operations.size(); ++op) {
        auto [v, decision] = operations[op];
        auto [tpos, dpos] = positions[op-1];
                
        if(tpos == dpos) bifurcation = op;

        if(last[v] != NIL) {
            auto cycle_end = op;
            if(decision == Move::Drone) cycle_end = bifurcation;
            if(cycle_end - last[v] <= max_size and cycle_end - last[v] > 0) {
                res.push_back({v, {}});
                for(auto j = last[v]; j < cycle_end; ++j) 
                if(operations[j].second == Move::Truck or operations[j].second == Move::Combined)
                    res.back().second.push_back(operations[j].first);
            }
        }
        last[v] = decision == Move::Drone ? last_truck_move : op+1;
        if(decision == Move::Truck) last_truck_move = op;
    }
    return res;
}

bool AugmentNG(BitGraph* NG, const RouteCycle& cycle, const vector<size_t>& max_ng_of) {

    auto& ng = *NG;
    bool can_forbid = true;
    auto& [v, C] = cycle;
    for(auto w : C) can_forbid = can_forbid and (ng.HasArc(w, v) or ng.Neighborhood(w).count() < max_ng_of[w]);
    if(can_forbid) {
        for(auto w : C) ng.AddArc(w, v);
    }
    return can_forbid;
}

bool AugmentNG(BitGraph* NG, const Route& route, const vector<size_t>& max_ng_of) {
    
    auto cycles = Cycles(route, MAX_N);
    sort(cycles.begin(), cycles.end(), [](const RouteCycle& c1, const RouteCycle& c2){return c1.second.size() < c2.second.size();});

    for(auto& c : cycles) 
    if(AugmentNG(NG, c, max_ng_of)) 
        return true;
    return false;
}

bool AugmentNG(BitGraph* NG, const std::vector<Route>& routes, const vector<size_t>& max_ng_of) {
    
    vector<RouteCycle> cycles;
    for(auto& r : routes) {
        auto r_cycles = Cycles(r, MAX_N);
        cycles.insert(cycles.end(), r_cycles.begin(), r_cycles.end());
    }    
    sort(cycles.begin(), cycles.end(), [](const RouteCycle& c1, const RouteCycle& c2){return c1.second.size() < c2.second.size();});

    bool res = false;
    for(auto& c : cycles) res = AugmentNG(NG, c, max_ng_of) or res;
    return res;
}

}
