//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "subinstance.h"
#include <numeric>

using namespace std;
using namespace goc;

namespace tspd
{
    
Subinstance::Subinstance(
    const TSPDInstance* f_tsp, 
    const TSPDInstance* b_tsp, 
    bool loops_allowed,
    size_t compatible,
    bool truck_serves_alone,
    Time drone_launch_time, 
    Time drone_rendezvous_time, 
    Time fly_range,
    bool can_land_and_wait
)
  : tsp{f_tsp, b_tsp},
    loops(loops_allowed),
    truck_serves(truck_serves_alone),
    launch_time(drone_launch_time),
    rendezvous_time(drone_rendezvous_time),
    range(fly_range),
    land_and_wait(can_land_and_wait)
{
    for(auto d : {Direction::Forward, Direction::Backward}) {
        BitGraph truck_arcs_(VertexCount()), fork_arcs_(VertexCount()), join_arcs_(VertexCount());

        for(auto v : Vertices()) for(auto w : Vertices())  
        if(v != w) {
            if(w != Origin(d) and v != Destination(d)) truck_arcs_.AddArc(v,w);
            if(w != Origin(d) and w != Destination(d)) fork_arcs_.AddArc(v,w);
            if(v != Origin(d) and v != Origin(d)) join_arcs_.AddArc(w,v);
        }
        
        // The number of operations is n-1
        alone_truck_arcs[idx(d)] = LenBitGraph(VertexCount()-1, truck_arcs_);
        combined_truck_arcs[idx(d)] = LenBitGraph(VertexCount()-1, truck_arcs_);
        fork_arcs[idx(d)] = LenBitGraph(VertexCount()-1, fork_arcs_);
        join_arcs[idx(d)] = LenBitGraph(VertexCount()-1, join_arcs_);
    }
    
    // Fix arcs because of drone compatibility
    for(Vertex i = 1; i <= compatible; ++i)    
    for(Vertex w = i; w < VertexCount(); w += 5)
    for(auto v : Vertices()) {
        RemoveForkArc(v, w);
        RemoveJoinArc(w, v);
    }
    
    // Fix arcs that do not respect the flying range
    for(auto v : Vertices()) for(auto w : Vertices()) if(w != v) {
        bool can_fork = false;
        for(auto z : Vertices()) if(z != w and (loops or z != v))
            if(epsilon_smaller_equal(LegTime(Direction::Forward, v, w, z), range)) can_fork = true;
        if(not can_fork) {
            RemoveForkArc(v, w);
            RemoveJoinArc(w, v);
        }
    }
}

const TSPDInstance* Subinstance::TSP(Direction d) const
{
    return tsp[idx(d)];
}

bool Subinstance::Loops() const {
    return loops;
}

bool Subinstance::TruckServes() const {
    return truck_serves;
}

Time Subinstance::LaunchTime() const {
    return launch_time;
}

Time Subinstance::RendezvousTime() const {
    return rendezvous_time;
}

Time Subinstance::Range() const {
    return range;
}

bool Subinstance::LandAndWait() const {
    return land_and_wait;
}

goc::Vertex Subinstance::Origin(Direction d) const {
    return TSP(d)->origin;
}
    
goc::Vertex Subinstance::Destination(Direction d) const {
    return TSP(d)->destination;
}
    
const VertexSet& Subinstance::VertexBitset() const {
    return tsp[0]->vertex_set;
}

const goc::Digraph& Subinstance::Network(Direction d) const {
    return TSP(d)->network;
}
    
const std::vector<Vertex>& Subinstance::Vertices() const {
    return tsp[0]->network.Vertices();
}
    
size_t Subinstance::VertexCount() const {
    return tsp[0]->network.VertexCount();
}

Route Subinstance::NaiveRoute() const {
    Route res(0, {Origin(Direction::Forward)});
    for (Vertex v: Vertices()) if(v != Origin(Direction::Forward) and v != Destination(Direction::Forward)) {
        res.truck.push_back(v);
    }
    res.truck.push_back(Destination(Direction::Forward));
    res.time = static_cast<Time>(MAX_N) * RouteTime(Direction::Forward, res);
    return res;
}

size_t Subinstance::AloneTruckArcsCount(Direction d) const {
    return alone_truck_arcs[idx(d)].ArcCount();
}

size_t Subinstance::CombinedTruckArcsCount(Direction d) const {
    return combined_truck_arcs[idx(d)].ArcCount();
}

size_t Subinstance::TruckArcsCount(Direction d) const {
    return AloneTruckArcsCount(d) + CombinedTruckArcsCount(d);
}

size_t Subinstance::ForkArcsCount(Direction d) const {
    return fork_arcs[idx(d)].ArcCount();
}

size_t Subinstance::JoinArcsCount(Direction d) const {
    return join_arcs[idx(d)].ArcCount();
}

size_t Subinstance::DroneArcsCount(Direction d) const {
    return ForkArcsCount(d) + JoinArcsCount(d);
}

Time Subinstance::LaunchTime(Direction d, goc::Vertex v) const {
    if(v == Origin(Direction::Forward)) return 0.0;
    return d == Direction::Forward ? LaunchTime() : RendezvousTime();
}

Time Subinstance::RendezvousTime(Direction d, goc::Vertex v) const {
    return LaunchTime(op(d), v);
}

bool Subinstance::HasAloneTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const {
    return alone_truck_arcs[idx(d)].HasArc(op, v, w);
}

bool Subinstance::HasCombinedTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const {
    return combined_truck_arcs[idx(d)].HasArc(op, v, w);
}

bool Subinstance::HasForkArc(Direction d, size_t b, goc::Vertex v, goc::Vertex w) const {
    return fork_arcs[idx(d)].HasArc(b, v, w);
}

bool Subinstance::HasJoinArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const {
    return join_arcs[idx(d)].HasArc(op, w, v);
}

bool Subinstance::HasLeg(Direction d, size_t b, size_t op, goc::Vertex v, goc::Vertex w, goc::Vertex z) const {
    return HasForkArc(d, b, v, w) and HasJoinArc(d, op, w, z) and epsilon_smaller_equal(LegTime(d, v, w, z), range);
}

bool Subinstance::HasRoute(Direction d, const Route& route) const {
    auto operations = route.Operations();
    auto positions = route.Positions();
    auto locations = route.Locations();
        
    bool res = true;
    for(auto op = 0ul, bifurcation = 0ul; op < operations.size() and res; ++op) {
        auto [v, decision] = operations[op];
        auto [tpos, dpos] = positions[op];
        auto [truck, drone] = locations[op];
        
        if(tpos == dpos) bifurcation = op;

        if(decision == Move::Truck) res = HasAloneTruckArc(d, op, truck, v);
        else if(decision == Move::Combined) res = HasCombinedTruckArc(d, op, truck, v);
        else res = HasLeg(d, bifurcation, op, drone, v, truck);
    }
    return res;
}

Time Subinstance::TruckTime(Direction d, Vertex v, Vertex w) const {
    return tsp[idx(d)]->truck_time(v, w);
}

Time Subinstance::ForkTime(Direction d, Vertex v, Vertex w) const {
    return tsp[idx(d)]->fork_time(v, w);
}
    
Time Subinstance::JoinTime(Direction d, Vertex v, Vertex w) const {
    return tsp[idx(d)]->join_time(v, w);    
}
    
Time Subinstance::LegTime(Direction d, Vertex v, Vertex w, Vertex z) const {
    return ForkTime(d, v, w) + JoinTime(d, w, z);    
}

Time Subinstance::RouteTime(Direction d, const Route& route) const {
    auto operations = route.Operations();
    auto positions = route.Positions();
    auto locations = route.Locations();
        
    Time res = 0, truck_time = 0;
    for(auto op = 0ul; op < operations.size(); ++op) {
        auto [v, decision] = operations[op];
        auto [tpos, dpos] = positions[op];
        auto [truck, drone] = locations[op];
            
        if(dpos == tpos) truck_time = 0;
        
        if(decision == Move::Truck) {
            if(dpos == tpos) res += LaunchTime(d, truck);
            truck_time += TruckTime(d, truck, v);
        } else if(decision == Move::Combined) {
            res += TruckTime(d, truck, v);
        } else {
            res += std::max(truck_time, LegTime(d, drone, v, truck)) + RendezvousTime(d, truck);
        }
    }
    return res;    
}


VertexSet Subinstance::ForksFrom(Direction d, size_t b, Vertex v) const {
    return fork_arcs[idx(d)].Neighborhood(b, v);
}

VertexSet Subinstance::JoinsAt(Direction d, size_t op, goc::Vertex v) const {
    return join_arcs[idx(d)].Neighborhood(op, v);
}


VertexSet Subinstance::FartherDroneArcs(Direction d, goc::Vertex v, goc::Vertex w, Time time) const {
    if(range != numeric_limits<double>::infinity())
        return tsp[idx(d)]->FartherDroneArcs(v, w, time);
    else
        return tsp[idx(d)]->FartherDroneArcs(v, w, min(time, 0.0));
}

VertexSet Subinstance::FartherDroneLegs(Direction d, goc::Vertex v, goc::Vertex w, Time time) const {
    if(range != numeric_limits<double>::infinity())
        return tsp[idx(d)]->FartherDroneLegs(v, w, time);
    else 
        return tsp[idx(d)]->FartherDroneLegs(v, w, min(time, range));
}

void Subinstance::RemoveTruckArc(goc::Vertex v, goc::Vertex w) {
    for(auto op = 0ul; op < VertexCount()-1; ++op) {
        alone_truck_arcs[0].RemoveArc(op, v, w);
        combined_truck_arcs[0].RemoveArc(op, v, w);
        alone_truck_arcs[1].RemoveArc(op, w, v);
        combined_truck_arcs[1].RemoveArc(op, w, v);
    }
}

void Subinstance::RemoveForkArc(goc::Vertex v, goc::Vertex w) {
    for(auto op = 0ul; op < VertexCount()-1; ++op) {
        fork_arcs[0].RemoveArc(op, v, w);
        join_arcs[1].RemoveArc(op, v, w);
    }
}

void Subinstance::RemoveJoinArc(goc::Vertex v, goc::Vertex w) {
    for(auto op = 0ul; op < VertexCount()-1; ++op) {
        join_arcs[0].RemoveArc(op, w, v);
        fork_arcs[1].RemoveArc(op, w, v);
    }    
}
    
void Subinstance::RemoveAloneTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) {
    if(op >= VertexCount()-2) return; 
    alone_truck_arcs[idx(d)].RemoveArc(op, v, w);
    alone_truck_arcs[opidx(d)].RemoveArc(VertexCount()-op-3, w, v);
}
    
void Subinstance::RemoveCombinedTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) {
    if(op >= VertexCount()-1) return; 
    combined_truck_arcs[idx(d)].RemoveArc(op, v, w);
    combined_truck_arcs[opidx(d)].RemoveArc(VertexCount()-op-2, w, v);
}
    
void Subinstance::RemoveForkArcAt(Direction d, size_t b, goc::Vertex v, goc::Vertex w) {
    if(b >= VertexCount()-1) return; 
    fork_arcs[idx(d)].RemoveArc(b, v, w);
    join_arcs[opidx(d)].RemoveArc(VertexCount()-b-2, v, w);
}
    
void Subinstance::RemoveJoinArcAt(Direction d, size_t op, goc::Vertex v, goc::Vertex w) {
    if(op >= VertexCount()-1) return; 
    join_arcs[idx(d)].RemoveArc(op, w, v);
    fork_arcs[opidx(d)].RemoveArc(VertexCount()-op-2, w, v);
}


} // namespace tspd
