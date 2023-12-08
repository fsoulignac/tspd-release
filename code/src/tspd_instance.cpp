//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "tspd_instance.h"

#include <cmath>

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace tspd
{
namespace {

pair<vector<Time>, vector<VertexSet>> ComputeByTime(vector<pair<Time, Vertex>>& times) {
    stable_sort(times.begin(), times.end());

    vector<double> changes{goc::INFTY};
    vector<VertexSet> sets{VertexSet()};
    for(auto it = times.rbegin(); it != times.rend(); ++it) {
        auto V = sets.back();
        changes.push_back(it->first);
        sets.push_back(V.set(it->second));
    }
    reverse(changes.begin(), changes.end());
    reverse(sets.begin(), sets.end());
    return {changes, sets};        
}
    
pair<vector<Time>, vector<VertexSet>> ComputeDroneArcsByTime(const TSPDInstance& tsp, Vertex v, Vertex w) {
    vector<pair<Time, Vertex>> times;
    for(auto z : tsp.network.Vertices()) times.push_back({tsp.fork_time[v][z] - tsp.fork_time[w][z], z});

    return ComputeByTime(times);
}
    
pair<vector<Time>,vector<VertexSet>> ComputeLegsByTime(const TSPDInstance& tsp, Vertex d, Vertex t) {
    vector<pair<Time, Vertex>> times;
    for(auto w : tsp.network.Vertices()) times.push_back({tsp.fork_time[d][w] + tsp.join_time[w][t], w});
    
    return ComputeByTime(times);
}

}

void to_json(json& j, const TSPDInput& input)
{
    j["vertex_count"] = input.vertex_count;
	j["start_depot"] = input.origin;
	j["end_depot"] = input.destination;
	j["truck_dist"] = input.truck_dist;
	j["drone_dist"] = input.drone_dist;
}

void from_json(const json& j, TSPDInput& input)
{
    input.vertex_count = j["vertex_count"];
	input.origin = j["start_depot"];
	input.destination = j["end_depot"];
	input.truck_dist = j["truck_dist"];
	input.drone_dist = j["drone_dist"];
}

void from_poi(istream& is, TSPDInput& input) {
    string _; 
    getline(is, _);   /*Number of nodes*/
    size_t n; is >> n; getline(is, _);
    getline(is, _); /*The Depot*/
    std::vector<Time> x(n+1), y(n+1);
    is >> x[0] >> y[0] >> _; getline(is, _);
    getline(is, _); /*The Locations (x_coor y_coor name)*/
    for(auto i = 1ul; i < n; ++i)
        is >> x[i] >> y[i] >> _;
    x[n] = x[0]; y[n] = y[0];
    
    if(n > 40) {
        for(auto& c : x) c *= 100;
        for(auto& c : y) c *= 100;
    }
    
    input.vertex_count = n+1;
    input.origin = 0;
    input.destination = n;
    
    input.truck_dist = Matrix<Time>(n+1, n+1);
    input.drone_dist = Matrix<Time>(n+1, n+1);
    for(auto i = 0ul; i <= n; ++i) for(auto j = 0ul; j <= n; ++j) {
        input.truck_dist(i,j) = abs(x[i] - x[j]) +  abs(y[i] - y[j]);
        input.drone_dist(i,j) = hypot(x[i] - x[j], y[i] - y[j]);
    }
} 

void TSPDInput::Print(ostream& os) const
{
	os << json(*this);
}

TSPDInstance::TSPDInstance(const TSPDInput& input, double the_drone_speed) {
    
    network = Digraph::Complete(input.vertex_count);    
    drone_speed = the_drone_speed;
    origin = input.origin;
	destination = input.destination;
	truck_time =  input.truck_dist;
    fork_time = input.drone_dist;
    for(auto i = 0ul; i < input.vertex_count; ++i) for(auto j = 0ul; j < input.vertex_count; ++j) {
        truck_time(i,j) = floor(truck_time(i,j));
        fork_time(i,j) = floor(fork_time(i,j) / drone_speed);
    }
	join_time = fork_time;

    for(auto v : network.Vertices()) vertex_set.set(v);
    UpdatePrecomputedTimes();

}

TSPDInstance TSPDInstance::Reverse() const
{
	TSPDInstance tsp;
	tsp.network = network.Reverse();
	tsp.origin = destination;
	tsp.destination = origin;
	tsp.truck_time = truck_time.transpose();
	tsp.fork_time = join_time.transpose();
    tsp.join_time = fork_time.transpose();
    tsp.vertex_set = vertex_set;
    
    tsp.UpdatePrecomputedTimes();

	return tsp;
}

VertexSet TSPDInstance::FartherDroneArcs(goc::Vertex v, goc::Vertex w, Time time) const {
    auto& C = drone_arcs_by_time_change(v, w);
    auto pos = std::upper_bound(C.begin(), C.end(), time) - C.begin();
    return drone_arcs_by_time_sets(v,w)[pos];
}

VertexSet TSPDInstance::FartherDroneLegs(goc::Vertex d, goc::Vertex t, Time time) const {
    auto& C = legs_by_time_change(d, t);
    auto pos = std::upper_bound(C.begin(), C.end(), time) - C.begin();
    return legs_by_time_sets(d,t)[pos];
}

void TSPDInstance::UpdatePrecomputedTimes() {
    auto n = network.VertexCount();
    drone_arcs_by_time_change = Matrix<vector<Time>>(n,n);
    drone_arcs_by_time_sets = Matrix<vector<VertexSet>>(n,n);
    legs_by_time_change = Matrix<vector<Time>>(n,n);
    legs_by_time_sets = Matrix<vector<VertexSet>>(n,n);
    for(auto d : network.Vertices()) for(auto t : network.Vertices()) {
        tie(drone_arcs_by_time_change(d,t), drone_arcs_by_time_sets(d,t)) = ComputeDroneArcsByTime(*this, d, t);
        tie(legs_by_time_change(d,t), legs_by_time_sets(d,t)) = ComputeLegsByTime(*this, d, t);
    }    
}

} // namespace tutorial1
