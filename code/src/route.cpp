//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include <unordered_set>
#include "route.h"

using namespace std;
using namespace nlohmann;
using namespace goc;

namespace tspd
{

Route::Route(Time tim, const GraphPath& t, const vector<Vertex>& d, const vector<Bifurcation>& b)
  : truck(t), 
    drone(d), 
    bifurcation(b), 
    time(tim)
{ }

std::tuple<goc::Vertex, goc::Vertex, goc::Vertex> Route::Leg(size_t i) const 
{
    return {truck[bifurcation[i].first], drone[i], truck[bifurcation[i].second]}; 
}


void Route::Reverse() 
{
    std::reverse(truck.begin(), truck.end());  
    std::reverse(drone.begin(), drone.end());  
    std::reverse(bifurcation.begin(), bifurcation.end());  
    for(auto& [p,n] : bifurcation) {
        p = truck.size() - p - 1;
        n = truck.size() - n - 1;
        swap(p, n);
    }
}

void Route::Merge(const Route& other) {
    assert(truck.back() == other.truck.front());
    time += other.time;
    for(auto [p, n] : other.bifurcation) bifurcation.push_back({p+truck.size()-1, n+truck.size()-1});
    truck.insert(truck.end(), std::next(other.truck.begin()), other.truck.end());
    drone.insert(drone.end(), other.drone.begin(), other.drone.end());
}


bool Route::IsElementary() const
{
    unordered_set<Vertex> elems;
    for(auto& td : {truck, drone}) for(auto v : td) {
        if(find(elems.begin(), elems.end(), v) != elems.end()) return false;
        elems.insert(v);
    }
	return true;
}

vector<size_t> Route::Descriptor(size_t n) const 
{
    std::vector<size_t> res(n, 0);
    for(auto& td : {truck,drone}) for(auto v : td) if(v < n) res[v]++;
    return res;
}

vector<pair<Vertex, Move>> Route::Operations() const
{
    vector<pair<Vertex, Move>> res;

    for(auto d = 0ul, t = 0ul; res.size() < truck.size() + drone.size() - 1;) {
        if(d < bifurcation.size() and bifurcation[d].second == t) {
            res.push_back({drone[d], Move::Drone});
            ++d;
        } else if(d < bifurcation.size() and bifurcation[d].first <= t) {
            ++t;
            res.push_back({truck[t], Move::Truck});
        } else {
            ++t;
            res.push_back({truck[t], Move::Combined});
        }
    }
    return res;
}

vector<pair<size_t, size_t>> Route::Positions() const 
{    
    std::vector<std::pair<Vertex, Vertex>> res{{0, 0}};
    
    for(auto [v, decision] : Operations()) {
        if(decision == Move::Truck) res.push_back({res.back().first + 1, res.back().second});
        else if(decision == Move::Combined) res.push_back({res.back().first + 1, res.back().second + 1});
        else res.push_back({res.back().first, res.back().first});
    }
    
    return res;
}

vector<pair<Vertex, Vertex>> Route::Locations() const
{
    vector<pair<Vertex, Vertex>> res;
    for(auto [t,d] : Positions()) res.push_back({truck[t], truck[d]});
    return res;
}


bool Route::NGFeasible(const BitGraph& ng) const {
    auto operations = Operations();
    auto positions = Positions();
    auto locations = Locations();
    VertexSet forbidden_truck, forbidden_drone;
        
    bool res = true;
    for(auto op = 0ul; op < operations.size() and res; ++op) {
        auto [v, decision] = operations[op];
        auto [tpos, dpos] = positions[op];
        auto [t, d] = locations[op];
        
        if(tpos == dpos) forbidden_drone = unite(forbidden_truck, {t});
        else forbidden_drone.set(t);
        
        if(decision == Move::Truck or decision == Move::Combined) {
            res = not forbidden_truck.test(v);
            forbidden_truck.set(t);
            forbidden_truck &= ng.Neighborhood(v);
        } else {
            res = not forbidden_drone.test(v);
        }        
    }
    return res;
}

double Route::ReducedCost(const DualVector& duals) const 
{
    double res = time;
    for(auto& td : {truck, drone}) for(auto v : td) res -= duals[v];
    return res;
}

Time Route::LowerBound(const DualVector& duals) const 
{
    return ReducedCost(duals) + accumulate(duals.begin(), duals.end(), 0.0);
}

void Route::Print(ostream& os) const
{
	os << json(*this);
}

void to_json(json& j, const Route& r)
{
	j["kd_type"] = "tspd_route";
	j["truck"] = r.truck;
	j["drone"] = r.drone;
	j["time"] = r.time;
	j["bifurcation"] = r.bifurcation;
}

void from_json(const json& j, Route& r)
{
	GraphPath t = j["truck"];
    vector<Route::Bifurcation> b = j["bifurcation"];
	vector<Vertex> d = j["drone"];
	r.truck = t;
    r.drone = d;
    r.bifurcation = b;
    r.time = j["time"];
}

std::ostream& operator<<(std::ostream& os, Move m) {
    switch(m) {
        case Move::Truck:
            return os << "t";
        case Move::Drone:
            return os << "d";
        case Move::Combined:
            return os << "c";
        default:
            return os << "ERROR!";
    }
}

} // namespace goc
