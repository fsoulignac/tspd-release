//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_SUBINSTANCE_H
#define TSPD_SUBINSTANCE_H

#include <vector>
#include "tspd_instance.h"
#include "route.h"
#include "bit_graph.h"

namespace tspd {

/*
 * Represents a dynamic instance of the TSP-D and it also serves as a wrapper for many usefull functions
 *
 * To avoid duplicating information, each subinstance is linked to an instance of the tspd and to its reverse
 * These members must survive the subinstance to avoid access to dangling pointers
 */
struct Subinstance {
    Subinstance() = default;
    
    Subinstance(
        const TSPDInstance* f_tsp, 
        const TSPDInstance* b_tsp, 
        bool loops = false,
        size_t compatible = 0,
        bool truck_serves = true,
        Time launch_time = 0.0, 
        Time rendezvous_time = 0.0,
        Time range = std::numeric_limits<double>::infinity(),
        bool land_and_wait = true
    );

    const TSPDInstance* TSP(Direction d) const;
    bool Loops() const;
    bool TruckServes() const;
    Time LaunchTime() const;
    Time RendezvousTime() const;
    Time Range() const;
    bool LandAndWait() const;

    goc::Vertex Origin(Direction d) const;
    goc::Vertex Destination(Direction d) const;
    const VertexSet& VertexBitset() const;
    const goc::Digraph& Network(Direction d) const;
    const std::vector<goc::Vertex>& Vertices() const;
    size_t VertexCount() const;
    Route NaiveRoute() const;
        
    size_t AloneTruckArcsCount(Direction d) const;
    size_t CombinedTruckArcsCount(Direction d) const;
    size_t TruckArcsCount(Direction d) const;
    
    size_t ForkArcsCount(Direction d) const;
    size_t JoinArcsCount(Direction d) const;
    size_t DroneArcsCount(Direction d) const;
    
    Time LaunchTime(Direction d, goc::Vertex v)const;
    Time RendezvousTime(Direction d, goc::Vertex v)const;
    bool HasAloneTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const;
    bool HasCombinedTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const;
    bool HasForkArc(Direction d, size_t b, goc::Vertex v, goc::Vertex w) const;
    bool HasJoinArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w) const;
    bool HasLeg(Direction d, size_t b, size_t op, goc::Vertex v, goc::Vertex w, goc::Vertex z) const;
    bool HasRoute(Direction d, const Route& route) const;
    Time TruckTime(Direction d, goc::Vertex v, goc::Vertex w) const;
	Time ForkTime(Direction d, goc::Vertex v, goc::Vertex w) const; 
    Time JoinTime(Direction d, goc::Vertex v, goc::Vertex w) const;
    Time LegTime(Direction d, goc::Vertex v, goc::Vertex w, goc::Vertex z) const;
    Time RouteTime(Direction d, const Route& r) const;
    VertexSet ForksFrom(Direction d, size_t b, goc::Vertex v) const;
    VertexSet JoinsAt(Direction d, size_t op, goc::Vertex v) const;
    VertexSet FartherDroneArcs(Direction d, goc::Vertex v, goc::Vertex w, Time time) const;
    VertexSet FartherDroneLegs(Direction d, goc::Vertex v, goc::Vertex z, Time time) const;

    void RemoveTruckArc(goc::Vertex v, goc::Vertex w);
    void RemoveForkArc(goc::Vertex v, goc::Vertex w);
    void RemoveJoinArc(goc::Vertex v, goc::Vertex w);
    void RemoveAloneTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w);
    void RemoveCombinedTruckArc(Direction d, size_t op, goc::Vertex v, goc::Vertex w);
    void RemoveForkArcAt(Direction d, size_t b, goc::Vertex v, goc::Vertex w);
    void RemoveJoinArcAt(Direction d, size_t op, goc::Vertex v, goc::Vertex w);

private:
    const TSPDInstance* tsp[2];
    LenBitGraph alone_truck_arcs[2];
    LenBitGraph combined_truck_arcs[2];
    LenBitGraph fork_arcs[2];

    bool loops;
    bool truck_serves;
    Time launch_time;
    Time rendezvous_time;
    Time range;
    bool land_and_wait;
    
    LenBitGraph join_arcs[2];
};
    
}  // namespace tspd

#endif //TSPD_SUBINSTANCE
