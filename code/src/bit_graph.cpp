//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "bit_graph.h"

using namespace std;
using namespace nlohmann;
using namespace goc;

namespace tspd
{
BitGraph::BitGraph(size_t n) 
  : arcs(n) 
{}

BitGraph::BitGraph(size_t n, const VertexSet& s) 
  : arcs(n, s)
{
    size = n * s.count();
}

BitGraph BitGraph::Reverse() const
{
    BitGraph res(VertexCount());
    
    for(auto v = 0ul; v < VertexCount(); ++v) for(auto w = 0ul; w < VertexCount(); ++w)
    if(HasArc(v,w)) res.AddArc(w,v);
    
    return res;
}


void BitGraph::AddArc(Vertex v, Vertex w) 
{
    if(not HasArc(v, w)) {
        ++size;
        arcs[v].set(w);
    }
}

void BitGraph::RemoveArc(Vertex v, Vertex w) 
{
    if(HasArc(v, w)) {
        --size;
        arcs[v].reset(w);
    }
}

bool BitGraph::HasArc(Vertex v, Vertex w) const
{
    return arcs[v].test(w);
}

const VertexSet& BitGraph::Neighborhood(Vertex v) const {
    return arcs[v];
}

size_t BitGraph::VertexCount() const 
{
    return arcs.size();
}

size_t BitGraph::ArcCount() const 
{
    return size;
}

void BitGraph::Print(ostream& os) const
{
	os << json(*this);
}

void to_json(json& j, const BitGraph& G)
{
	j["arcs"] = G.arcs;
	j["arc_count"] = G.size;
}

void from_json(const json& j, BitGraph& G)
{
	vector<VertexSet> a = j["arcs"];
	G.arcs = a;
    G.size = j["arc_count"];
}


LenBitGraph::LenBitGraph(size_t k, size_t n) 
  : graph(k, BitGraph(n)) 
{}

LenBitGraph::LenBitGraph(size_t k, const BitGraph& g)
  : graph(k, g)
{
    size = k * g.ArcCount();
}


void LenBitGraph::AddArc(size_t l, Vertex v, Vertex w) 
{
    if(not HasArc(l,v,w)) {
        ++size;
        graph[l].AddArc(v,w);
    }
}

void LenBitGraph::RemoveArc(size_t l, Vertex v, Vertex w) 
{
    if(HasArc(l,v,w)) {
        --size;
        graph[l].RemoveArc(v,w);
    }
}

bool LenBitGraph::HasArc(size_t l, Vertex v, Vertex w) const 
{
    return graph[l].HasArc(v, w);
}

const VertexSet& LenBitGraph::Neighborhood(size_t l, Vertex v) const {
    return graph[l].Neighborhood(v);
}


size_t LenBitGraph::Copies() const 
{
    return graph.size();
}  

size_t LenBitGraph::ArcCount() const 
{
    return size;
}  

void LenBitGraph::Print(ostream& os) const
{
	os << json(*this);
}

void to_json(json& j, const LenBitGraph& G)
{
	j["graph"] = G.graph;
	j["arc_count"] = G.size;
}

void from_json(const json& j, LenBitGraph& G)
{
	vector<BitGraph> g = j["arcs"];
	G.graph = g;
    G.size = j["arc_count"];
}

} // namespace tspd
