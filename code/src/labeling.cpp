//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "labeling.h"
#include "time_guardian.h"

using namespace std;
using namespace goc;


namespace tspd
{    

namespace 
{
    
Route BuildReversedRouteOf(const Label* l)
{    
    Route r;
    for(; l->prev != nullptr; l = l->prev) {
        if(l->last != l->truck) { // is a drone arc? 
            r.drone.push_back(l->last);
            r.bifurcation.push_back({r.truck.size(), r.truck.size() + l->length - 1 - l->prev->bifurcation});
        } else {
            r.truck.push_back(l->last);
        }
    }
    r.truck.push_back(l->last);
    return r;
}

Route BuildRouteOf(const Label* l)
{
    auto res = BuildReversedRouteOf(l);
    res.Reverse();
    return res;    
}

bool Precedes(const tuple<Time, Time, double>& a, const tuple<Time, Time, double>& b) {
        if(epsilon_bigger(get<0>(a), get<0>(b))) return false;
        if(epsilon_smaller(get<0>(a), get<0>(b))) return true;
        if(epsilon_bigger(get<1>(a), get<1>(b))) return false;
        if(epsilon_smaller(get<1>(a), get<1>(b))) return true;
        return epsilon_smaller_equal(get<2>(a), get<2>(b));
}
    
bool Dominates(const tuple<Time, Time, double>& a, const tuple<Time, Time, double>& b) {
    return epsilon_smaller_equal(get<0>(a), get<0>(b)) and 
           epsilon_smaller_equal(get<1>(a), get<1>(b)) and 
           epsilon_smaller_equal(get<2>(a), get<2>(b));
}

}

double Label::CostToTruck() const 
{
    return cost + truck_time;
}

bool Label::TruckCarriesDrone() const 
{
    return bifurcation == length;
}

Label Label::Associated() const
{
    Label res;
    res.length = length;
    res.bifurcation = length;
    res.last = truck;
    res.truck = truck;
    res.drone = truck;
    res.cost = cost + truck_time;
    res.truck_time = 0;
    res.forbidden_truck = forbidden_truck;
    res.extensions_lbl = VertexSet().set();
    res.extensions = VertexSet().set();
    res.prev = prev;
    return res;
}
    
void Label::Print(ostream& os) const
{
    using goc::operator<<;
	os <<  "(length: "  << length
       << ", truck: "  << truck
       << ", drone: "  << drone 
       << ", bifurcation: "  << bifurcation
       << ", truck cost: " << CostToTruck()
       << ", cost: "  << cost 
       << ", truck time: " << truck_time 
       << ", forbidden truck: " << forbidden_truck
       << ", extensions lbl: "<< extensions_lbl
       << ", extensions: " << extensions
       << ", prev id: " << prev
       << ")";
}


Labeling::Labeling(
    const string& description,
    const Subinstance* the_instance, 
    const BitGraph* the_ng,
    const vector<double>& the_duals, 
    DominanceLevel the_dominance_level,
    bool the_relax_ng,
    bool the_relax_cost
) :
    instance(the_instance),
    ng(the_ng),
    duals(the_duals),
    dominance_level(the_dominance_level),
    relax_cost(the_relax_cost),
    relax_ng(the_relax_ng)
{
    NewLogPage(description);

    sorted_duals = duals;
    sorted_duals.push_back(numeric_limits<double>::infinity());
    stable_sort(sorted_duals.begin(), sorted_duals.end());

    bigger_dual.resize(sorted_duals.size());
    for(auto i = 0ul; i < bigger_dual.size(); ++i) {
        for(auto v : instance->Vertices())
        if(epsilon_bigger_equal(duals[v], sorted_duals[i]))
            bigger_dual[i].set(v);
    }

    V_by_duals = instance->Vertices();
    stable_sort(V_by_duals.begin(), V_by_duals.end(), [&](Vertex v, Vertex w) {return epsilon_bigger(duals[v], duals[w]);});

    dual_bigger.resize(instance->VertexCount());
    for(auto v : instance->Vertices()) 
    for(auto w : instance->Vertices()) 
    if(epsilon_smaller(duals[v], duals[w])) 
        dual_bigger[v].set(w);
    
    for(auto d : {Direction::Forward, Direction::Backward}) {
        length[idx(d)] = 1;
        
        auto initial = new Label();
        initial->length = 0;
        initial->bifurcation = 0;
        initial->last = instance->Origin(d);
        initial->truck = instance->Origin(d);
        initial->drone = instance->Origin(d);
        initial->cost = -duals[instance->Origin(d)];
        initial->truck_time = 0;
        initial->forbidden_truck = {};
        initial->extensions_lbl = instance->VertexBitset();
        initial->extensions = instance->VertexBitset();
        labels[idx(d)].push_back(LabelPtr(initial));
        non_extended[idx(d)].push_back(initial);
    }    
}

void Labeling::Run(size_t ext)
{
    Run(ext, goc::INFTY);
}

void Labeling::Run(size_t ext, double bound)
{
    MRun(Direction::Forward, ext, bound);
    MRun(Direction::Backward, instance->VertexCount() - 1 - ext, bound);
}

void Labeling::MRun(Direction d, size_t ext)
{
    MRun(d, ext, goc::INFTY);
}


void Labeling::MRun(Direction d, size_t ext, double bound)
{
	Stopwatch rolex_run(true);

    for(; length[idx(d)] <= ext; ++length[idx(d)])
    {
        auto undominated = ExtendUndominated(d, bound);     
        non_extended[idx(d)].clear();
    
        labels[idx(d)].insert(labels[idx(d)].end(), undominated.begin(), undominated.end());
        for(auto& l : undominated) {
            UpdateExtensionData(l.get());
            non_extended[idx(d)].push_back(l.get());
        }
            
        log.back().monolog[idx(d)].processed_count += undominated.size();
        log.back().monolog[idx(d)].count_by_length[length[idx(d)]] = undominated.size();

        time_guardian.FailIfTimeLimit();
    }  

    log.back().monolog[idx(d)].next_length = length[idx(d)];
	log.back().monolog[idx(d)].time += rolex_run.Peek();
    log.back().time += rolex_run.Peek();
}

vector<Route> Labeling::Merge(size_t ext, double bound, size_t limit)
{
	Stopwatch rolex_merge(true);

    multimap<double, LBLMerge> best_merge;

    BoundingBucket B{BoundingBucketTruck(instance->VertexCount()), BoundingBucketTruck(instance->VertexCount())};

    auto rext = instance->VertexCount() - ext - 1;
	for (auto l : labels[1]) {
        if ((l->length == rext and l->TruckCarriesDrone()) or (l->length == rext-1 and (instance->TruckServes() or l->TruckCarriesDrone())))
            B[rext-l->length][l->truck].push_back(l.get());
    }
	for(auto e : {0,1}) for (auto& entry1: B[e])
        sort(entry1.begin(), entry1.end(), [](const Label* l1, const Label* l2) {return l1->CostToTruck() < l2->CostToTruck();});
    
	for (auto f: labels[0]) if(f->length == ext)
	{
        DoMerge(Direction::Forward, f.get(), B, bound, limit, numeric_limits<int>::max(), &best_merge);
    }
    
    vector<Route> res;
    vector<vector<size_t>> descriptors;
    for(auto [c, t] : best_merge) {
        auto [f, b, w] = t;
        
        auto r = RouteOf(Direction::Forward, f); 
        if(not f->TruckCarriesDrone()) {
            r.drone.push_back(w);
            r.bifurcation.push_back({r.truck.size() - 1 - f->length + f->bifurcation, r.truck.size()-1 + b->length - b->bifurcation});
        }
        r.Merge(ReversedRouteOf(Direction::Backward, b));
        r.time = instance->RouteTime(Direction::Forward, r);
        
        auto d = r.Descriptor(instance->VertexCount());
        if(find(descriptors.begin(), descriptors.end(), d) == descriptors.end()) {
            descriptors.push_back(d);
            res.push_back(r);
        }
    }

	log.back().merge_time += rolex_merge.Peek();
    log.back().time += rolex_merge.Peek();
	return res;
}

vector<Route> Labeling::Merge(double bound, size_t limit)
{
    return Merge(length[0] - 1, bound, limit);
}

void Labeling::SetBoundingLabels(Direction d, const vector<LabelPtr>* lbls, bool associated) 
{
    use_associated = associated;
    auto& bb = bounding_buckets[idx(d)];
    bb.assign(instance->VertexCount(), BoundingBucket(2, BoundingBucketTruck(instance->VertexCount())));

	for (auto l : *lbls) {
        if(l->TruckCarriesDrone()) 
            bb[instance->VertexCount()-l->length-1][0][l->truck].push_back(l.get());
        
        if(l->length <= instance->VertexCount() - 2) 
        if(instance->TruckServes() or l->TruckCarriesDrone())
            bb[instance->VertexCount()-l->length-2][1][l->truck].push_back(l.get());
    }

	for(auto& B : bb) for(auto e : {0,1}) for (auto& entry1: B[e])
        sort(entry1.begin(), entry1.end(), [](const Label* l1, const Label* l2) {return l1->CostToTruck() < l2->CostToTruck();});
        
}

void Labeling::SetExtensionCallback(const ExtensionCallback& cb)
{
    at_each_extension.reset(new ExtensionCallback(cb));
}

const vector<LabelPtr>* Labeling::Undominated(Direction d) const 
{
    return &labels[idx(d)];
}

const DualVector& Labeling::Duals() const 
{
    return duals;
}

const vector<LabelingLog>& Labeling::Log() const 
{
    return log;
}

const LabelingLog& Labeling::Log(size_t i) const 
{
    return log[i];
}


void Labeling::NewLogPage(const string& description)
{
    log.emplace_back(LabelingLog());
    log.back().description = description;
    for(auto d : {Direction::Forward, Direction::Backward}) {
        log.back().monolog[idx(d)].count_by_length.resize(instance->VertexCount());
    }
    
}

Route Labeling::RouteOf(Direction d, const Label* l) const
{
    auto r = BuildRouteOf(l);
    r.time = instance->RouteTime(d, r);
    return r;
}

Route Labeling::ReversedRouteOf(Direction d, const Label* l) const 
{
    auto r = BuildReversedRouteOf(l);
    r.time = instance->RouteTime(op(d), r);
    return r;
}

vector<LabelPtr> Labeling::ExtendUndominated(Direction d, double bound) const
{
    vector<LabelPtr> res;
    
    auto extensions = ExtendUndominatedCarry(d, bound);
    for(auto w : instance->Vertices()) {
        for(auto& [ignore, bucket] : extensions[w]) {
            if(bucket.carry != nullptr) res.push_back(bucket.carry);
            for(auto& m : bucket.routes) res.push_back(m);
        }
        
        if(relax_cost) continue;
        
        ExtendAloneToBuckets(d, &extensions, w, bound);
        
        vector<LabelPtr> temp;

        if(dominance_level == DominanceLevel::Fork) {
            auto sorted_buckets = SortBySize(&extensions[w]);
            vector<pair<VertexSet, ForkDominators>> dominators;
            for(auto& [forb, bucket] : sorted_buckets) {
                
                for(auto& [included, included_dominators] : dominators) {
                    if(bucket->alone.empty()) break;
                    if(included.count() >= forb.count()) break;
                    if(not is_subset(included, forb)) continue;
                    
                    vector<LabelPtr> new_bucket;
                    for(auto& m : bucket->alone) {
                        auto min_dual = -numeric_limits<double>::infinity();
                        if(auto b_it = extensions[m->prev->truck].find(included); b_it != extensions[m->prev->truck].end())
                            min_dual = m->prev->CostToTruck() - b_it->second.carry_cost;
                        
                        RemoveDualDominated(m.get(), included_dominators, min_dual);
                        RemoveForkDominated(m.get(), included_dominators);
                        
                        if(m->extensions_lbl.any()) {
                            new_bucket.push_back(m);
                        } else {
                            log.back().monolog[idx(d)].r_dominated_count += 1;
                        }
                    }
                    swap(bucket->alone, new_bucket);
                }
                
                if(bucket->alone.empty()) continue;
                ForkDominators fork_dominators;
                for(auto& m : bucket->alone) {
                    UpdateForkDominators(d, &fork_dominators, m.get());
                    temp.push_back(m);
                }
                dominators.push_back({forb, fork_dominators});
            }
        } else {
            for(auto& [forb, bucket] : extensions[w]) 
            for(auto& m : bucket.alone)
                temp.push_back(m);            
        }
        
        log.back().monolog[idx(d)].r_dominated_count += RemoveSortedDominated(d, &temp);
        
        res.insert(res.end(), temp.begin(), temp.end());
    }
    
    return res;
}

Labeling::LBLStorage Labeling::ExtendUndominatedCarry(Direction d, double bound) const
{
    LBLStorage undominated(instance->VertexCount());

    for(auto w : instance->Vertices()) {
        undominated[w] = ExtendCarryToBuckets(d, w, bound);
    
        auto sorted_buckets = SortBySize(&undominated[w]);
        for(auto& [forb, bucket] : sorted_buckets) { 
            for(auto& [included, included_bucket] : sorted_buckets) {
                if(bucket->carry == nullptr) break;
                if(included.count() >= forb.count()) break;
                if(is_subset(included, forb)) 
                if(epsilon_smaller_equal(included_bucket->carry_cost, bucket->carry_cost)) {
                    bucket->carry = nullptr;
                    bucket->carry_cost = included_bucket->carry_cost;
                    log.back().monolog[idx(d)].r_dominated_count += 1;
                }
            }
        }
    }
    return undominated;
}

Labeling::LBLForbStorage Labeling::ExtendCarryToBuckets(Direction d, goc::Vertex w, double bound) const
{
    LBLForbStorage extensions; 
        
    for(auto l : non_extended[idx(d)]) {
        auto extensions_carry = ExtensionsCarry(d, l, w, bound);
        
        if(relax_cost) {
            auto m = ExtensionAlone(d, l, w, bound);
            if(m != nullptr) extensions_carry.push_back(m);
        }
        
        for(auto& m : extensions_carry) {
            if(at_each_extension != nullptr) (*at_each_extension)(d, l, m.get());
                
            if(m->length == instance->VertexCount()-1) {
                extensions[VertexSet()].routes.push_back(m);
                continue;
            } 
                
            auto [temp, inserted] = extensions.insert({BucketKey(m.get()), LBLBucket()}); 
            auto& bucket = temp->second;
            if(inserted) {
                bucket.carry = m;
            } else if(epsilon_smaller_equal(m->CostToTruck(), bucket.carry_cost)) {
                swap(bucket.carry, m);
                log.back().monolog[idx(d)].b_dominated_count++; 
            } else {
                log.back().monolog[idx(d)].b_dominated_count++; 
            }
            bucket.carry_cost = bucket.carry->CostToTruck();
        }
    }
    
    return extensions;
}

void Labeling::ExtendAloneToBuckets(Direction d, LBLStorage* extensions, goc::Vertex w, double bound) const
{
    ExtendAloneToUnsortedBuckets(d, extensions, w, bound);
        
    for(auto& [forb, bucket] : (*extensions)[w]) {
        log.back().monolog[idx(d)].b_dominated_count += RemoveSortedDominated(d, &bucket.alone);
        reverse(bucket.alone.begin(), bucket.alone.end());
        log.back().monolog[idx(d)].b_dominated_count += RemoveSortedDominated(d, &bucket.alone);
    }
}

void Labeling::ExtendAloneToUnsortedBuckets(Direction d, LBLStorage* extensions, goc::Vertex w, double bound) const
{
    
    vector<LabelPtr> first_pass_extensions;
        
    unordered_map<VertexSet, ForkDominators> dominators;
    for(auto l : non_extended[idx(d)]) {
        auto m = ExtensionAlone(d, l, w, bound);
        if(m == nullptr) continue;
        if(at_each_extension != nullptr) (*at_each_extension)(d, l, m.get());
               
        if(dominance_level == DominanceLevel::Fork) {
            double min_dual = -numeric_limits<double>::infinity();
            if(auto b_it = (*extensions)[m->prev->truck].find(BucketKey(m->prev)); b_it != (*extensions)[m->prev->truck].end()) {
                min_dual = m->prev->CostToTruck() - b_it->second.carry_cost;
            }
            RemoveDualDominated(m.get(), dominators[BucketKey(m.get())], min_dual);
            RemoveForkDominated(m.get(), dominators[BucketKey(m.get())]);
            RemoveSelfDominated(d, m.get());
            UpdateForkDominators(d, &dominators[BucketKey(m.get())], m.get());
        
            if(m->extensions_lbl.any()) {
                first_pass_extensions.push_back(m);
            } else {
                log.back().monolog[idx(d)].b_dominated_count++; 
            }
        } else {
            (*extensions)[m->truck][BucketKey(m.get())].alone.push_back(m);
        }
    }
    if(dominance_level == DominanceLevel::NoFork) return;

    
    dominators = unordered_map<VertexSet, ForkDominators>();
    for(auto m_it = first_pass_extensions.rbegin(); m_it != first_pass_extensions.rend(); ++m_it) {

        RemoveDualDominated(m_it->get(), dominators[BucketKey(m_it->get())], -numeric_limits<double>::infinity());
        RemoveForkDominated(m_it->get(), dominators[BucketKey(m_it->get())]);
        UpdateForkDominators(d, &dominators[BucketKey(m_it->get())], m_it->get());
        
        if(m_it->get()->extensions_lbl.any()) {
            (*extensions)[(*m_it)->truck][BucketKey(m_it->get())].alone.push_back(*m_it);
        } else {
            log.back().monolog[idx(d)].b_dominated_count++; 
        }
    }
}



vector<LabelPtr> Labeling::ExtensionsCarry(Direction d, const Label* l, Vertex w, double bound) const
{
    vector<LabelPtr> successors;

    auto c = instance->VertexCount() - 2;
            
    if(w != instance->Origin(d))
    if(w != l->truck)
    if(l->TruckCarriesDrone())
    if((instance->Loops() and d == Direction::Forward) or (l->length == c) == (w == instance->Destination(d)))
    if(not l->forbidden_truck.test(w))
    if(instance->HasCombinedTruckArc(d, l->length, l->truck, w))
    {
        log.back().monolog[idx(d)].enumerated_count++;
        auto combined = new Label();
        
        combined->length = l->length + 1;
        combined->bifurcation = l->length + 1;
        combined->last = w;
        combined->truck = w;
        combined->drone = w;
        combined->cost = l->cost + instance->TruckTime(d,l->truck,w) - duals[w];
        combined->truck_time = 0;
        combined->forbidden_truck = l->forbidden_truck & ng->Neighborhood(w);
        if(ng->HasArc(w, l->truck)) combined->forbidden_truck.set(l->truck);
        combined->extensions_lbl = instance->VertexBitset();
        combined->extensions = instance->VertexBitset();
        combined->prev = l;

        if(Bounded(d, combined, bound))
            successors.push_back(LabelPtr(combined));
        else {
            delete combined;
            log.back().monolog[idx(d)].bounded_count++;
        }            
    }

    if(l->truck == w)
    if((instance->Loops() and l->truck != instance->Origin(Direction::Forward)) or not l->TruckCarriesDrone())
    if(instance->Loops() or l->drone != instance->Origin(d) or l->truck != instance->Destination(d)) //rare loops
    for(auto v : instance->Vertices())
    if(v != instance->Destination(d)) 
    if(not l->forbidden_truck.test(v))
    if(l->extensions_lbl.test(v))
    if(instance->HasLeg(d, l->bifurcation, l->length, l->drone, v, l->truck))
    {
        log.back().monolog[idx(d)].enumerated_count++;

        auto drone_alone = new Label();
        drone_alone->length = l->length + 1;
        drone_alone->bifurcation = l->length + 1;
        drone_alone->last = v;
        drone_alone->truck = l->truck;
        drone_alone->drone = l->truck;
        drone_alone->cost = l->cost + max(l->truck_time, instance->LegTime(d, l->drone, v, l->truck)) - duals[v] + instance->RendezvousTime(d, l->truck);
        drone_alone->truck_time = 0;
        drone_alone->forbidden_truck = l->forbidden_truck;
        if(ng->HasArc(l->truck, v)) drone_alone->forbidden_truck.set(v);
        drone_alone->extensions_lbl = instance->VertexBitset();
        drone_alone->extensions = instance->VertexBitset();
        drone_alone->prev = l;
        
        if(Bounded(d, drone_alone, bound))
            successors.push_back(LabelPtr(drone_alone));
        else {
            delete drone_alone;
            log.back().monolog[idx(d)].bounded_count++;
        }
        
    }

	return successors;
}

LabelPtr Labeling::ExtensionAlone(Direction d, const Label* l, Vertex w, double bound) const
{

    auto c = instance->VertexCount() - 2;
    Label* res = nullptr;

    if(w != instance->Origin(d))
    if(w != l->truck)
    if(l->length < c)
    if(instance->TruckServes() or l->TruckCarriesDrone())
    if((instance->Loops() and d == Direction::Forward) or (l->length == c-1) == (w == instance->Destination(d))) 
    if(instance->LandAndWait() or epsilon_smaller_equal(l->truck_time + instance->TruckTime(d, l->truck, w), instance->Range()))
    if(not l->forbidden_truck.test(w))
    if(instance->HasAloneTruckArc(d, l->length, l->truck, w))
    {
        auto alone_extensions_lbl = (l->TruckCarriesDrone() ? ~l->forbidden_truck & instance->ForksFrom(d, l->bifurcation, l->drone) : l->extensions_lbl) & ~VertexSet().set(w);
        if(alone_extensions_lbl.any()) 
        {
            auto truck_alone = new Label();
            log.back().monolog[idx(d)].enumerated_count++;
            truck_alone->extensions_lbl = alone_extensions_lbl;
            truck_alone->length = l->length + 1;
            truck_alone->bifurcation = l->bifurcation;
            truck_alone->last = w;
            truck_alone->truck = w;
            truck_alone->drone = l->drone;
            truck_alone->cost = l->cost - duals[w];
            if(l->TruckCarriesDrone()) truck_alone->cost += instance->LaunchTime(d, l->truck);
            truck_alone->truck_time = l->truck_time + instance->TruckTime(d, l->truck, w);
            truck_alone->forbidden_truck = l->forbidden_truck & ng->Neighborhood(w);
            if(ng->HasArc(w, l->truck)) truck_alone->forbidden_truck.set(l->truck);
            truck_alone->extensions = (l->TruckCarriesDrone() ? ~l->forbidden_truck & instance->ForksFrom(d, l->bifurcation, l->drone) : l->extensions) & ~VertexSet().set(w);
            truck_alone->prev = l;

            if(Bounded(d, truck_alone, bound))
                res = truck_alone;
            else {
                delete truck_alone;
                log.back().monolog[idx(d)].bounded_count++;
            }
            
        }
    }

    return LabelPtr(res);
}


size_t Labeling::RemoveSortedDominated(Direction d, vector<LabelPtr>* ms) const
{
    size_t res = 0;
    
    struct SameDominatorsBy {
        SameDominators other;
        SameDominators destination;
    };    
    unordered_map<VertexSet, SameDominatorsBy> dominators;
    
    vector<LabelPtr> temp;
    swap(*ms, temp);
        
    stable_sort(temp.begin(), temp.end(), [](const LabelPtr& l, const LabelPtr& m){
        return epsilon_smaller(l->CostToTruck(), m->CostToTruck());
    });    

    vector<unordered_map<VertexSet, void*>> bucket_keys(instance->VertexCount());
    for(auto& m : temp) {
        auto m_bucket_key = BucketKey(m.get());
        for(auto size = 0ul; size <= m_bucket_key.count(); ++size) {
            if(m->extensions_lbl.none()) break;
            for(auto [included, ignore] : bucket_keys[size]) {
                if(m->extensions_lbl.none()) break;
                if(not is_subset(included, m_bucket_key)) continue;
                
                auto& dom = m->drone == instance->Destination(Direction::Forward) ? dominators[included].destination : dominators[included].other;                
                RemoveSameDominated(d, m.get(), dom);
            }
        }
        if(m->extensions_lbl.any()) {
            ms->push_back(m);
            auto& dom = m->drone == instance->Destination(Direction::Forward) ? dominators[m_bucket_key].destination : dominators[m_bucket_key].other;                
            UpdateSameDominators(d, &dom, m.get());
            bucket_keys[m_bucket_key.count()].insert({m_bucket_key, nullptr});
        }
        else {
            res++; 
        }
    }
    return res;
}

void Labeling::RemoveSelfDominated(Direction d, const Label* m) const
{
    if(m->extensions_lbl.none()) return;
            
    for(auto i = 0ul; i < V_by_duals.size(); ++i) 
    if(auto v = V_by_duals[i]; m->extensions_lbl.test(v)) {        
        if(relax_ng or not ng->HasArc(m->truck, v))
        if(instance->HasLeg(d, m->bifurcation, m->length, m->drone, v, m->truck))
        if(epsilon_smaller_equal(ForkCost(d, m, v), m->CostToTruck())) {
            m->extensions_lbl &= dual_bigger[v];
            m->extensions_lbl.set(v);
            return;
        }
    }
}

void Labeling::RemoveDualDominated(const Label* m, const ForkDominators& dominators, double min_dual) const
{
    min_dual = max(min_dual, m->CostToTruck() - dominators.dual);
    
    size_t first_bigger = upper_bound(sorted_duals.begin(), sorted_duals.end(), min_dual, epsilon_smaller) - sorted_duals.begin();
    m->extensions_lbl &= bigger_dual[first_bigger];
}

void Labeling::RemoveForkDominated(const Label* m, const ForkDominators& dominators) const
{
    if(m->extensions_lbl.none()) return;
 
    size_t first_bigger = upper_bound(dominators.no_dual_values.begin(), dominators.no_dual_values.end(), m->CostToTruck(), epsilon_smaller) - dominators.no_dual_values.begin();
    if(first_bigger == 0) return;
    m->extensions_lbl &= dominators.no_dual_survive[first_bigger-1];
}

void Labeling::RemoveSameDominated(Direction d, const Label* m, const SameDominators& dominators) const
{
    if(dominators.size() == 0) return;
    auto count = m->extensions_lbl.count();
        
    auto rendezvous_cost = d == Direction::Forward ? 0 : instance->LaunchTime();
    
    for(auto i = 0ul; i < V_by_duals.size(); ++i) 
    if(auto v = V_by_duals[i]; m->extensions_lbl.test(v)) {
        for(auto& [truck_time, fork_time, cost] : dominators[v]) {
            if(epsilon_bigger(truck_time, m->truck_time)) break;
            if(epsilon_bigger(fork_time, instance->ForkTime(d, m->drone, v))) continue;
            if(epsilon_bigger(cost, m->cost + instance->ForkTime(d, m->drone, v))) 
            if(epsilon_bigger(cost + min(instance->Range()-fork_time, instance->JoinTime(d, v, m->truck)) + rendezvous_cost, m->CostToTruck()))    
                continue;
            
            
            m->extensions_lbl.reset(v);
            if(--count == 0) return;
            break;
        }
    }
}

void Labeling::UpdateForkDominators(Direction d, ForkDominators* dominators, const Label* m) const 
{    
    if(m->extensions_lbl.none()) return;

    auto no_dual = dominators->no_dual;

    for(auto i = 0ul; i < V_by_duals.size(); ++i)
    if(auto v = V_by_duals[i]; m->extensions_lbl.test(v))
    if(instance->HasLeg(d, m->bifurcation, m->length, m->drone, v, m->truck)) {
        no_dual.push_back({ForkCost(d, m, v), not relax_ng and ng->HasArc(m->truck, v), v});
            
        if(relax_ng or not ng->HasArc(m->truck, v))
        if(epsilon_smaller(ForkCost(d, m, v) - duals[v], dominators->dual)) {
            dominators->dual = ForkCost(d, m, v) - duals[v];
        }
    }
    
    dominators->no_dual.clear();
    dominators->no_dual_values.clear();
    dominators->no_dual_survive.clear();
    
    sort(no_dual.begin(), no_dual.end());
    for(auto [cost, single, v] : no_dual) {
        VertexSet survive = single ? instance->VertexBitset() : dual_bigger[v];
        survive.reset(v);
        if(not dominators->no_dual_survive.empty()) survive &= dominators->no_dual_survive.back();
        if(dominators->no_dual_survive.empty() or survive != dominators->no_dual_survive.back()) {
            dominators->no_dual_survive.push_back(survive);
            dominators->no_dual_values.push_back(cost);
            dominators->no_dual.push_back({cost, single, v});
        }
    }

}



void Labeling::UpdateSameDominators(Direction d, SameDominators* dominators, const Label* m) const
{
    if(m->extensions_lbl.none()) return;
    dominators->resize(instance->VertexCount(), RangeDominators());
    
    for(auto i = 0ul; i < V_by_duals.size(); ++i)
    if(auto v = V_by_duals[i]; m->extensions_lbl.test(v)) 
    if(instance->HasForkArc(d, m->bifurcation, m->drone, v)) {
        auto truck_time = instance->LandAndWait() ? -1 : m->truck_time;
        auto fork_time = instance->Range() == numeric_limits<double>::infinity() ? -1 : instance->ForkTime(d, m->drone, v);
        auto& doms = (*dominators)[v];

        InsertToRangeDominators(&doms, {truck_time, fork_time, m->cost + instance->ForkTime(d, m->drone, v)});
    }
}


bool Labeling::Bounded(Direction d, const Label* m, double bound) const
{    
    if(bound == goc::INFTY) return true;
    if(bounding_buckets[idx(d)][m->length][0].size() + bounding_buckets[idx(d)][m->length][1].size() == 0) return true;
    
    multimap<double, LBLMerge> bounds;
    DoMerge(d, m, bounding_buckets[idx(d)][m->length], bound, 1, 1, &bounds);
    if(use_associated and bounds.empty() and not m->TruckCarriesDrone())
        DoMerge(d, m->Associated(), bounding_buckets[idx(d)][m->length], bound, 1, 1, &bounds);
        
    return not bounds.empty();
}

void Labeling::DoMerge(
    Direction d,
    const Label& l,
    const BoundingBucket& B,
    double bound,
    size_t limit,
    size_t gen_limit,
    multimap<double, LBLMerge>* output
) const
{
    return DoMerge(d, &l, B, bound, limit, gen_limit, output);
}

void Labeling::DoMerge(
    Direction d,
    const Label* l,
    const BoundingBucket& B,
    double bound,
    size_t limit,
    size_t gen_limit,
    multimap<double, LBLMerge>* output
) const
{
    auto generated = 0ul;
        
    auto best_bound = [output, bound, limit]() -> double {
        return output->size() == limit ? output->rbegin()->first : bound;
    };
    
    auto best_push = [output, &generated, limit](double c, const Label* lbl, const Label* b, Vertex v = -1) {
        ++generated;
        output->insert({c,{lbl,b,v}});
        if(output->size() > limit) output->erase(prev(output->end()));
    };


    if(l->TruckCarriesDrone()) {
        for(auto b : B[0][l->truck]) {
            if(generated >= gen_limit) break;
            if(epsilon_bigger_equal(l->cost + b->cost + duals[l->truck], best_bound())) break;
            if((l->forbidden_truck & b->forbidden_truck).any()) continue;
            best_push(l->cost + b->cost + duals[l->truck], l, b);
        }
    } else {
        if(B[1][l->truck].empty()) 
            return;
        auto extensions = MergeExtensions(l, l->CostToTruck() + B[1][l->truck].front()->CostToTruck() + duals[l->truck] - bound);
        if(extensions.empty()) return;
        
        auto merge_cost = [&](const Label* lbl, const Label* b, Vertex w) -> double {
            return lbl->cost + b->cost + duals[lbl->truck] - duals[w] +
                   max(lbl->truck_time + b->truck_time, instance->LegTime(d, lbl->drone, w, b->drone)) +
                   (b->TruckCarriesDrone() ? instance->RendezvousTime(d, b->drone) : 0);
        };
        
        auto l_max_dual = duals[extensions.front()];
        for(auto& b : B[1][l->truck]) {
            if(generated >= gen_limit) break;
            if(epsilon_bigger_equal(l->CostToTruck() + b->CostToTruck() - l_max_dual + duals[l->truck], best_bound())) break;
            if((l->forbidden_truck & b->forbidden_truck).any()) continue;
            if(not instance->Loops() and l->drone == instance->Origin(d) and b->drone == instance->Destination(d)) continue; //rare loops
            if(not instance->LandAndWait() and epsilon_bigger(l->truck_time + b->truck_time, instance->Range())) continue;
            
            if(b->TruckCarriesDrone()) {
                for(auto w: extensions) {
                    if(epsilon_bigger_equal(l->CostToTruck() + b->CostToTruck() - duals[w] + duals[l->truck], best_bound())) break;
                    if(w != b->drone)
                    if(not b->forbidden_truck.test(w))
                    if(b->extensions.test(w))
                    if(instance->HasForkArc(op(d), b->bifurcation, b->drone, w))
                    if(epsilon_smaller_equal(instance->LegTime(d, l->drone, w, b->drone), instance->Range()))
                    if(epsilon_smaller(merge_cost(l,b,w), best_bound())) 
                    {
                        best_push(merge_cost(l, b, w), l, b, w);
                    }
                }
            } else {
                if((l->extensions & b->extensions_lbl).none()) continue;
                for(auto w: b->the_extensions_lbl) {
                    if(epsilon_bigger_equal(l->CostToTruck() + b->CostToTruck() - duals[w] + duals[l->truck], best_bound())) break;
                    if(w != l->drone)
                    if(not l->forbidden_truck.test(w))
                    if(l->extensions.test(w))
                    if(instance->HasForkArc(d, l->bifurcation, l->drone, w))
                    if(epsilon_smaller_equal(instance->LegTime(d, l->drone, w, b->drone), instance->Range()))
                    if(epsilon_smaller(merge_cost(l,b,w), best_bound())) 
                    {
                        best_push(merge_cost(l, b, w), l, b, w);
                    }
                }
            }
        }
    }
}


void Labeling::UpdateExtensionData(const Label* l) const
{
    if(l->TruckCarriesDrone()) return;
    
    if(relax_ng or relax_cost) 
        l->extensions_lbl = l->extensions;

    l->the_extensions_lbl.clear();
    if(l->prev->TruckCarriesDrone()) {
        for(auto i = 0ul; i < V_by_duals.size(); ++i) {
            auto v = V_by_duals[i];
            if(l->extensions_lbl.test(v)) 
                l->the_extensions_lbl.push_back(v);
        }
    } else {
        for(auto v : l->prev->the_extensions_lbl) {
            if(l->extensions_lbl.test(v)) 
                l->the_extensions_lbl.push_back(v);
        }
    }
}

vector<Vertex> Labeling::MergeExtensions(const Label* l, double min_dual) const
{
    vector<Vertex> res;
    
    if(not l->the_extensions_lbl.empty()) {
        for(auto v : l->the_extensions_lbl) {
            if(epsilon_smaller_equal(duals[v], min_dual)) break;
            res.push_back(v);
        }        
    } else if(l->prev->TruckCarriesDrone()) {
        for(auto i = 0ul; i < V_by_duals.size(); ++i) {
            auto v = V_by_duals[i];
            if(epsilon_smaller_equal(duals[v], min_dual)) break;
            if(l->extensions_lbl.test(v)) 
                res.push_back(v);
        }
    } else {
        for(auto v : l->prev->the_extensions_lbl) {
            if(epsilon_smaller_equal(duals[v], min_dual)) break;
            if(l->extensions_lbl.test(v)) 
                res.push_back(v);
        }
    }
    return res;
}

double Labeling::ForkCost(Direction d, const Label* l, goc::Vertex v) const {
    return max(l->CostToTruck(), l->cost + min(instance->LegTime(d, l->drone, v, l->truck), instance->Range())) + 
       (d == Direction::Forward ? 0 : instance->LaunchTime());
}


ostream& operator<<(ostream& os, DominanceLevel dominance_level) {
    switch(dominance_level) {
        case DominanceLevel::Fork:
            return os << "Fork";
        case DominanceLevel::NoFork:
            return os << "No Fork";
        default:
            return os << "ERROR!";
    }
}


vector<pair<VertexSet, Labeling::LBLBucket*>> Labeling::SortBySize(LBLForbStorage* buckets) const
{
    using res_type = vector<pair<VertexSet, Labeling::LBLBucket*>>;
    vector<res_type> sort_buckets;
    
    for(auto& [forb, bucket] : *buckets) {
        sort_buckets.resize(max(sort_buckets.size(), forb.count())+1);
        sort_buckets[forb.count()].push_back({forb, &bucket});
    }
    
    res_type res;
    for(auto& sb : sort_buckets) 
    for(auto& x : sb) 
        res.push_back(x);
        
    return res;
}

VertexSet Labeling::BucketKey(const Label* l) const {
    return relax_ng ? VertexSet() : l->forbidden_truck;
}

void Labeling::InsertToRangeDominators(RangeDominators* dominators, const tuple<Time, Time, double>& dom) const
{        
    for(auto& d : *dominators) {
        if(not Precedes(d, dom)) break;
        if(Dominates(d, dom)) return;
    }

    dominators->erase(remove_if(dominators->begin(), dominators->end(), [&dom](auto& d) {return Dominates(dom, d);}), dominators->end());
    
    dominators->insert(upper_bound(dominators->begin(), dominators->end(), dom, [](auto& d1, auto& d2) {return Precedes(d1, d2);}), dom);
}

} // namespace tspd
