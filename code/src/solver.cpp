//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include <climits>
#include <cmath>

#include "solver.h"

#include "ng_utils.h"
#include "time_guardian.h"
#include "mcmf_heuristic.h"

using namespace std;
using namespace goc;


namespace tspd
{    
    
Solver::Solver(const Subinstance& instance, const BitGraph& ng_graph) 
  : ng(ng_graph),
  	main(instance)
{   
    tstream = make_shared<TableStream>(&clog, 1.0);
    tstream->AddColumn("Time", 10).AddColumn("Process",20).AddColumn("Duration", 10).AddColumn("RMP Bound", 12).AddColumn("Pric-LB", 12).
            AddColumn("LB", 12).AddColumn("Prblm-LB", 12).AddColumn("Prblm-UB", 12).AddColumn("UB", 12).AddColumn("#Cols", 8).AddColumn("NG", 8).
            AddColumn("TArcs", 8).AddColumn("DArcs", 8);

    scf = make_shared<SCF>(main.instance.VertexCount(), main.ub);
    elemental_ng = ElementaryNG(instance);
        
    log = make_shared<SolverLog>();
    main.log->start_ng = main.log->final_ng = ng.ArcCount();
}

Solver::Subproblem::Subproblem(const Subinstance& the_instance) 
  : instance(the_instance) 
{
    ub = instance.NaiveRoute();
    log = make_shared<SolverProblemLog>();
}

SolverLog Solver::Run()
{
    Stopwatch rolex_main(true), rolex_subproblem;
    unique_ptr<Subproblem> current = nullptr;
    
    try {
        clog << endl << endl << time_guardian.Now() << "\tFirst Column Generation" << endl;
        tstream->WriteHeader();

        auto [lbl_main, price_bound_main] = ColGenPlus(&main, 0.005);
        
        log->root_time = time_guardian.Now();
        log->root_price_lb = price_bound_main;
        log->root_lb = LowerBound();
        log->root_ub = UpperBound();
        log->root_exact_labelings = main.log->exact_labelings_plus;
        log->root_total_labelings = main.log->exact_labelings + main.log->exact_labelings_plus;
        log->root_last_lbl = lbl_main->Log(0);

        Duration max_fix_time = lbl_main->Log(0).time;
        Duration min_fix_time = max_fix_time;

        while(not Solved(main)) {
            rolex_main.Pause();
            rolex_subproblem.Reset().Resume();
            current.reset(NewSubproblem(&main, main.lb, min(UpperBound(), ceil(LowerBound() * 1.0025 - 1) + 0.01)));
            auto price_bound_current = price_bound_main;
            unique_ptr<Labeling> lbl_current(new Labeling(*lbl_main));
            
            int augmented = 0;     // number of times NG was augmented since last time solved main was solved
            while(not Solved(*current)) {
                clog << endl << time_guardian.Now() << "\tSolving subproblem with fictitious upper bound " << current->ub.time << endl;
                tstream->WriteHeader();     

                ArcFixing(current.get(), lbl_current.get(), price_bound_current, max_fix_time, min_fix_time, true);
                if(Solved(*current)) 
                    break;
                
                NeighborhoodAugmentation(current.get(), lbl_current.get());
                log->DNA_iters++;
                augmented += 1;

                if(augmented % 3 == 0) {
                    clog << endl << time_guardian.Now() << "\tColGen of main problem with new NG after many retries " << current->ub.time << endl;
                    current.reset(NewSubproblem(&main, main.lb, min(UpperBound(), ceil(LowerBound() * 1.0025 - 1) + 0.01)));
                } else {
                    clog << endl << time_guardian.Now() << "\tColGen of subproblem with fictitious upper bound " << current->ub.time << " retry: " << augmented << endl;                    
                }

                tie(lbl_current, price_bound_current) = ColGenPlus(current.get(), 0.005);
                max_fix_time = max(max_fix_time, lbl_current->Log(0).time);
            }
            main.lb = ceil(current->ub.time);
            current->log->time = rolex_subproblem.Peek();
            Close(*current);
            if(log_level == LogLevel::All) log->subproblems.push_back(*current->log);
            current.reset(nullptr);

            if(Solved(main)) break;
            
            if(augmented > 0) {
                rolex_subproblem.Pause();
                rolex_main.Resume();

                clog << endl << time_guardian.Now() << "\tTry ficitituos duals on main" << endl;            
                auto [lbl_try, neg_routes, price_bound_try] = SolvePricing("Solution New", &main, lbl_current->Duals());
                max_fix_time = max(max_fix_time, lbl_try->Log(0).time);

                if(epsilon_smaller_equal(price_bound_main, price_bound_try)) {
                    swap(lbl_main, lbl_try);
                    price_bound_main = price_bound_try;
                } 
            }
        }
    } catch (TimeLimitExceeded& tle) {
        if(current != nullptr) {
            current->log->time = rolex_subproblem.Peek();
            Close(*current);
            if(log_level == LogLevel::All) log->subproblems.push_back(*current->log);
        }
        main.log->time = rolex_main.Peek();
        Close(main);
        if(log_level != LogLevel::None) log->main = *main.log;
        log->time = time_guardian.Now();
        log->status = SolverStatus::TimeLimitExceeded;
        throw tle;
    }
    
    main.log->time = rolex_main.Peek();
    Close(main);
    if(log_level != LogLevel::None) {
        log->main = *main.log;
    }
    log->time = time_guardian.Now();
    log->status = SolverStatus::Optimum;
    
	return *log;
}


pair<bool, DualVector> Solver::ColGen(
    Time objective, 
    bool resume,
    bool bidirectional,
    DominanceLevel dominance_level
)
{
    Stopwatch rolex(true);
    
    tstream->WriteHeader();
    unique_ptr<Labeling> lbl;
    try {
        double rmp_bound;
        tie(lbl, rmp_bound, ignore) = ColGen(&main, resume, objective, 1.1, false, bidirectional, dominance_level);
        FindSolutionIfOptimumRMPBound(lbl.get(), rmp_bound);
    }
    catch(TimeLimitExceeded& tle) 
    {
        main.log->time += rolex.Peek();
        log->root_exact_labelings = main.log->exact_labelings;
        throw tle;
    }
    
    main.log->time += rolex.Peek();
    log->root_exact_labelings = main.log->exact_labelings;
    log->root_total_labelings = main.log->exact_labelings_plus + main.log->exact_labelings;
    return {epsilon_smaller_equal(LowerBound(), objective), lbl->Duals()};
}

pair<bool, DualVector> Solver::ColGenPlus(Time objective, bool resume, double factor) 
{
    Stopwatch rolex(true);

    unique_ptr<Labeling> lbl;
    try {
        double rmp_bound;
        tie(lbl, rmp_bound) = ColGenPlus(&main, factor, resume, objective);
        FindSolutionIfOptimumRMPBound(lbl.get(), rmp_bound);        
    }
    catch(TimeLimitExceeded& tle) 
    {
        main.log->time += rolex.Peek();
        log->root_exact_labelings = main.log->exact_labelings_plus;
        throw tle;
    }
    
    main.log->time += rolex.Peek();
    log->root_exact_labelings = main.log->exact_labelings_plus;
    log->root_total_labelings = main.log->exact_labelings_plus + main.log->exact_labelings;
    return {epsilon_smaller_equal(LowerBound(), objective), lbl->Duals()};
}


pair<SolverPtr, SolverPtr> Solver::Branch()
{    
    auto opt = lp_solver.Solve(const_cast<Formulation*>(scf->RMPFormulation()), {LPOption::Incumbent}).incumbent;

    auto [left,right] = BranchVehicle(opt);
    if(left == nullptr) tie(left,right) = BranchTruckArcs(opt);
    if(left == nullptr) tie(left,right) = BranchDroneArcs(opt);

    for(auto s : {left,right}) if(s != nullptr) {
        s->id = next_id++;
        s->log = make_shared<SolverLog>();
        s->main.log = make_shared<SolverProblemLog>();
        s->log->description = s->Name();
        s->RebuildSPF();
    }
    return {left, right};
}

void Solver::Close() 
{
    Close(main);
    log->time = main.log->time;
    log->main = *main.log;   
}

const Route& Solver::BestSolution() const 
{
    return main.ub;
}

double Solver::LowerBound() const 
{
    return main.lb;
}

double Solver::UpperBound() const {
    return BestSolution().time;
}

bool Solver::Solved() const {
    return Solved(main);
}

void Solver::UpdateBestSolution(const Route& r) {
    UpdateBestSolution(&main, r);
}

size_t Solver::Id() const {
    return id;
}

std::string Solver::Name() const {
    return name;
}

const SolverLog& Solver::Log() const {
    return *log;
}

Solver::Subproblem* Solver::NewSubproblem(Subproblem* parent, double lb, double ub)
{
    auto res = new Subproblem(*parent);
    res->parent = parent;
    res->lb = ceil(lb);
    res->ub.time = max(lb, ub);
    res->log = make_shared<SolverProblemLog>();
    res->log->start_ub = res->ub.time;
    res->log->start_ng = res->log->final_ng = ng.ArcCount();
    return res;
}

tuple<unique_ptr<Labeling>, double, bool> Solver::ColGen(
    Subproblem* problem, 
    bool resume,
    Time objective, 
    double rmp_objective,
    bool fixing,
    bool bidirectional,
    DominanceLevel dominance_level
)
{
    ColGenState state = resume ? ColGenState::Exact : ColGenState::RelaxAll;  
    unique_ptr<Labeling> lbl;
    double pricing_bound = 0;
    bool solved_mp = false;
        
	while (state != ColGenState::Finished)
	{
        auto [rmp_bound, duals] = SolveRMP(problem);
        
        std::vector<Route> neg_routes;
        tie(lbl, neg_routes, pricing_bound) = SolvePricing("CG", problem, duals, state, rmp_bound, bidirectional, 
            dominance_level, state == ColGenState::Exact ? 200 : 50, 0);
        if(state == ColGenState::Exact) problem->log->exact_labelings += 1;
                
        if(Solved(*problem)) break; 

        bool added_routes = AddNegativeRoutesToSPF(neg_routes, duals);
        
        if(state == ColGenState::Exact) {
            solved_mp = not added_routes;
            if(fixing) ArcFixing(problem, lbl.get(), pricing_bound);    
            if(epsilon_bigger(problem->lb, objective) or epsilon_bigger(pricing_bound, rmp_bound*rmp_objective)) break;
        }
         
        if(not added_routes) state = Next(state);
	}
    return {std::move(lbl), pricing_bound, solved_mp};
}

std::pair<std::unique_ptr<Labeling>, double> Solver::ColGenPlus(
    Subproblem* problem,
    double factor,
    bool resume,
    Time objective    
)
{
    auto prev_labelings = problem->log->exact_labelings;
    auto [lbl_problem, price_bound_problem, solved_mp] = ColGen(problem, resume, objective, resume ? 0 : .9, false);
    auto lower_bound = ceil(price_bound_problem);
    problem->log->exact_labelings_plus += problem->log->exact_labelings - prev_labelings;
    problem->log->exact_labelings = prev_labelings;
    
    if(problem == &main) {
        log->initial_lb = LowerBound();
        log->initial_ub = UpperBound();
        log->initial_time = time_guardian.Now();
    }
        
    while(not solved_mp and not Solved(*problem) and epsilon_smaller(lower_bound, problem->ub.time)) 
    {
        Stopwatch current_time(true);
        auto current = unique_ptr<Subproblem>(NewSubproblem(problem, price_bound_problem, min(problem->ub.time, lower_bound * (1+factor))));
        auto price_bound_current = price_bound_problem;
        unique_ptr<Labeling> lbl_current(new Labeling(*lbl_problem));

        clog << endl << time_guardian.Now() << "\tResuming solution with fictitious upper bound " << current->ub.time << endl;
        tstream->WriteHeader();   
        current->prev_fixing_gap = goc::INFTY;
        ArcFixing(current.get(), lbl_current.get(), price_bound_current);
        bool solved_mp_current;
        tie(lbl_current, price_bound_current, solved_mp_current) = ColGen(current.get(), true);

        problem->log->lp_time += current->log->lp_time;
        problem->log->variable_fixing_time += current->log->variable_fixing_time;
        problem->log->pricing_time += current->log->pricing_time;
        problem->log->labelings.insert(problem->log->labelings.end(), current->log->labelings.begin(), current->log->labelings.end());
        problem->log->exact_labelings += current->log->exact_labelings;

        if(Solved() or epsilon_bigger(LowerBound(), objective)) break;

        if((solved_mp_current and epsilon_smaller(ceil(price_bound_current), current->ub.time)) or 
           current_time.Peek() >= lbl_problem->Log(0).time or
           epsilon_equal(current->ub.time, problem->ub.time)
        ) {
            auto [lbl_try, neg_routes, price_bound_try] = SolvePricing("Solution New", problem, lbl_current->Duals());

            AddNegativeRoutesToSPF(neg_routes, lbl_current->Duals());
            
            if(epsilon_smaller_equal(price_bound_problem, price_bound_try)) {
                swap(lbl_problem, lbl_try);
                price_bound_problem = price_bound_try;
                solved_mp = epsilon_bigger_equal(neg_routes[0].ReducedCost(lbl_problem->Duals()), 0);
            }
            problem->log->exact_labelings_plus += 1;
        }        
        lower_bound = current->ub.time;
        
    }
    return {std::move(lbl_problem), price_bound_problem};
}



void Solver::NeighborhoodAugmentation(Subproblem* problem, Labeling* last_pricer)
{
    Stopwatch rolex(true);
    
    auto to_separate = last_pricer->Merge(numeric_limits<double>::infinity(), 100);
    auto separated = DoSeparate(&ng, last_pricer->Duals(), to_separate);
    if(separated) RebuildSPF();

    problem->log->final_ng = ng.ArcCount();
    problem->log->DNA_time += rolex.Peek();
}

tuple<unique_ptr<Labeling>, vector<Route>, double> Solver::SolvePricing(
    const string& description,
    Subproblem* problem, 
    const vector<double>& duals,
    ColGenState state,
    double fail_ub,
    bool bidirectional, 
    DominanceLevel dominance_level,
    size_t route_limit,
    double bound
) {
    auto lbl = make_unique<Labeling>(description + " " + STR(state), &problem->instance, &ng, duals,
        dominance_level, state == ColGenState::RelaxAll or state==ColGenState::RelaxNG, state == ColGenState::RelaxCost or state == ColGenState::RelaxAll);
    lbl->Run(bidirectional ? problem->instance.VertexCount()/2 : problem->instance.VertexCount()-1);
    auto routes = lbl->Merge(bound, route_limit);
    problem->log->labelings.push_back(lbl->Log().back());
    problem->log->pricing_time += lbl->Log().back().time;

    auto price_bound = UpdateBounds(problem, fail_ub, duals, routes, state);
    
    StreamInfo(problem, description + " " + STR(state), lbl->Log().back().time, price_bound, STR(fail_ub));

    return {std::move(lbl), routes, price_bound};
}


double Solver::UpdateBounds(Subproblem* problem, double fail_ub, const vector<double>& duals, const std::vector<Route>& routes, ColGenState state) {
    
    UpdateBestSolution(problem, routes);
    
    auto pricing_bound = routes.empty() ? fail_ub : routes[0].LowerBound(duals);
    if(state == ColGenState::Exact or state == ColGenState::Finished) {
        problem->log->lb = problem->lb = min(problem->ub.time, ceil(max(problem->lb, pricing_bound) - EPS));
        for(auto parent = problem->parent; parent != nullptr; parent = parent->parent)
            parent->lb = max(parent->lb, problem->lb);
        problem->log->labelings.back().pricing_value = pricing_bound; 
    }
        
    return pricing_bound;
}

pair<double, vector<double>> Solver::SolveRMP(Subproblem* problem) {

    auto lp_log = lp_solver.Solve(const_cast<Formulation*>(scf->RMPFormulation()), {LPOption::Duals, LPOption::Incumbent});
    problem->log->lp_time += lp_log.time;

    if (lp_log.status == LPStatus::MemoryLimitReached) {
        fail("Memory limit reached");
    } else if (lp_log.status == LPStatus::Unbounded or lp_log.status == LPStatus::Infeasible) {
        fail("Relaxation can not be unbounded or infeasible.");
    }
    return {lp_log.incumbent_value, lp_log.duals};
}

bool Solver::AddNegativeRoutesToSPF(const std::vector<Route>& routes, const std::vector<double>& duals) {

    bool res = false;
    auto redbest = BestSolution().ReducedCost(duals);
    for (auto &r: routes)
    {
        //OK, tuve que hacer esto de sacar las elementales porque sino entra en tailing off.
        auto redcost = r.ReducedCost(duals);
        if(not r.IsElementary() and epsilon_smaller(redcost, redbest) and epsilon_smaller(redcost, 0)) {
            scf->AddRoute(r);
            res = true;
        }
    }
    return res;
}

void Solver::UpdateBestSolution(Subproblem* problem, Route r) {
    if(not r.IsElementary()) return;
    
    if(epsilon_smaller(r.time, problem->ub.time)){ 
        problem->ub = r;
    }
    if(epsilon_smaller(r.time, main.ub.time)) {
        main.ub = r;
    }
    problem->log->ub = UpperBound();
}

void Solver::UpdateBestSolution(Subproblem* problem, const std::vector<Route>& routes) {
    for(auto& r : routes) UpdateBestSolution(problem, r);
}


bool Solver::DoSeparate(BitGraph* the_ng, const vector<double>& duals, const vector<Route>& routes) {
    vector<size_t> max_ng_of(main.instance.VertexCount());
    for(auto v : main.instance.Vertices()) max_ng_of[v] = the_ng->Neighborhood(v).count();
    auto reached = *max_element(max_ng_of.begin(), max_ng_of.end());
    for(auto& x : max_ng_of) x = reached + 1;

    vector<Route> cero;
    for(auto& r : routes) if(epsilon_equal(r.ReducedCost(duals), routes[0].ReducedCost(duals))) 
        cero.push_back(r);

    AugmentNG(the_ng, cero, max_ng_of);
    AugmentNG(the_ng, routes, max_ng_of);
    return true;
}

Duration Solver::ArcFixing(
    Subproblem* problem,
    Labeling* lbl, 
    double price_bound, 
    const Duration& max_price_time, 
    const Duration& min_fix_time,     
    bool iterative
) {    
    Stopwatch rolex_fixer(true);
    
    auto gap = problem->ub.time - price_bound;
    if(not iterative and epsilon_smaller(problem->prev_fixing_gap * .9, gap)) 
        return Duration();
    problem->prev_fixing_gap = gap;

    auto fix_dir = Direction::Forward;
    auto fix_bound = problem->ub.ReducedCost(lbl->Duals());
    auto fix_ng = ng;
    
    auto fixer = RunVariableFixing(problem, fix_dir, lbl, fix_ng, fix_bound, true);
    auto fix_time = fixer->Log().back().monolog[0].time;
    StreamInfo(problem, "Variable Fixing", fix_time, price_bound);
    
    do {
        auto separate = fixer->Merge(numeric_limits<double>::infinity(), 100);
        for(auto& r : separate)
            UpdateBestSolution(problem, MinCostMaxFlowHeuristic(main.instance, r));
        
        rolex_fixer.Pause();
        auto fix_price_bound = UpdateBounds(problem, problem->ub.time, fixer->Duals(), separate, ColGenState::Finished);
        rolex_fixer.Resume();
        fix_bound = problem->ub.ReducedCost(lbl->Duals());
        problem->prev_fixing_gap = problem->ub.time - price_bound;
        
        if(Solved(*problem) or separate.empty() or not DoSeparate(&fix_ng, lbl->Duals(), separate)) break;
                
        fix_dir = op(fix_dir);
        fixer = RunVariableFixing(problem, fix_dir, fixer.get(), fix_ng, fix_bound, false);
        StreamInfo(problem, "Variable Fixing", fixer->Log().back().monolog[idx(fix_dir)].time, fix_price_bound);
                
    } while(iterative and not Solved(*problem) and 
        (fix_time < max_price_time or fixer->Log().back().monolog[idx(fix_dir)].time > min_fix_time) and 
        fixer->Log().back().monolog[idx(fix_dir)].time < max_price_time);

    auto separate = fixer->Merge(numeric_limits<double>::infinity(), 100);
    for(auto& r : separate)
        UpdateBestSolution(problem, MinCostMaxFlowHeuristic(main.instance, r));

    if(iterative) problem->log->iterative_fixing_time += rolex_fixer.Peek();
    else problem->log->variable_fixing_time += rolex_fixer.Peek();
    return rolex_fixer.Peek();
}


unique_ptr<Labeling> Solver::RunVariableFixing(
    Subproblem* problem,
    Direction d,
    Labeling* lbl, 
    const BitGraph& the_ng,
    double discard_bound,
    bool use_associated
) 
{    
    auto n = problem->instance.VertexCount();    
    discard_bound += goc::EPS;

    lbl->NewLogPage("Completion bounds for Variable Fixing");
    lbl->SetBoundingLabels(op(d), lbl->Undominated(d), use_associated);
    lbl->MRun(op(d), n-1, discard_bound);
    if(lbl->Log().back().monolog[opidx(d)].enumerated_count > 0) problem->log->labelings.push_back(lbl->Log().back());

    LenBitGraph alone(n-1,n), join(n-1,n);
    auto fixer = make_unique<Labeling>("Variable Fixing", &problem->instance, &the_ng, lbl->Duals(), DominanceLevel::NoFork);
    fixer->SetBoundingLabels(d, lbl->Undominated(op(d)), use_associated);
    
    fixer->SetExtensionCallback(
        [&alone,&join](Direction, const Label* l, const Label* m) {

            if(not m->TruckCarriesDrone())
                alone.AddArc(l->length, l->truck, m->truck);
            else if(not l->TruckCarriesDrone() or l->truck == m->truck)
                join.AddArc(l->length, m->last, m->truck);
        }
    );
    fixer->MRun(d, n-1, discard_bound);
    
    for(auto op = 0ul; op < n-1; ++op) for(auto v : problem->instance.Vertices()) for(auto w : problem->instance.Vertices()) {
        if(not alone.HasArc(op, v, w)) problem->instance.RemoveAloneTruckArc(d, op, v, w);
        if(not join.HasArc(op, v, w)) problem->instance.RemoveJoinArcAt(d, op, v, w);
    }
        
    problem->log->labelings.push_back(fixer->Log().back());
    
    return fixer;
}

void Solver::FindSolutionIfOptimumRMPBound(Labeling* lbl, double rmp_bound) {
    if(not Solved(main) and epsilon_equal(rmp_bound, floor(rmp_bound + 2*goc::EPS))) {

        auto n = main.instance.VertexCount();    

        auto lbl2 = make_unique<Labeling>("Lbl", &main.instance, &ng, lbl->Duals(), DominanceLevel::Fork);
        lbl2->Run(n/2);
        
        lbl->SetBoundingLabels(Direction::Forward, lbl->Undominated(Direction::Backward), true);
        lbl->MRun(Direction::Forward, n-1, 2*goc::EPS);
    
        auto checker = make_unique<Labeling>("Checker", &main.instance, &elemental_ng, lbl->Duals(), DominanceLevel::NoFork);
        checker->SetBoundingLabels(Direction::Backward, lbl->Undominated(Direction::Forward), true);
        checker->MRun(Direction::Backward, n-1, 2*goc::EPS);
        UpdateBestSolution(&main, checker->Merge(2*goc::EPS, 1));            
        StreamInfo(&main, "Checking LB", checker->Log().back().monolog[idx(Direction::Backward)].time, 0);
    }    
}
    

pair<SolverPtr, SolverPtr> Solver::BranchVehicle(const Valuation& opt) const
{

	vector<double> x(main.instance.VertexCount(), 0.0), y(main.instance.VertexCount(), 0);
	for (auto &[var, val]: opt) {
        auto& r = scf->RouteOf(var);
        for (auto d : r.drone) x[d] += val;
        for (auto t : r.truck) y[t] += val;            
    }
	
	vector<Vertex> x_most;

	for (Vertex v : main.instance.Vertices()) 
	if (epsilon_bigger(x[v], 0.0) and epsilon_smaller(x[v], 1.0) and epsilon_bigger(y[v], 0) and epsilon_smaller(y[v], 1)) {
        x_most.push_back(v);
    }
    if(x_most.empty()) return {nullptr, nullptr};

    auto v = *std::min_element(x_most.begin(), x_most.end(), 
		 [&](Vertex u, Vertex w) { return epsilon_smaller(fabs(0.5 - x[u]), fabs(0.5 - x[w])); });

    auto left = make_unique<Solver>(*this), right = make_unique<Solver>(*this);

    for (Vertex w: main.instance.Vertices()) {
        // Left node: v is visited by a drone, thus w->v and v->w are not truck arcs
        left->name = name + ", " + STR(v) + "d";
        left->main.instance.RemoveTruckArc(w, v);
        left->main.instance.RemoveTruckArc(v, w);
        // Right node: v is visited by a truck, thus w->v is not a fork arc and v->w is not a join arc
        right->name = name + ", " + STR(v) + "t";
        right->main.instance.RemoveForkArc(w, v);
        right->main.instance.RemoveJoinArc(v, w);
    }
    return {std::move(left),std::move(right)};
}

pair<SolverPtr, SolverPtr> Solver::BranchTruckArcs(const goc::Valuation& opt) const
{
	Matrix<double> x(main.instance.VertexCount(), main.instance.VertexCount(), 0.0);
	for (auto &[var, val]: opt)
	{
        auto& r = scf->RouteOf(var);
		for (auto k = 0ul; k < r.truck.size()-1; ++k) x[r.truck[k]][r.truck[k + 1]] += val;
	}

	vector<Arc> x_most;

	for (Vertex u : main.instance.Vertices()) for (Vertex v : main.instance.Vertices())
	if (u != v and epsilon_bigger(x[u][v], 0.0) and epsilon_smaller(x[u][v], 1.0)) {
        x_most.push_back({u, v});
    }
    if(x_most.empty()) return {nullptr, nullptr};

    auto e = *min_element(x_most.begin(), x_most.end(), 
		 [&](Arc a, Arc b) { return epsilon_smaller(fabs(0.5 - x[a.tail][a.head]), fabs(0.5 - x[b.tail][b.head])); });

    auto left = make_unique<Solver>(*this), right = make_unique<Solver>(*this);
    // Left node (x_e = 0).  
    //The vertices can be visited by either the drone or the truck, but the truck cannot take e
    left->name = name + ", " + STR(e) + "nt";
    left->main.instance.RemoveTruckArc(e.tail, e.head);

    // Right node (x_e = 1) and the truck takes e.
    for (Vertex j: main.instance.Vertices()) {            
        //e.head  and e.tail must be visited by the truck
        right->name = name + ", " + STR(e) + "t";
        right->main.instance.RemoveForkArc(j, e.head);
        right->main.instance.RemoveForkArc(j, e.tail);
        right->main.instance.RemoveJoinArc(e.head, j);
        right->main.instance.RemoveJoinArc(e.tail, j);
        
        //the only edge possible for the truck is e.tail -> e.head
        if(j != e.head) right->main.instance.RemoveTruckArc(e.tail, j);
        if(j != e.tail) right->main.instance.RemoveTruckArc(j, e.head);
    }
    return {std::move(left), std::move(right)};
}

pair<SolverPtr, SolverPtr> Solver::BranchDroneArcs(const Valuation& opt) const
{
	Matrix<double> x(main.instance.VertexCount(), main.instance.VertexCount(), 0.0);
	for (auto &[var, val]: opt)
	{
		auto& r = scf->RouteOf(var);
        for(auto i = 0ul; i < r.drone.size(); ++i) {
            auto [f,d,_] = r.Leg(i);
            x[f][d] += val;
        }
	}

	vector<Arc> x_most;

	for (Vertex u : main.instance.Vertices()) for (Vertex v : main.instance.Vertices())
	if (u != v and epsilon_bigger(x[u][v], 0.0) and epsilon_smaller(x[u][v], 1.0)) {
        x_most.push_back({u, v});
    }
	if (x_most.empty()) return {nullptr, nullptr};

    auto e = *min_element(x_most.begin(), x_most.end(), 
		 [&](Arc a, Arc b) { return epsilon_smaller(fabs(0.5 - x[a.tail][a.head]), fabs(0.5 - x[b.tail][b.head])); });

    auto left = make_unique<Solver>(*this), right = make_unique<Solver>(*this);
    // Left node: x_e = 0
    //The vertices can be visited by either the drone or the truck, but the drone cannot split via e
    left->name = name + ", " + STR(e) + "nd";
    left->main.instance.RemoveForkArc(e.tail, e.head);

    // Right node: x_e = 1 and the drone splits from e.
    for (Vertex j: main.instance.Vertices()) {
        // e.head must be visited by the drone alone.
        right->name = name + ", " + STR(e) + "d";
        right->main.instance.RemoveTruckArc(e.head, j);
        right->main.instance.RemoveTruckArc(j, e.head);
        
        //e.tail must be visited by the truck
        right->main.instance.RemoveForkArc(j, e.tail);
        right->main.instance.RemoveJoinArc(e.tail, j);
        
        //the only edge possible for the drone to split at e.tail or to visit e.head is e.tail -> e.head
        if(j != e.head) right->main.instance.RemoveForkArc(e.tail, j);
        if(j != e.tail) right->main.instance.RemoveForkArc(j,e.head);
    }
    return {std::move(left), std::move(right)};
}



void Solver::StreamInfo(Subproblem* problem, const std::string& process, const Duration& time, double pricing_bound, const string& rmp_bound) const
{
    tstream->WriteRow({STR(time_guardian.Now()), process, STR(time), STR(rmp_bound), STR(pricing_bound), STR(main.lb), 
        STR(problem->lb), STR(problem->ub.time), STR(UpperBound()), STR(scf->RMPFormulation()->VariableCount()), STR(ng.ArcCount()), 
        STR(problem->instance.AloneTruckArcsCount(Direction::Forward)), STR(problem->instance.DroneArcsCount(Direction::Forward))});    
}

void Solver::Close(const Subproblem& problem)
{
	log->lb = main.log->lb = max(LowerBound(), problem.log->lb);
	log->ub = main.log->ub = min(UpperBound(), problem.log->ub);
    main.log->final_ng = problem.log->final_ng;
	log->DNA_time += problem.log->DNA_time;
	log->lp_time += problem.log->lp_time;
	log->pricing_time += problem.log->pricing_time;
	log->variable_fixing_time += problem.log->variable_fixing_time;
    log->iterative_fixing_time += problem.log->iterative_fixing_time;
}

std::ostream& operator<<(std::ostream& os, ColGenState state) {
    switch(state) {
        case ColGenState::RelaxAll:
            return os << "Relax All";
        case ColGenState::RelaxCost:
            return os << "Relax Cost";
        case ColGenState::RelaxNG:
            return os << "Relax NG";
        case ColGenState::Exact:
            return os << "Exact";
        default:
            return os << "Finished";
    }
}

VariableFixingLog Solver::VariableFixingExperiment(const std::vector<double>& gaps, double time_limit_factor)
{
    VariableFixingLog res;
    auto get_arcs = [](const Subproblem& p) {
        auto& i = p.instance;
        return i.DroneArcsCount(Direction::Forward) + i.DroneArcsCount(Direction::Backward) + 
               i.TruckArcsCount(Direction::Forward) + i.TruckArcsCount(Direction::Backward);
    };
    
    res.total_arcs = get_arcs(main);
    auto [lbl, price_bound, _] = ColGen(&main, false, goc::INFTY, .9, false);
    
    res.duals = lbl->Duals();
    res.lbl_pre_time = lbl->Log(0).time;
    
    for(auto factor : gaps) 
    {
        auto current = unique_ptr<Subproblem>(NewSubproblem(&main, price_bound, price_bound * (1+.01*factor)));
        unique_ptr<Labeling> lbl_current(new Labeling(*lbl));

        clog << endl << time_guardian.Now() << "\tExperiment with upper bound " << current->ub.time << endl;
        tstream->WriteHeader();   
        current->prev_fixing_gap = goc::INFTY;
        
        auto start = time_guardian.Now();
        time_guardian.SetTimeLimit(start + res.lbl_pre_time * time_limit_factor);
        bool fixed = false;
        try {
            ArcFixing(current.get(), lbl_current.get(), price_bound);        
            fixed = true;
        } catch(TimeLimitExceeded& tle) {
            clog << "Time limit" << endl;
        }
        res.time.push_back(time_guardian.Now() - start);
        res.gap.push_back(factor);
        res.arcs.push_back(get_arcs(*current));

        if(fixed) {
            time_guardian.SetTimeLimit(time_guardian.Now() + res.lbl_pre_time*1000);
            try {
                auto [lbl_try, neg_routes, price_bound_try] = SolvePricing("Try pricing", current.get(), lbl_current->Duals());
                res.lbl_post_time.push_back(lbl_try->Log(0).time);
            } catch(TimeLimitExceeded& tle) {
                res.lbl_post_time.push_back(goc::Duration());
            }
        } else {
            res.lbl_post_time.push_back(goc::Duration());
        }
        
    }        
    return res;
}

} // namespace tspd

