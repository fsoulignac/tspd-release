class TSPDSolverProblemParser extends Parser
{
  get_attributes(obj)
  {
    return [];
  }

  detail_view_rows(problem, view_section) {

    view_section.add_table_row(
      ["Time", "Final LB", "Start UB", "Final UB"],
      [problem.time, problem.lb, problem.start_ub, problem.ub]
    );
    
    view_section.add_table_row(
      ["Start NG", "Final NG", "LP time"],
      [problem.start_ng, problem.final_ng, problem.lp_time]
    );
    
    view_section.add_table_row(
      ["Pricing time", "DNA time", "Fixing time", "Iterative Fixing time"],
      [problem.pricing_time, problem.dna_time, problem.fix_time, problem.iterative_fix_time]
    );
    
    view_section.add_label_row("Pricing iterations", " ");
    view_section.add_slider_row(
      problem.labelings,
      (iter, lbl, lbl_section) => {
        const name = lbl.description || "";
        lbl_section.add_html_row(`<span class="slider_header">#${iter+1} ${name}</span>`);
        kd.get_parser(lbl.kd_type)?.detail_view_rows(lbl, lbl_section);
      }
    );
  }
}
kd.add_parser("tspd_solver_problem", new TSPDSolverProblemParser());
