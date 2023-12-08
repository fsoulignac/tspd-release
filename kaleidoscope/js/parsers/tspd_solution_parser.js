class TSPDSolutionParser extends Parser
{
  get_attributes(obj)
  {
    return [];
  }

  detail_view_rows(obj, view_section) {
    view_section.add_label_row("Value", this.f2(obj.time));
    view_section.add_label_row("Truck", this.jsonify(obj.truck));
    view_section.add_label_row("Drone", this.jsonify(obj.drone));
    view_section.add_label_row("Forks", this.jsonify(obj.bifurcation));
  }
}
kd.add_parser("tspd_route", new TSPDSolutionParser());
