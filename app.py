from MissionScenario import MissionScenario
from ZoningProblem import ZoningSolution, ZoningAlgorithm, RoutingAlgorithm
from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route("/", methods=['GET', 'POST'])
def index():
  jsonresp = request.json
  

  scenario = MissionScenario.load(jsonresp)

  

  z_algo = ZoningAlgorithm(scenario)
  [zoning_sol, envelope] = z_algo.search()


  #zoning_sol = scenario.zoning_sol


  ## zone plygones
  zone_polygons = []
  for i in range(1, scenario.nb_agents+1):
    zone_poly = zoning_sol.separators[i-1] + zoning_sol.separators[i][::-1]
    zone_polygons.append(zone_poly)

  ## render solution as JSON output
  solution = {}
  solution['zone_polygons'] = zone_polygons
  solution['envelope'] = envelope


  # if routing is required
  if int(jsonresp['use_routing'])==1:
    ## routes
    r_algo = RoutingAlgorithm(scenario)
    [sm_routes, routes, dropped] = r_algo.search()

    solution['routes'] = list(sm_routes)
  print(solution)
  return jsonify(solution)

if __name__=='__main__':
  app.run(debug=True, host='0.0.0.0')
