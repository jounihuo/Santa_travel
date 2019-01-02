##https://developers.google.com/optimization/routing/cvrp

from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

import numpy as np
from math import sin, cos, sqrt, atan2, radians, ceil

###########################
# Problem Data Definition #
###########################
def create_data_model(i):
  """Stores the data for the problem"""
  data = {}

  datafile = 'nicelist'+str(i)+'.csv'

  #data_txt = np.genfromtxt('nicelist.txt', delimiter=';', max_rows=2000)
  data_txt = np.genfromtxt(datafile, delimiter=';')
  
  print('Number of locations')
  print(len(data_txt[:,0]))
  
  #Creating locations list from data and adding the starting point
  _locations = [(i,j) for i,j in data_txt[:,1:3]]
  _locations.insert(0, (68.073611, 29.315278))

  demands = [i for i in data_txt[:,3]]
  demands.insert(0,0)

  n_runs = int(np.ceil(sum(demands)/10000000.)+1)

  capacities = [10000000 for i in range(n_runs)]

  #Compiling data
  data["locations"] = _locations
  data["num_locations"] = len(data["locations"])
  data["num_vehicles"] = len(capacities)
  data["depot"] = 0
  data["demands"] = demands
  data["vehicle_capacities"] = capacities
  data["id"] = data_txt[:,0]
  return data
 

def split_data(ndivs):
    data_txt = np.genfromtxt('nicelist.txt', delimiter=';')
    data_txt = data_txt[data_txt[:,2].argsort()]

    nlen = data_txt.shape[0]/ndivs

    print('Separating data')
    for i in range(ndivs):
        temp = data_txt[i*nlen:i*nlen+nlen,:]
        filename = 'nicelist'+str(i)+'.csv'
        print('Writing...')
        print(filename)
        print('')
        np.savetxt(filename, temp, delimiter=";")
    print('Files done.') 
#######################
# Problem Constraints #
#######################

def latlong_distance(position_1, position_2):
	# approximate radius of earth in km
	# https://stackoverflow.com/questions/19412462/getting-distance-between-two-points-based-on-latitude-longitude
	R = 6378.0

	lat1 = radians(position_1[0])
	lon1 = radians(position_1[1])
	lat2 = radians(position_2[0])
	lon2 = radians(position_2[1])

	dlon = lon2 - lon1
	dlat = lat2 - lat1

	a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))

	distance = R * c
	
	return distance
	  
def create_distance_callback(data):
  """Creates callback to return distance between points."""
  _distances = {}

  for from_node in range(data["num_locations"]):
    _distances[from_node] = {}
    for to_node in range(data["num_locations"]):
      if from_node == to_node:
        _distances[from_node][to_node] = 0
      else:
        _distances[from_node][to_node] = (
            latlong_distance(data["locations"][from_node],
                               data["locations"][to_node]))

  def distance_callback(from_node, to_node):
    """Returns the distance between the two nodes"""
    return _distances[from_node][to_node]

  return distance_callback

def create_demand_callback(data):
    """Creates callback to get demands at each location."""
    def demand_callback(from_node, to_node):
        return data["demands"][from_node]
    return demand_callback

def add_capacity_constraints(routing, data, demand_callback):
    """Adds capacity constraint"""
    capacity = "Capacity"
    routing.AddDimensionWithVehicleCapacity(
        demand_callback,
        0, # null capacity slack
        data["vehicle_capacities"], # vehicle maximum capacities
        True, # start cumul to zero
        capacity)
###########
# Printer #
###########
def print_solution(data, routing, assignment):
    """Print routes on console."""
    total_dist = 0
    f = open('result.txt', 'a')
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for run {0}:\n'.format(vehicle_id)
        route_dist = 0
        route_load = 0
        path_out = []
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
            route_dist += latlong_distance(
                data["locations"][node_index],
                data["locations"][next_node_index])
            route_load += data["demands"][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            index = assignment.Value(routing.NextVar(index))
            path_out.append(node_index)

            
        #Write file out
        out_txt = ''
        path_out.pop(0)
        for i in path_out:
            out_txt += str(int(data["id"][i-1])) + ';' 
        #print(path_out)
        out_txt = out_txt[:-1]
        out_txt += '\n'
        if len(out_txt)>2:
          f.write(out_txt)
        node_index = routing.IndexToNode(index)
        total_dist += route_dist
        plan_output += ' {0} Load({1})\n'.format(node_index, route_load)
        plan_output += 'Distance of the route: {0} km\n'.format(route_dist)
        plan_output += 'Load of the route: {0}\n'.format(route_load)
        
        print(plan_output)
    print('Total Distance of all routes: {0} km'.format(total_dist))
    f.close()
########
# Main #
######## 

def main():
  """Entry point of the program"""
  #Number of data splits
  ndivs = 2
  split_data(ndivs)
  for i in range(ndivs):
    # Instantiate the data problem.
    data = create_data_model(i)
    # Create Routing Model  
    routing = pywrapcp.RoutingModel(
      data["num_locations"],
      data["num_vehicles"],
      data["depot"])
    # Define weight of each edge
    distance_callback = create_distance_callback(data)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
  # Add Capacity constraint
    demand_callback = create_demand_callback(data)
    add_capacity_constraints(routing, data, demand_callback)
    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()	  
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)	 
		# Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
      print_solution(data, routing, assignment)
if __name__ == '__main__':
  main()
