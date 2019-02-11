import os
import xmltodict
from frvcp_py.labelalgo.solver import Solver
import convertOldInstances

# define function to get routes given an instance
def get_routes(instance_name, routes_folder):
  routes = []
  
  # get the names of the route files corresponding to the given instance
  route_filenames = [
    os.fsdecode(route_filename) for route_filename in os.listdir(os.fsencode(routes_folder)) \
    if os.fsdecode(route_filename).startswith(f'route_{instance_name[:-4]}_')
  ]

  # get the routes from those files
  # iterate over the files
  for route_filename in route_filenames:
    route_fullname = f'{routes_folder}/{route_filename}'
    
    # and extract the route (list of integer ids) from each file
    with open (route_fullname) as r_file:
      doc = xmltodict.parse(r_file.read())
      # append the route to the list of routes for this instance
      routes.append([int(node['@id']) for node in doc['route']['node']])

  return routes

# iterates over a bunch of old instances and a bunch of old routes and tries to solve the FRVCP for each
# using the labeling algorithm
def run_checker():
  # specify instances and routes locations
  instances_folder = 'C:/Users/Nick/Desktop/dataFRVCP/instances'
  routes_folder = 'C:/Users/Nick/Desktop/dataFRVCP/routesFRVCP'

  instance_names = [os.fsdecode(instance) for instance in os.listdir(os.fsencode(instances_folder))]

  for instance in instance_names:
    
    # a name for our converted instance file
    temp_instance_name = 'temp_converted_instance.json'
    # in case we encounter this file, skip it
    if instance == temp_instance_name:
      continue
    instance_jsonname = f'{instances_folder}/{temp_instance_name}'
    
    print(f'Instance:\n{instance}')
    instance_fullname = f'{instances_folder}/{instance}'

    # convert the instance to one that can be read by this implementation of the labeling algorithm
    convertOldInstances.convert(instance_fullname,instance_jsonname)

    routes = get_routes(instance, routes_folder)
    for route in routes:
      print(f"Initial route:\n{route}")

      # solve and report output
      solver = Solver(instance_jsonname,route)

      # run solver, get output
      obj,route = solver.solve()
      print(f'Objective:\n{obj:.4}')
      print(f'Energy-feasible route:\n{route}')

if __name__ == "__main__":
    run_checker()