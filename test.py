import argparse
import csv
import datetime
import os

import xmltodict

from frvcpy.solver import Solver
from frvcpy.translator import translate

def _get_routes(instance_name, routes_folder):
    """Get list of routes (and their names) to solve for the given instance"""
    routes = []
    route_names = []
    
    # get the names of the route files corresponding to the given instance
    route_filenames = [
        os.fsdecode(route_filename) for route_filename in os.listdir(os.fsencode(routes_folder))
        if os.fsdecode(route_filename).startswith(f'route_{instance_name[:-4]}_')
    ]

    # get the routes from those files
    for route_filename in route_filenames:
        route_fullname = f'{routes_folder}/{route_filename}'
        
        # and extract the route (list of integer ids) from each file
        with open (route_fullname) as r_file:
            doc = xmltodict.parse(r_file.read())
            # append the route to the list of routes for this instance
            routes.append([int(node['@id']) for node in doc['route']['node']])
            route_names.append(route_filename[:-4])

    return routes,route_names   

def test_evrpnl_instances(instances_folder, routes_folder, output_file=None):

    instance_names = [os.fsdecode(instance) for instance in os.listdir(os.fsencode(instances_folder))]

    total_run_time = 0
    num_runs = 0
    total_translate_time = 0
    num_translations = 0
    records = []

    for i,instance in enumerate(instance_names):
        
        print(f'Instance {i+1}/{len(instance_names)}: {instance}')
        instance_fullname = f'{instances_folder}/{instance}'

        routes,route_names = _get_routes(instance, routes_folder)
        if len(routes) == 0:
            continue

        # translate the E-VRP-NL instances
        t0 = datetime.datetime.now()
        inst_dict = translate(instance_fullname)
        t1 = datetime.datetime.now()
        total_translate_time += (t1-t0).total_seconds()
        num_translations += 1

        for i_r, (route,route_name) in enumerate(zip(routes,route_names)):

            print(f'Route {i_r+1}/{len(routes)} ({route_name}): {route}')

            solver = Solver(inst_dict,route,inst_dict['max_q'])

            # run solver
            t0 = datetime.datetime.now()
            obj,f_route = solver.solve()
            t1 = datetime.datetime.now()
            elapsed = (t1-t0).total_seconds()
            
            total_run_time += elapsed
            num_runs += 1
            
            if output_file is not None:
                records.append({
                    'instance':instance,
                    'route':route,
                    'route_name':route_name,
                    'route_len':len(route),
                    'run_time(s)':elapsed,
                    'css':len(inst_dict['css'])+1
                })
    
    
    print(f'Average algorithm run time (s):   {total_run_time/num_runs:.4}.')
    print(f'Average time to translate instances (s): {total_translate_time/num_translations}')
    if output_file is not None:
        keys = records[0].keys()
        with open(output_file, 'w', newline='') as f:
            dict_writer = csv.DictWriter(f, keys)
            dict_writer.writeheader()
            dict_writer.writerows(records)
        print(f'Results written to file {output_file}')
    print("Done")

def main():
    
    parser = argparse.ArgumentParser(
        description="""Runs the FRVCP over all instances in the Froger et al. (2019) testbed.
        
        Download the testbed here: 
        https://www.math.u-bordeaux.fr/~afroger001/documents/data-Improved_formulations_and_algorithmic_components.zip
        
        By default, instances are assumed to be located at './instances/evrpnl/instances'
        And routes at './instances/evrpnl/routes'
        """)
    parser.add_argument(
        '-i',
        '--instancesdir',
        type=str,
        default='./instances/evrpnl/instances',
        help='Folder where E-VRP-NL instances are located (contents of instancesEVRPNL.zip)')
    parser.add_argument(
        '-r',
        '--routesdir',
        type=str,
        default='./instances/evrpnl/routes',
        help='Folder where E-VRP-NL routes are located (contents of routesFRVCP.zip)')
    parser.add_argument(
        '-o',
        '--output',
        type=str,
        default=None,
        help='File to which to write runtime results. Default is not to write output.')

    args = parser.parse_args()

    test_evrpnl_instances(args.instancesdir, args.routesdir, args.output)

    sys.exit(0)


if __name__ == "__main__":
    main()