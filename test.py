import argparse
from datetime import datetime,timedelta
import random

from frvcpy.solver import Solver
from frvcpy.core import FrvcpInstance


def main():
    
    parser = argparse.ArgumentParser(description="Time tests for solutions to the FRVCP for random customer permutations and initial charges for the given instance")
    parser.add_argument(
        '-i',
        '--instance',
        type=str,
        help='Filename for the frvcpy-compatible problem instance on which to run tests')

    args = parser.parse_args()

    num_tests = 100

    # specify instance and route
    instance_filename = args.instance or "./instances/frvcpy-instance.json"
    instance = FrvcpInstance(instance_filename)

    # assumes depot is 0, CSs are listed in instance, and all the rest are customers
    cust_nodes = [n_id for n_id in list(range(instance.n_nodes_g)) if n_id not in [e["node_id"] for e in instance.cs_details] if n_id > 0]
    total_setup_time = timedelta()
    total_run_time = timedelta()

    for _ in range(num_tests):
        init_soc = random.random()*instance.max_q
        random.shuffle(cust_nodes) # shuffles customer nodes in place
        route = [0] + cust_nodes + [0]

        pre_setup = datetime.now()
        solver = Solver(instance_filename, route, init_soc)
        post_setup = datetime.now()
        duration, feas_route = solver.solve()
        post_solve = datetime.now()
        
        total_setup_time += post_setup-pre_setup
        total_run_time += post_solve-post_setup

    # TODO can we do asymmetric travel times?

    print(total_run_time)
    print(f'{num_tests} routes solved for instance {instance_filename}.')
    print(f'Average algorithm setup time (ms): {total_setup_time.microseconds/num_tests/1000:.4}.')
    print(f'Average algorithm run time (ms):   {total_run_time.microseconds/num_tests/1000:.4}.')

if __name__ == "__main__":
    main()