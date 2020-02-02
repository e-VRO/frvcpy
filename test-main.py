import argparse
import csv
from datetime import datetime
import math
import random
import time

from frvcpy.solver import Solver
from frvcpy.core import FrvcpInstance


def main():
    
    parser = argparse.ArgumentParser(description="Run tests on randomly-generated instances")
    parser.add_argument(
        '-s',
        '--seed',
        type=int,
        help='Seed to be used in random instance generation',
        default=321)
    parser.add_argument(
        '-n',
        '--numcusts',
        type=int,
        help='Number of customers in the instance',
        default=100)
    parser.add_argument(
        '-c',
        '--numcss',
        type=int,
        help='Number of charging stations in the instance',
        default=50)
    parser.add_argument(
        '-t',
        '--chargetype',
        type=int,
        help='Type of charging function to use. 0 for a simple linear charging rate; 1 for a more realistic piecewise linear approximation',
        default=1)
    parser.add_argument(
        '-o',
        '--output',
        type=str,
        help='File to which to write results',
        default=None)

    args = parser.parse_args()
    if args.output is None:
        args.output = f"frvcpy-testresults-{datetime.now().strftime('%Y%m%d-%H%M%S')}.csv"
    records = []

    def dist(i,j):
        return math.sqrt((i[0] - j[0])**2 + (i[1] - j[1])**2)

    num_instances = 1
    num_routes = 15
    rng = random.Random()
    rng.seed(args.seed)
    
    speed = 40 # km/hr
    discharge_rate = 125 # Wh/km
    maxq = 16000 # Wh
    qdist = maxq/discharge_rate # km

    for inst in range(num_instances):
        
        # build instance
        instance = {}
        # specify fixed instance info
        instance['max_q'] = 16000

        # specify charging functions
        if args.chargetype > 0: # piecewise linear
            instance["breakpoints_by_type"] = [
                {
                    "cs_type": 0,
                    "time": [
                        0.0,
                        0.31,
                        0.39,
                        0.51
                    ],
                    "charge": [
                        0.0,
                        13600.0,
                        15200.0,
                        16000.0
                    ]
                },
                {
                    "cs_type": 1,
                    "time": [
                        0.0,
                        0.62,
                        0.77,
                        1.01
                    ],
                    "charge": [
                        0.0,
                        13600.0,
                        15200.0,
                        16000.0
                    ]
                },
                {
                    "cs_type": 2,
                    "time": [
                        0.0,
                        1.26,
                        1.54,
                        2.04
                    ],
                    "charge": [
                        0.0,
                        13600.0,
                        15200.0,
                        16000.0
                    ]
                }
            ]
        else: # linear
            instance["breakpoints_by_type"] = [
                {
                    "cs_type": 0,
                    "time": [
                        0.0,
                        0.51
                    ],
                    "charge": [
                        0.0,
                        16000.0
                    ]
                },
                {
                    "cs_type": 1,
                    "time": [
                        0.0,
                        1.01
                    ],
                    "charge": [
                        0.0,
                        16000.0
                    ]
                },
                {
                    "cs_type": 2,
                    "time": [
                        0.0,
                        2.04
                    ],
                    "charge": [
                        0.0,
                        16000.0
                    ]
                }
            ]

        # node locations
        depot = [(qdist/2,qdist/2)]
        customers = [(rng.random()*qdist,rng.random()*qdist) for _ in range(args.numcusts)]
        css = [(rng.random()*qdist,rng.random()*qdist) for _ in range(args.numcss)]

        # make css object for instance
        instance['css'] = [{
            "node_id":i+args.numcusts+1,
            "cs_type":rng.randint(0,2)} for i,_ in enumerate(css)]

        # make energy and time matrices
        nodes = depot + customers + css
        instance['energy_matrix'] = [
            [dist(i,j)*discharge_rate for j in nodes] for i in nodes
        ]
        instance['time_matrix'] = [
            [dist(i,j)/speed for j in nodes] for i in nodes
        ]
        cust_nodes = [n_id+1 for n_id in range(args.numcusts)]

        for r in range(num_routes):
            random.shuffle(cust_nodes) # shuffles customer nodes in place
            route = [0] + cust_nodes + [0]
            t0 = time.process_time()
            Solver(instance, route, maxq).solve()
            t1 = time.process_time()
            records.append({
                'custs':args.numcusts,
                'css':args.numcss,
                'chargetype':args.chargetype,
                'exectime':t1-t0
            })
    
    keys = records[0].keys()
    with open(args.output, 'w') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(records)
    print(f"Results written to file: {args.output}")


if __name__ == "__main__":
    # customer tests: -n [10,50,100,500,1000,5000]
    # cs tests: -c [5,10,50,100,500]
    # charge type tests: -l [0,1]
    main()