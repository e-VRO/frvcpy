# specify instance and route
instance_filename = "./instances/LAInstance-c30s4l4dT5.json"
import random
cust_seq = list(range(1,30))
random.shuffle(cust_seq)
route=[0]+cust_seq+[0]
print(f"Initial route:\n{route}")

# intialize solver
from frvcp_py.labelalgo.solver import Solver
solver = Solver(instance_filename,route)

# run solver, get output
obj,route = solver.solve()
print(f'Objective:\n{obj:.4}')
print(f'Energy-feasible route:\n{route}')
