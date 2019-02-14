# specify instance and route
instance_filename = "./instances/LAInstance-c40s5l4dT5.json"
init_soc = 500
# generate random fixed route over the customer set
import random
cust_seq = list(range(1,40))
random.shuffle(cust_seq)
route=[0]+cust_seq+[0]
print(f"Initial route:\n{route}")

# intialize solver
from frvcp_py.labelalgo.solver import Solver
solver = Solver(instance_filename,route, init_soc)

# run solver, get output
obj,route = solver.solve()
print(f'Objective:\n{obj:.4}')
print(f'Energy-feasible route:\n{route}')
