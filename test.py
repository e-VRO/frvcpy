# TODO write instance file if it does not already exist
# instance_file = write_instance_file(self.stuff, instance_filename)

# once it exists...
instance_filename = "./instances/LAInstance2.json"
route= [0,1,2,0]

from frvcp_py.labelalgo.solver import Solver
solver = Solver(instance_filename,route)

obj,route = solver.solve()
print(f'Objective:{obj:.4}')
print(f'Route:\n{route}')
