# TODO write instance file if it does not already exist
# instance_file = write_instance_file(self.stuff, instance_filename)

# once it exists...
instance_filename = "./la_instances/instance1.json"
route= [0,1,2,3]

from frvcp_py.labelalgo.solver import Solver
solver = Solver(instance_filename,route)

e_feas_route = solver.solve()

# node = Node(1,"first node",NodeType.DEPOT)
# node2 = Node(2,"second node",NodeType.CUSTOMER)

# label1 = PCCMLabel(1,5,5,7,9,11,[[0,10],[0,100]],[10],14,15,None)
# label2 = PCCMLabel(2,4,5,7,9,11,[[0,10],[0,100]],[10],14,15,label1)

# print(label1 > label2)
# print(label1)
# print(label2)