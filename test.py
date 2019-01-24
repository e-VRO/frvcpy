from frvcp_py.labelalgo.core import NodeType,Node,PCCMLabel

node = Node(1,"first node",NodeType.DEPOT)
node2 = Node(2,"second node",NodeType.CUSTOMER)

label1 = PCCMLabel(1,5,5,7,9,11,[[0,10],[0,100]],[10],14,15,None)
label2 = PCCMLabel(2,4,5,7,9,11,[[0,10],[0,100]],[10],14,15,label1)

print(label1 > label2)
print(label1)
print(label2)