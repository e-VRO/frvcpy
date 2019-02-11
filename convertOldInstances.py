import xmltodict
import json
import math
import sys

CS_INT = {
  'fast':0,
  'moderate':1,
  'normal':1, # not usually a qualifier, but it showed up in some old instances
  'slow':2
}

# define energy and time functions
def dist(i_node,j_node) -> float:
  return math.sqrt((float(i_node['cx']) - float(j_node['cx']))**2 + (float(i_node['cy']) - float(j_node['cy']))**2)

def t(i_node, j_node, speed) -> float:
  return dist(i_node,j_node)/speed

def e(i_node,j_node,consump_rate) -> float:
  return dist(i_node,j_node)*consump_rate

# primary method here that converts an old (xml) instance file into a new (json) one
def convert(full_filename_old, full_filename_new=None):

  #filename_old = "tc0c10s3cf1"
  if full_filename_new is None:
    print("No name specified for new instance. Saving as ./temp_instance.json")
    full_filename_new  = "./temp_instance.json"
  
  with open(full_filename_old) as f_xml:
    doc = xmltodict.parse(f_xml.read())

  instance_xml = doc["instance"]
  network = instance_xml["network"]
  nodes = network["nodes"]["node"]
  ev = instance_xml["fleet"]["vehicle_profile"]

  # append depot's CS to nodes
  nodes.append({'@id':str(len(nodes)),'cx':nodes[0]['cx'],'cy':nodes[0]['cy'],'@type':'2','custom':{'cs_type':'fast'}})

  # CSs
  css = [node for node in nodes if node['@type'] == '2']

  # vehicle info
  speed = float(ev['speed_factor'])
  max_t = float(ev['max_travel_time'])
  consump_rate = float(ev['custom']['consumption_rate'])
  max_q = float(ev['custom']['battery_capacity'])
  cfs = ev['custom']['charging_functions']['function']

  # instantiate output
  instance = {}

  # populate output
  # max charge and starting charge
  instance["max_q"] = max_q
  instance["init_soc"] = max_q
  # max duration
  instance["t_max"] = max_t # some big upper bound since none given
  # store CSs
  instance["css"] = [{'node_id':int(cs['@id']), 'type':CS_INT[cs['custom']['cs_type']]} for cs in css]
  # process times are zero
  instance["process_times"] = [0 for _ in nodes]
  # breakpoints
  if isinstance(cfs,(list,)): # if we have a list of charging functions
    instance["breakpoints_by_type"] = {
      str(CS_INT[cfs[k]['@cs_type']]):{
        "time":[float(bpt['charging_time']) for bpt in cfs[k]['breakpoint']],
        "charge":[float(bpt['battery_level']) for bpt in cfs[k]['breakpoint']]
      } for k in range(len(cfs))
    }
  else: # if we just have one
    instance["breakpoints_by_type"] = {
      str(CS_INT[cfs['@cs_type']]):{
        "time":[float(bpt['charging_time']) for bpt in cfs['breakpoint']],
        "charge":[float(bpt['battery_level']) for bpt in cfs['breakpoint']]
      }
    }
  # energy and time matrices
  instance["energy_matrix"] = [[e(i,j,consump_rate) for j in nodes] for i in nodes]
  instance["time_matrix"] = [[t(i,j,speed) for j in nodes] for i in nodes]

  # write to file
  with open(full_filename_new, 'w') as fp:
      json.dump(instance, fp)

if __name__ == "__main__":
    convert(sys.argv[1],sys.argv[2])