# frvcpy: An Open-Source Solver for the FRVCP
This package offers a solver for the fixed route vehicle charging problem.

##### Table of Contents  
[The FRVCP](#frvcp)  
[The Solver](#solver)  
[Usage](#usage)  
[Instance Translation](#translation)  
[Solver Input](#input)  
[Additional Information](#moreinfo)  


<a name="frvcp"></a>
## The FRVCP
### Inserting charging stations into an EV's route
The __fixed route vehicle charging problem__ (FRVCP) is characterized by a vehicle that must visit an ordered sequence of locations (a fixed route). The vehicle is limited in its onboard energy, which gets depleted as it travels. As a result, it must restore its energy along the way. The typical objective of the FRVCP is to find the optimal "insertion" of energy restoration operations into the fixed route that minimize the route's duration. The problem was given its acronym in Montoya et al. (2017) for the case of electric vehicles (EVs), which require nontrivial amounts of time to restore the energy in their batteries and must therefore carefully consider their recharging operations.

<a name="solver"></a>
## The Solver  
To solve the FRVCP, frvcpy implements the labeling algorithm from Froger et al. (2019), providing an exact solution in low runtime. The algorithm incorporates realistic problem features such as nonlinear charging functions, heterogeneous charging station technologies, and multiple CS visits between stops. 

<a name="usage"></a>
## Usage
With a compatible instance file ([see the schema](https://github.com/e-VRO/frvcpy/blob/master/instances/frvcpy-instance.schema.json)), solve the FRVCP from a Python script: 
```python
from frvcpy.translator import translate
from frvcpy.solver import Solver

route = [0,3,2,1,0]      # route to make energy feasible
q_init = 750           # vehicle's initial energy level

# using an existing instance from file
frvcp_solver = Solver("instances/frvcpy-instance.json", route, q_init)

# run the algorithm
duration, feas_route = frvcp_solver.solve()

print(f"Duration: {duration:.4}")
# Duration: 123.4567

print(f"Energy-feasible route:\n{feas_route}")
# Energy-feasible route:
# [(0, None), (3, None), (2, None), (4, 300.0), (1, None), (0, None)]
```
Or from the command line:
```bash
frvcpy --instance=./instances/frvcpy-instance.json --route=0,3,2,1,0 --qinit=750
# Duration: 123.4567
# Energy-feasible route:
# [(0, None), (3, None), (2, None), (4, 300.0), (1, None), (0, None)]
```

<a name="translation"></a>
## Instance Translation
frvcpy includes a translator for some E-VRP instances formatted according to the [VRP-REP](http://www.vrp-rep.org/) [specification](http://www.vrp-rep.org/schemas/download/vrp-rep-instance-specification-0.5.0.xsd). 
If you have such an instance file, it can be translated with the Python API via 
```python
from frvcpy.translator import translate

# Option 1) make instance object to be passed directly to the solver
frvcp_instance = translate("instances/vrprep-instance.xml")

# Option 2) write the translated instance to file
frvcp_instance = translate("instances/vrprep-instance.xml", to_file="instances/my-instance.json")
```
Or via the command line via
```bash
frvcpy-translate instances/vrprep-instance.xml instances/my-instance.json
```
### Translation requirements for VRP-REP instances
The translator assumes VRP-REP instances are formatted similarly to the [Montoya et al. (2017) instances](http://vrp-rep.org/datasets/item/2016-0020.html): 
  - CSs are identified as `<node>` elements having attribute `type="2"`
  - Charging stations nodes have a `<custom>` child element which contains a `cs_type` element
  - For each unique CS type `t` appearing in those `cs_type` elements, there exists a charging `function` element with attribute `cs_type=t`
  - These `function` elements are part of a `charging_functions` element in a `vehicle_profile`'s `custom` element
  - The depot has node ID 0, the N customers have IDs {1, ..., N}, and the CSs have IDs {N+1, ..., N+C}

Here is a small example meeting these requirements:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<instance>
  <network>
    <nodes>
      <node id="0" type="0">
        <cx>74.83</cx>
        <cy>51.85</cy>
      </node>
      <node id="1" type="1">
        <cx>68.77</cx>
        <cy>75.69</cy>
      </node>
      <node id="11" type="2">
        <cx>57.0</cx>
        <cy>57.04</cy>
        <custom>
          <cs_type>fast</cs_type>
        </custom>
      </node>
    </nodes>
    <euclidean/>
    <decimals>14</decimals>
  </network>
  <fleet>
    <vehicle_profile type="0">
      <departure_node>0</departure_node>
      <arrival_node>0</arrival_node>
      <speed_factor>25.0</speed_factor>
      <custom>
        <consumption_rate>0.125</consumption_rate>
        <battery_capacity>16.0</battery_capacity>
        <charging_functions>
          <function cs_type="fast">
            <breakpoint>
              <battery_level>0.0</battery_level>
              <charging_time>0.0</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>13.6</battery_level>
              <charging_time>0.317</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>15.2</battery_level>
              <charging_time>0.383</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>16.0</battery_level>
              <charging_time>0.517</charging_time>
            </breakpoint>
          </function>
        </charging_functions>
      </custom>
    </vehicle_profile>
  </fleet>
  <requests>
    <request id="1" node="1">
      <service_time>0.5</service_time>
    </request>
  </requests>
</instance>
```

<a name="input"></a>
## Solver Input
Use of the solver requires
 1. An instance (either a JSON file or equivalent Python dictionary) containing the information described in [the schema](https://github.com/e-VRO/frvcpy/blob/master/instances/frvcpy-instance.schema.json).

    - We suggest following the convention where the first (zeroth) node is the depot, followed by customer nodes, followed by CS nodes.

    - If the depot also serves as a CS for the EV, then an additional CS node should be created for it (_note: this is handled automatically if using the translator_).
 
 2. EV's fixed route
    - Ordered list of depot/customer nodes the EV must visit
 
 3. EV's initial charge
    - A number in [0, `max_q`], as defined in the instance

<a name="moreinfo"></a>
## Additional information
For more information about the algorithm used in the solver, see [Froger et al. (2019)](https://www.sciencedirect.com/science/article/abs/pii/S0305054818303253).

For a write-up of the package: _Manuscript in preparation_
