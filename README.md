# The FRVCP
The __fixed-route vehicle charging problem__ (FRVCP) is characterized by a vehicle that must visit an ordered sequence of locations (a fixed route). The vehicle is limited in its onboard energy, which gets depleted as it travels. As a result, it must restore its energy along the way. The typical objective of the FRVCP is to find the optimal "insertion" of energy restoration operations into the fixed route that minimize the route's duration. The problem was given its acronym in Montoya et al. (2017) for the case of electric vehicles (EVs), which require nontrivial amounts of time to restore the energy in their batteries and must therefore carefully consider their charging operations.

We anticipate branching this repository when new variants of the FRVCP are addressed. As of Feb 14, 2019, there are two for which a solution method exists here:
  1. The original FRVCP as described in Montoya et al. (2017) for the Electric Vehicle Routing Problem with Non-linear Charging Functions (E-VRP-NLC)
  2. The FRVCP for the Ridehail Problem with Autonomous Electric Vehicles (RP-AEV), currently under development by Kullman et al. (2019)

While other solution methods may be implemented in the future, we currently offer only one: the labeling algorithm from Froger et al. (2018), which provides an exact solution to the FRVCP.