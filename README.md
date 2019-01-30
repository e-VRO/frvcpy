# Solution methods for the FRVCP
The fixed-route vehicle charging problem (FRVCP) is characterized by a vehicle that must visit an ordered sequence of locations (a fixed route). The vehicle is limited in its onboard energy, which gets depleted as it travels. As a result, it must restore its energy along the way. The typical objective of the FRVCP is to find the optimal "insertion" of energy restoration operations into the fixed route that minimize the route's duration. The problem was originally conceived in MONTOYA for the case of electric vehicles (EVs), which require nontrivial amounts of time to restore the energy in their batteries and must therefore carefully consider their charging operations.

A number of variants of the FRVCP exist. The one addressed here is the original FRVCP as described in MONTOYA. This variant uses non-linear charging functions, continuous charging decisions, zero wait times at CSs, no release dates for customers, etc.

While other solution methods may be implemented in the future, currently we offer one: the labeling algorithm from FROGER. This algorithm solves this variant of the FRVCP exactly. Notes on this implementation:
 - Charging stations' charging functions are non-convex and either linear or piecewise linear
 - Types of nodes in the problem are either customer or charging station. No other node types paid attention to (depot type exists, but is handled identically to a customer node)
 - If `n` is the number of nodes in the underlying graph (which we will refer to as `G`), then the energy and time matrices in the passed JSON instance must be of size `n`x`n`; similarly, the processing times must be `n`x`1`
 - Assumes there is no energy required to process a customer