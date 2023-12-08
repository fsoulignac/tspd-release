# Decremental State-Space Relaxations for the Basic Traveling Salesman Problem with a Drone (DSSRs for the TSP-D)

Repository with the source code to replicate the experiments from our article.

## Abstract

Truck-and-drone routing problems have become an important research topic in the last decade due to their applications for last-mile deliveries.   Despite the many publications in this area, the most efficient exact algorithms designed thus far struggle to solve the benchmark instances with 39 or more customers.  This fact holds even for one of the simplest variants involving one truck and one drone whose routes must synchronize at customers' locations: the Basic Traveling Salesman Problem with a Drone (TSP-D).  In this article, we devise a new algorithm for the TSP-D that solves every benchmark instance with up to 59 customers, and it scales up to 99 customers when the drone is much faster than the truck.  The core of our method is a dynamic programming algorithm that is applied for column generation and variable fixing within tailored decremental state-space relaxation strategies.

## Overview of the code

Compiling the code yields five programs (see _CMakeLists.txt_):
1. _tspd_pfc_: main solver for the TSP-D (_main_pfc.cpp_),
1. _tspd_bp_:  a simple branch-and-price solver for the TSP-D (_main_bp.cpp_),
1. _tspd_lbl_: solver for a single princing problem (_main_lbl.cpp_)
1. _tspd_vf_: experiment for testing variable fixing (_main_vf.cpp_)
1. _poi_to_json_: transforms txt files by Poikonen et al. to the required json format (_poi_to_json.cpp_).

### tspd_pfc

This is the main solver for the TSP-D based on decremental state-space relaxations strategies (DSSR).  The algorithm includes:
- A new dynamic programming algorithm for the TSP-D with the following features:
    - A novel NG-route relaxation that is stronger to the one previously proposed
    - New dominance rules based on partial dominance
    - A bidirectional search procedure that can be adapted into a forward or backward search
    - A method to compute completions bounds based on a backward execution of the DP algorithm
- A new variable fixing method based on completion bounds
- A tailored column generation algorithm that works by repeatedly guessing upper bounds to fix variables
- A DSSR method that combines column generation and iterative variable fixing to solve the TSP-D

The algorithm has different options to solve the following variants, as proposed by Roberti and Ruthmair:
- LOOP: TSP-D allowing loops.
- INCOMP: TSP-D with incompatible customers.
- RANGE: TSP-D with a drone's flying range.
- NOLW: RANGE, but the drone cannot land and wait for the truck when it reaches the rendezvous point first.
- LRT: TSP-D with launch and rendezvous times.
- TR0: TSP-D, but the truck cannot visit customers alone.
- MHD: LRT with 20% of incompatible customers in which the drone has a flying range of 20% and it cannot land and wait for the truck.  This variant emulates the one studied by Murray and Chu (2015), Ha et al. (2018), and Dell'Amico et al (2022).
- POI: LOOP in which the drone has a flying range of 20% and it cannot land and wait for the truck.  This variant was studied by Poikonen et al. (2019).

### tspd_bp

A simple branch-and-price (BP) solver built on top of TSPD_PFC with the following features:
- Branching rules proposed by Roberti and Ruthmair
- Two column generation algorithm for solving the linear relaxation of the master problem (a generic one and the new algorithm used in the DSSR).

This is a rather basic BP algorithm that we use as a baseline to evaluate TSPD_PFC.  As such, it lacks some features such as a strong branching strategy, variable fixing methods, etc.

### tspd_lbl

A solver for a pricing problem that makes it simpler to evaluate the DP method without running a master solver.

### tspd_vf

Implementation of the variable fixing experiment described in the paper

### poi_to_json

Our solver work on json files.  This program transforms the instances by Poikonen et al. to our format.

## Getting started
The following instructions will guide you through the steps to execute the experiments from the article.

### Prerequisites
- Python >= 3.6 [(more info)](https://www.python.org/)
- CPLEX >= 12.8 [(more info)](https://www.ibm.com/products/ilog-cplex-optimization-studio)
- Boost Graph Library >=1.66 [(more info)](https://www.boost.org/doc/libs/1_66_0/libs/graph/doc/index.html)
    - On Linux: ```sudo apt-get install libboost-all-dev```
- CMake >= 3.22.1 [(more info)](https://cmake.org/)
    - On Linux: ```sudo apt-get install cmake```
- C++17 or higher [(more info)](https://es.wikipedia.org/wiki/C%2B%2B17)
- G++ fully implementing C++17.

### Built with modified versions of
- Kaleidoscope: A tool to visualize the outputs of Optimization Problems [(more info)](https://github.com/gleraromero/kaleidoscope)
    - Parser files for this project are included in the kaleidoscope folder
- Runner: A script to ease the process of running experiments [(more info)](https://github.com/gleraromero/runner)
- GOC lib: A library that includes interfaces for using (Mixed Integer) Linear Programming solvers, and some useful resources [(more info)](https://github.com/gleraromero/goc).

### Running the experiments.
1. Add environment variables with the paths to the libraries.
    1. Add two environment variables to bash with CPLEX include and library paths.
        1. ```export CPLEX_INCLUDE=<path_to_cplex_include_dir>```
            - Usually on Linux: _/opt/ibm/ILOG/CPLEX_Studio\<VERSION\>/cplex/include_
        1. ```export CPLEX_BIN=<path_to_cplex_lib_binary_file>```
            - Usually on Linux: _/opt/ibm/ILOG/CPLEX_Studio\<VERSION\>/cplex/lib/x86-64_linux/static_pic/libcplex.a_
    1. Add two environment variables to bash with BOOST Graph Library include and library paths.
        1. ```export BOOST_INCLUDE=<path_to_boost_include_dir>```
            - Usually on Linux: _/usr/include_
        1. ```export BOOST_BIN=<path_to_boost_lib_binary_file>```
            - Usually on Linux: _/usr/lib/x86_64-linux-gnu/libboost_graph.a_
1. Go to the labeling_example root directory.
1. Execute ```python3 runner/runner.py <experiment_file>``` (run ```python3 runner/runner.py``` to check some options)
1. The execution output will be continually saved to the output folder.

### Experiments
There are different experiment json files that are self-documented.  Each experiment executes a single program (see Section [Overview](#Overview)); the main file of each program describes the parameters of the experiments.


### Visualizing the experiment results
1. Go to the folder kaleidoscope and open the file index.html with a Web Browser (Chrome prefered, Firefox working).
1. Add the output file.
1. Select the experiments.
1. Add some attributes to visualize.
1. Click on Refresh.
1. If more details on an experiment are desired click on the + icon in a specific row.

> Option ```log_level``` in the experiment files control the amount of information output (0: minimal, 1: information for the main problem only, 2: log everything).  Default is ```log_level = 0``` Expect a huge output when ```log_level = 2```.

### Instances
- We include the original dataset of raw instances proposed by Poikonen et al. for up to 99 customers (folder _poi-instances_)
- Use _poi_to_json_ to transform the raw instances into JSON instances that our algorithms consume (source file _poi_to_json.cpp_)
- We provide precompiled JSON files for the raw instances (foldef _instances_)

## Built With
* [JSON for Modern C++](https://github.com/nlohmann/json)
* [Boost Graph Library](https://www.boost.org/doc/libs/1_66_0/libs/graph/doc/index.html)

## Authors
Optimization and logistics group
- Marcos Blufstein (Universidad de Buenos Aires)
- Gonzalo Lera-Romero (Universidad de Buenos Aires)
- Francisco Juan Soulignac (ICC-CONICET and Universidad de Buenos Aires)

## License
This project is licensed under the MIT License - see the LICENSE file for details
