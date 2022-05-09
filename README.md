
[![CI_Build](https://github.com/vidalt/HGS-CVRP/actions/workflows/CI_Build.yml/badge.svg)](https://github.com/vidalt/HGS-CVRP/actions/workflows/CI_Build.yml)

# HGS-CVRP: A modern implementation of the Hybrid Genetic Search for the CVRP

This is a modern implementation of the Hybrid Genetic Search (HGS) with Advanced Diversity Control of [1], specialized to the Capacitated Vehicle Routing Problem (CVRP).

The C++ code of this algorithm has been designed to be transparent, specialized, and extremely concise, retaining only the core elements that make this method a success.
Beyond a simple reimplementation of the original algorithm, this code also includes speed-up strategies and methodological improvements learned over the past decade of research and dedicated to the CVRP.
In particular, it relies on an additional neighborhood called SWAP* which consists in exchanging two customers between different routes without an insertion in place.

## References

When using this algorithm (or part of it) in derived academic studies, please refer to the following works:

[1] Vidal, T., Crainic, T. G., Gendreau, M., Lahrichi, N., Rei, W. (2012). 
A hybrid genetic algorithm for multidepot and periodic vehicle routing problems. Operations Research, 60(3), 611-624. 
https://doi.org/10.1287/opre.1120.1048 (Available [HERE](https://w1.cirrelt.ca/~vidalt/papers/HGS-CIRRELT-2011.pdf) in technical report form).

[2] Vidal, T. (2022). Hybrid genetic search for the CVRP: Open-source implementation and SWAP* neighborhood. Computers & Operations Research, 140, 105643.
https://doi.org/10.1016/j.cor.2021.105643 (Available [HERE](https://arxiv.org/abs/2012.10384) in technical report form).

We also recommend to refer to the Github version of the code used, as future versions may achieve different (better) performance as the code evolves.
The version associated with the results presented in [2] is [v1.0.0](https://github.com/vidalt/HGS-CVRP/releases/tag/v1.0.0).

## Scope

This code has been designed to solve the "canonical" Capacitated Vehicle Routing Problem (CVRP).
It can also directly handle asymmetric distances as well as duration constraints.

This version of the code has been designed and calibrated for medium-scale instances with up to 1,000 customers. 
It is **not** designed in its current form to run very-large scale instances (e.g., with over 5,000 customers), as this requires additional solution strategies (e.g., decompositions and additional neighborhood limitations).
If you need to solve problems outside of this algorithm's scope, do not hesitate to contact me at <thibaut.vidal@polymtl.ca>.

## Compiling the executable 

You need [`CMake`](https://cmake.org) to compile.

Build with:
```console
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make bin
```
This will generate the executable file `hgs` in the `build` directory.

Test with:
```console
ctest -R bin --verbose
```

## Running the algorithm

After building the executable, try an example: 
```console
./hgs ../Instances/CVRP/X-n157-k13.vrp mySolution.sol -seed 1 -t 30
```

The following options are supported:
```
Call with: ./hgs instancePath solPath [-it nbIter] [-t myCPUtime] [-bks bksPath] [-seed mySeed] [-veh nbVehicles] [-log verbose]
[-it <int>] sets a maximum number of iterations without improvement. Defaults to 20,000                                     
[-t <double>] sets a time limit in seconds. If this parameter is set the code will be run iteratively until the time limit           
[-seed <int>] sets a fixed seed. Defaults to 0                                                                                    
[-veh <int>] sets a prescribed fleet size. Otherwise a reasonable UB on the the fleet size is calculated                      
[-round <bool>] rounding the distance to the nearest integer or not. It can be 0 (not rounding) or 1 (rounding). Defaults to 1. 
[-log <bool>] sets the verbose level of the algorithm log. It can be 0 or 1. Defaults to 1.                                       

Additional Arguments:
[-nbGranular <int>] Granular search parameter, limits the number of moves in the RI local search. Defaults to 20               
[-mu <int>] Minimum population size. Defaults to 25                                                                            
[-lambda <int>] Number of solutions created before reaching the maximum population size (i.e., generation size). Defaults to 40
[-nbElite <int>] Number of elite individuals. Defaults to 5                                                                    
[-nbClose <int>] Number of closest solutions/individuals considered when calculating diversity contribution. Defaults to 4     
[-targetFeasible <double>] target ratio of feasible individuals in the last 100 generatied individuals. Defaults to 0.2        
```

There exist different conventions regarding distance calculations in the academic literature.
The default code behavior is to apply integer rounding, as it should be done on the X instances of Uchoa et al. (2017).
To change this behavior, for example, when testing on the CMT or Golden instances, give a flag `-round 0`, when you run the executable.

## Code structure

The main classes containing the logic of the algorithm are the following:
* **Params**: Stores the main data structures for the method
* **Individual**: Represents an individual solution in the genetic algorithm, also provide I/O functions to read and write individual solutions in CVRPLib format.
* **Population**: Stores the solutions of the genetic algorithm into two different groups according to their feasibility. Also includes the functions in charge of diversity management.
* **Genetic**: Contains the main procedures of the genetic algorithm as well as the crossover
* **LocalSearch**: Includes the local search functions, including the SWAP* neighborhood
* **Split**: Algorithms designed to decode solutions represented as giant tours into complete CVRP solutions
* **CircleSector**: Small code used to represent and manage arc sectors (to efficiently restrict the SWAP* neighborhood)

In addition, additional classes have been created to facilitate interfacing:
* **AlgorithmParameters**: Stores the parameters of the algorithm
* **CVRPLIB** Contains the instance data and functions designed to read input data as text files according to the CVRPLIB conventions
* **commandline**: Reads the line of command
* **main**: Main code to start the algorithm
* **C_Interface**: Provides a C interface for the method

## Compiling the shared library

You can also build a shared library so that you can call the HGS-CVRP algorithm from your code.

```console
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make lib
```
This will generate the library file, `libhgscvrp.so` (Linux), `libhgscvrp.dylib` (macOS), or `hgscvrp.dll` (Windows),
in the `build` directory.

To test calling the shared library from a C code:
```console
make lib_test_c
ctest -R lib --verbose
```

## Other programming languages

There exists wrappers for this code in the following languages:
* **C**: The **C_Interface** file contains a simple C API
* **Python**: The [PyHygese](https://github.com/chkwon/PyHygese) package is maintained to interact with the latest release of this algorithm
* **Julia**: The [Hygese.jl](https://github.com/chkwon/Hygese.jl) package is maintained to interact with the latest release of this algorithm

We strongy encourage you to consider using these wrappers in your different projects.
Please contact me if you wish to list other wrappers and interfaces in this section.

## Contributing

Thank you very much for your interest in this code.
This code is still actively maintained and evolving, and we definitely welcome pull requests and contributions seeking to improve the code in terms of readability, usability and performance. Development is conducted in the `dev` branch. I recommend to contact me beforehand at <thibaut.vidal@polymtl.ca> before any major rework.

As a general guideline, the goal of this code is to stay **simple**, **stand-alone** and **specialized** to the CVRP. 
Therefore, contributions that aim to extend this approach to different variants of the vehicle routing problem should usually remain in a separate repository.
Similarly, contributions that require additional libraries or increase the conceptual complexity of the method are not generally recommended.
Indeed, when developing (meta-)heuristics, it seems always possible to do a bit better at the cost of extra conceptual complexity.
The overarching goal of this code is to find a good trade-off between algorithm simplicity and performance. This code can still evolve when the magnitude of the improvements are significant in relation to the extra complexity required.

There are two main types of contributions:
* Changes that do not impact the sequence of solutions found by the HGS algorithm when running `ctest` and testing other instances with a fixed seed.
This is visible by comparing the average solution value in the population and diversity through a test run. Such contributions include refactoring, simplification, and code optimization. Pull requests of this type are likely to be integrated more quickly.
* Changes that impact the sequence of solutions found by the algorithm.
In this case, I recommend to contact me beforehand with (i) a detailed description of the changes, (ii) detailed results on 10 runs of the algorithm for each of the 100 instances of Uchoa et al. (2017) before and after the changes, using the same termination criterion as used in [2](https://arxiv.org/abs/2012.10384).

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright(c) 2020 Thibaut Vidal




