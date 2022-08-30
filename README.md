
[![CI_Build](https://github.com/vidalt/PDTSP/actions/workflows/CI_Build.yml/badge.svg)](https://github.com/vidalt/PDTSP/actions/workflows/CI_Build.yml)

# Exponential-Size Neighborhoods for the Pickup-and-Delivery Traveling Salesman Problem

This is an implementation of the new neighborhood searches for one-to-one pickup-and-delivery problems. More precisely, we focus on the pickup-and-delivery traveling salesman problem (PDTSP), which is the canonical problem in this family [1].

Main project features:
* <b>Relocate-Pair</b> neighborhood: Enhanced implementation in O(n<sup>2</sup>) instead of O(n<sup>3</sup>) time.
* <b>2-Opt</b> neighborhood: Adaptation to the PDTSP.
* <b>Or-Opt</b> neighborhood: Adaptation to the PDTSP.
* <b>2k-Opt</b> neighborhood: New exponential-size neighborhood containing all solutions formed with nested 2-Opts and explored in O(n<sup>2</sup>) time.
* <b>4-Opt</b> neighborhood: Based on the concepts of Glover (1996) and adapted to the PDTSP.
* <b>Balas and Simonetti</b> neighborhood: Based on the concepts of Balas and Simonetti (2001) and adapted to the PDTSP.
* All these neighborhoods are integrated within the hybrid genetic search (HGS) of Vidal et al. (2012, 2014).
* We also provide a reimplementation of Veenstra et al. (2017) Ruin and Recreate algorithm, including an O(n<sup>2</sup>) reinsertion operator.

## References

When using this algorithm (or part of it) in derived academic studies, please refer to the following work:

[1] Pacheco, T., Martinelli, R., Subramanian, A., Toffolo, T. A., & Vidal, T. (2021). Exponential-Size Neighborhoods for the Pickup-and-Delivery Traveling Salesman Problem. Transportation Science, Forthcoming. ArXiv preprint arXiv:2107.05189.

## Scope

This code has been designed to solve the Pickup-and-Delivery Traveling Salesman Problem (PDTSP).

This code version has been designed and calibrated for medium-scale instances with up to 1,000 customers.

## Compiling the executables 

You need [`CMake`](https://cmake.org) to compile.

Build with:
```console
mkdir build
cd build
cmake ..
make -j4
```
This will generate the executable file `pdphgs` and `pdprr` in the `build` directory.

## Running the algorithm

After building the executables, you can try an example of `pdphgs`: 
```console
./pdphgs --instance=../instances/RBO00/Class1/U159C.PDT --it=1000
```

The following options are supported: (default values are included in parentheses)
```
./pdphgs --help
  --help                                Display a help message.
  --version                             Display the current version.
  --verbose                             Enable verbose mode.
  --instance                            Instance file path.
  --grubhub                             Read file as a distance matrix (GrubHub
                                        format).
  --seed arg (=0)                       Sets a random seed.
  --time-limit arg (=2147483647)        Set the maximum execution time in seconds.
  --bs-k arg (=3)                       Balas&Simonetti k parameter.
  --or-k arg (=30)                      Or-Opt k parameter.
  --ratio-slow-nb arg (=1)              Ratio of slow neigborhoods usage in local searches.
  --neighborhoods arg (=RELOCATE-2OPT-2KOPT-OROPT-4OPT-BS)
                                        Select neighborhood structure.
  --mu arg (=25)                        Minimum population size.
  --it arg (=1000000)                   Maximum iterations without improvement.
  --div arg (=4000)                     Iterations without improvement that trigger. diversification
  --lambda arg (=40)                    Number of offspring in each generation.
  --nb-elite arg (=1)                   Number of elite individuals.
  --nb-close arg (=2)                   Number of close individuals.
```

You can also run the Ruin and Recreate implementation as follows:
```console
./pdprr --instance=../instances/RBO00/Class1/U159C.PDT --fast
```

The following options are supported:
```
./pdprr --help
  --help                              Display a help message.
  --version                           Display the current version.
  --instance arg                      Instance file path.
  --fast                              Use the fast reinsertion operator.
  --seed arg (=0)                     Seed provided to srand.
  --p-accept arg (=3)                 p parameter to accept a request as worst.
  --c-rate arg (=0.99987571600000003) Cooling rate.
  --it arg (=50000)                   Maximum number of iterations.
  --time-limit arg                    Set maximum execution time in seconds.
```

## Code structure

### Hybrid Genetic Search (./PDP-HGS folder)
The main folders containing the logic of the algorithm are the following:
* **hgsadc**: Implementation of HGS with a generic problem and solution interface.
* **pdp**: Implementation of the local search and other operators required by the GA (crossover, repair and mutate).
* **pdp/moves**: Generic interface for moves (PDPMoves), and individual classes with the implementations of the six neighborhoods: Relocate-Pair, 2-Opt, Or-Opt, 2k-Opt, Balas and Simonetti and 4-Opt.

In addition, additional classes have been created to facilitate interfacing:
* **Application**: Reads command line and stores the parameters of the algorithm.
* **random**: Utilities to initiate RNG and allow ranged sampling.
* **main**: Main code to start the algorithm

### Ruin and Recreate (./PDP-RR folder)
The main classes containing the logic of the algorithm are the following:
* **Application**: Reads command line and stores the parameters of the algorithm.
* **Instance**: Reads instance file and stores precompute distances.
* **Operators**: Implementation of the Removal and Reinsertion (classical and fast) operators.
* **Solution**: Represents an individual solution.
* **Solver**: Implementation of the Ruin and Recreate algorithm.

### Instances and Solutions

* **instances/RBO00**: Folder with instances introduced in Renaud et al. (2000) - (.sol files containing the best-known solutions found).
* **instances/Dumitrescu**: Folder with instances introduced in Dumitrescu et al. (2010) - (.sol files containing the best-known solutions found).
* **instances/Grubhub**: Folder with instances introduced in O’Neil (2018).
* **instances/PDP-X**: Folder with instances introduced in Pacheco et al. (2022) - (with .sol files containing the best-known solutions found).

## Contributing

Thank you very much for your interest in this code. This code is actively maintained and evolving. Pull requests and contributions seeking to improve the code in terms of readability, usability, and performance are welcome. Development is conducted in the dev branch. I recommend contacting us beforehand at <tpacheco@inf.puc-rio.br> or <thibaut.vidal@polymtl.ca> before any major rework.

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright(c) 2022 Toni Pacheco

