# explainable-mapf
Code for MAPF with explainable plans

Usage: the code requires python3, and the networkx package.
In addition, please download the relevant test files from the link below.

Run using:

python ExplainablePlanning <.map filename> <.scen filename> <number of agents> <number of segments> <max timeout>

The .map and .scen files can be taken from:

https://movingai.com/benchmarks/mapf/index.html

To run all the tests, use runtests.sh. 

The required .map and .scen files are listed in tests.txt, and should be put in the same folder as the code and script files.

#CBS-generated Plans 
The file CBS_Plans.7z contains pairs of files of the form (scenario.yaml,scenario.schd).
The .yaml file represents the instance, and then .schd file represents the plan found by CBS.
Both files are YAML files.
