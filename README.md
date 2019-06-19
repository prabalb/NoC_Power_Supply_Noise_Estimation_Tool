# NoC Power Supply Noise Estimation Tool

This is a NoC power supply noise estimation tool integrated with the Booksim 2.0
NoC architectural simulator. The tool can dump average as well as cycle-wise
noise profile for an 8X8 mesh NoC architecture only. The tool also has the
implementation of the proactive noise mitigation techniques in the following paper.

IcoNoClast: Tackling Voltage Noise in the NoC Power Supply Through Flow-Control and Routing Algorithms

At each cycle, router activities are populated and stored in memory. Cycle wise loads
are generated for all the routers based on the activity. The load information is stored 
in memory. Then voltages of all the nodes are dumped based on the calculated load in an 
output file (e.g. psn_cycle_output.txt).

Source files: psn/config.h, psn/psn.hpp, psn/psn.cpp
To compile: cd booksim_psn; make;
To run: ./booksim <config_file> (e.g. ./booksim uniform)

NOTE: 
The current version of the source code computes cycle wise voltage statistics based on the following:
1. Technology: 32-nm
2. Vdd = 0.7 volt
3. NoC topology: 8x8 mesh
4. No. of power grid nodes: 1600
5. A Gaussian random distribution is assumed for the link switching activities. 

The tool expects a grid file (e.g., grid_1600_mesh.txt), a floorplan file (e.g., floor_plan_1600_8x_8y.txt) and 
a link load file (e.g., rndlds25.txt) to be present in the working directory.

If you use this tool, please cite the following publication in your work:

Basu, P., Shridevi, R.J., Chakraborty, K., Roy, S.: Iconoclast: Tackling 
voltage noise in the noc power supply through flow-control and routing algorithms. 
IEEETrans. VLSI Syst.25(7), 2035–2044 (2017)
