/*
 * config.h
 * Author: Prabal Basu
 * Email: prabalb@aggiemail.usu.edu
 */

#include <iostream>

using namespace std;

// Define the microarchitectural process energies (from DSENT 0.91, 32nm)
#define PWR_INCOMING 1.926675e-11   // write buffer energy
#define PWR_SWALLOC  1.306024e-11   // switch allocation energy
#define PWR_XBAR     2.957041e-11   // crossbar traversal energy
#define PWR_FORWARD  1.845249e-12   // read buffer energy
#define PWR_STANDBY  4.381635e-12/5 // leakage energy

// microarchitectural process IDs
#define PROCESS_INCOMING  0
#define PROCESS_SWALLOC   1
#define PROCESS_XBAR      2
#define PROCESS_FORWARD   3
#define PROCESS_STANDBY   4

#define DIRECTIONS      5
#define DIRECTION_NORTH 0
#define DIRECTION_EAST  1
#define DIRECTION_SOUTH 2
#define DIRECTION_WEST  3
#define DIRECTION_LOCAL 4

// NoC size
#define NO_OF_TILES_X 8
#define NO_OF_TILES_Y 8
#define NO_OF_TILES NO_OF_TILES_X*NO_OF_TILES_Y

// NoC traffic and routing
#define TRAFFIC "synfull"
#define ROUTING "dbar"
#define PIR = 0.25

// Volatge, frequency
#define VDD 0.7
#define FREQUENCY 0.769E9
#define WARM_UP_TIME 1000

// Floorplan and Grid
#define GRID_SIZE 1600
#define GRID_FILE "./psn/grid_1600_mesh.txt"

#define NO_VOLTAGE_PINS 81
#define NO_RLC_COMB 3120

#define FLOORPLAN_FILE "./psn/floor_plan_1600_8x_8y.txt"
