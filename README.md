# Solar-powered-UAV-Flight-Simulator

This simulator simulates SUAV point-to-point flight based on defined starting conditions and mission parameters, accounting for wind and cloud cover.
A full description of the applied flight model can be found in the 'Solar-powered Unmanned Aerial 
Vehicle (SUAV) PROTOTYPE Flight Model' document, which is a segment of the final year project report
associated with this simulator. The purpose of each .py file is described below:

model\SolarUAV.py - Contains the class used to define the SUAV parameters, mission starting
conditions, and mission parameters. It initially applies parameters of a generic SUAV.

data\SolarModel.py - Defines the solar irradiance model applied into the flight simulation.

data\AeroCoeff.py - Extracts airfoil data and implements it into the flight simulation based on the angle of attack.

data\WDataGen.py - Generates sample weather data based on defined sampling.

app\ExampleFlight.py* - Outlines how a flight may be simulated using the above files. (Running this module with a low-end gaming laptop takes approximately one minute to complete the simulation)

app\ExampleMCSim.py* - Outlines how a Monte-Carlo (MC) simulation with uncertainty in cloud cover and wind may be conducted to determine the success rate of a flight. (Running this module with a low-end gaming laptop takes approximately one hour to complete the MC simulation)

*As these files are modules, you will need to enter, for example, "python -m app.ExampleFlight" in the console to run those simulations.

## Required Python packages
1) math
2) numpy
3) scipy
4) pandas
5) pvlib
6) csv
7) timeit
8) matplotlib

*Language: Python 3

If you are interested to know more about the final year project, you may contact me at justinchan111@yahoo.com for a copy of the full report, or for any other enquires.
