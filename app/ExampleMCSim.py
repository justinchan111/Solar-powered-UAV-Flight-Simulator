import numpy as np
import timeit
import csv
import matplotlib.pyplot as plt
from model import SolarUAV
from data import SolarModel, WDataGen
start = timeit.default_timer()

print('Defining flight parameters')
# Define flight parameters
t_start = 7.
dur = 100
dt = 0.01
launch_vel = [0, 15, 0]
cr_alt = 500
pts = np.array((
    [0, 0],
    [300, 450],
    [0, 650],
    [-300, 450],
    [0, 0]
)) * 1000

# Define simulation parameters
tot_sim = 10  # total number of simulations for each month
months = np.arange(1, 13)  # months Jan to Dec
CANtot_succ = 0  # number of successful simulations in Canada

print('Start of MC simulations')
# Run simulations
with open(r"C:\Users\justi\Documents\Final year project\Modelling\app\MCResults\MCSim1CAN.csv", 'w') as file:
    writer = csv.writer(file)
    writer.writerow(['Month', 'Success rate (CAN)'])
    for month in months:
        for i in range(1, tot_sim+1):

            # Skip simulations if quarter-way through the MC sim there is no successful flight
            if i > 10 and CANtot_succ == 0:
                continue

            # Generate weather data
            wdata = WDataGen.wdata_sampling(t_start, 5, 0, 0, 0)

            # Generate flight model
            flight_modelCAN = SolarUAV.FlightModel(t_start, dur, dt)
            flight_modelCAN.vel_ini = launch_vel  # set initial velocity
            flight_modelCAN.wdata = wdata  # load weather data
            flight_modelCAN.mission_p2p(
                pts, cr_alt, dsc_rate=0.6, asc_rate=1.6)  # define flight mission
            SolarModel.repslr_irr(
                55, -100, 'America/Winnipeg', month)  # Canada
            print(f'Simulating flight in CAN, month: {month} ........ {i}')

            flight_modelCAN.sim_flight()  # simulate flight

            if flight_modelCAN.flight_succ:
                CANtot_succ += 1

            del flight_modelCAN

        CANrate_succ = CANtot_succ * 100 / tot_sim  # success rate in Canada
        writer.writerow([month, CANrate_succ])  # write result to file
        CANtot_succ = 0

stop = timeit.default_timer()
print('Time: ', stop - start)
print('End of simulations.')
