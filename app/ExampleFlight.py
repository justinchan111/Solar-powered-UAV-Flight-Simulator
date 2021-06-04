from data import SolarModel
import timeit
import numpy as np
from model import SolarUAV
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

start = timeit.default_timer()

# Define flight parameters
t_start = 7.
dur = 100
dt = 0.01
launch_vel = [0, 15, 0]
cr_alt = 500
# flight points [x position, y position] (in m)
pts = np.array((
    [0, 0],
    [300, 450],
    [0, 650],
    [-300, 450],
    [0, 0]
)) * 1000

# Weather data for 4 days
# wdata is in the format [time elapsed(hrs), vel. magnitude(m/s),
# vel. hor. angle(deg), vel. ver angle(deg), cc (%)]
wdata = np.zeros((1, 5))
for i in range(0, 24):  # day 1
    wdata_row = np.array([i, 0, 0, 0, 0])
    wdata = np.vstack((wdata, wdata_row))
wdata = np.delete(wdata, 0, 0)
for i in range(24, 48):  # day 2
    wdata_row = np.array([i, 0, 0, 0, 0])
    wdata = np.vstack((wdata, wdata_row))
for i in range(48, 96):  # day 3 n 4
    wdata_row = np.array([i, 0, 0, 0, 0])
    wdata = np.vstack((wdata, wdata_row))

flight_model1 = SolarUAV.FlightModel(
    t_start, dur, dt)  # Create SUAV flight model
flight_model1.vel_ini = launch_vel  # set initial velocity
flight_model1.wdata = wdata  # set weather data
# Define flight mission
flight_model1.mission_p2p(pts, cr_alt, dsc_rate=0.5, asc_rate=1.6)
flight_model1.collect_Pdata = True  # collect power data of simulation
# Central Canada, June
SolarModel.repslr_irr(55, -100, 'America/Winnipeg', 6)
# flight_model1.store_PE = True  # Uncomment to store positive net power when at 100% battery charge

flight_model1.sim_flight()  # simulate flight

# Plot results

# Plot velocities
fig1, ax1 = plt.subplots(5, 1, figsize=(8, 12))
ax1[0].set_title('Simulation Results')
t_seconds = (flight_model1.t - t_start) * 3600
v_grhor = np.sqrt(
    flight_model1.state_var[3, :] ** 2 + flight_model1.state_var[4, :] ** 2)
ax1[0].plot(flight_model1.sol_t, v_grhor)
ax1[1].plot(flight_model1.sol_t, flight_model1.state_var[5, :])
ax1[0].set(ylabel='Horizontal Velocity (m/s)')
ax1[1].set(ylabel='z Velocity (m/s)')
ax1[1].set(xlabel='Time (hrs)')
ax1[0].grid(True, which='both')
ax1[1].grid(True, which='both')

# Plot altitude
ax1[2].plot(flight_model1.sol_t, flight_model1.state_var[2, :])
ax1[2].set(ylabel='Altitude (m)')
ax1[2].set(xlabel='Time (hrs)')
ax1[2].grid(True, which='both')


# Plot battery energy level and net power to battery
ax1[3].plot(flight_model1.sol_t,
            flight_model1.state_var[6, :] * 100 / flight_model1.E_max, label=r'$SoC$')
ax1[3].set(ylabel='State of Charge (%)')
ax1[4].set(xlabel='Time (hrs)')
ax1[3].set_ylim(0, 110)
ax1[3].grid(True, which='both')
ax1[4].plot(flight_model1.tx, flight_model1.P_netx, 'r', label=r'$P_{net}$')
ax1[4].plot(flight_model1.tx, flight_model1.P_propx,
            'b', label=r'$P_{prop}$')
ax1[4].set(ylabel='Power (W)')
ax1[4].grid(True, which='both')

# Plot available solar power
t_max = int(np.ceil(flight_model1.sol_t[-1]))
t_pslr = np.arange(int(t_start), t_max)
P_slr = np.zeros(0)
for ttt in t_pslr:
    P_slr = np.append(P_slr, SolarModel.Pslr(
        ttt, flight_model1.A_slrpnl, wdata[ttt, 4])*0.2)
ax1[4].plot(t_pslr, P_slr, '--', label=r'$P_{solar}$')
ax1[4].legend(loc='upper left')

# Plot 3-D flight path taken
fig = plt.figure()
ax = plt.axes(projection="3d")
ax.set_title('3D Flight Path')
ax.plot3D(flight_model1.state_var[0, :] / 1000,
          flight_model1.state_var[1, :] / 1000, flight_model1.state_var[2, :] / 1000)
ax.set(xlabel='x Position (km)')
ax.set(ylabel='y Position (km)')
ax.set(zlabel='z Position (km)')

stop = timeit.default_timer()
print('Time: ', stop - start)
plt.show()
