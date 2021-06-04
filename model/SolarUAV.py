from math import cos, pi, sin, sqrt, tan
import numpy as np
import scipy.integrate as spi
from data import SolarModel
from data import AeroCoeff as ac

g = 9.80665  # gravitational acceleration

# Lookup-table for density of air.
# Format: altitude(km), air density (kg/m^3)
rho_array = np.array((
    [0, 1.225],
    [1, 1.112],
    [2, 1.007],
    [3, 0.9093],
    [4, 0.8134],
    [5, 0.7364],
    [6, 0.6601],
    [7, 0.5900],
    [8, 0.5258],
    [9, 0.4671],
    [10, 0.4135]
))
rho = 1.225  # density of air at current altitude
msg = ''


# Terminal event when SUAV lands on the ground
def hit_ground(t, state_var):
    return state_var[2]


# Class containing information about SUAV and mission parameters, and
# used to define mission profile.
class FlightModel:
    def __init__(self, t_start, dur, dt):
        # Intialise model with realistic SUAV parameters
        # Fixed parameters
        self.m = 18.2  # mass (in kg)
        self.A = 2.857  # wing area (in m^2)
        self.A_slrpnl = 2.857  # area of solar panel (in m^2)
        self.t_start = t_start  # start time of flight (in hrs)
        self.dur = dur  # allowed duration for simulation (in hrs)
        self.t_end = t_start + dur  # maximum duration for simulation (in hrs)
        self.dt = dt  # time step of simulation datas extraction (in hrs)
        # initial position (in m), altitude of 1 m by default
        self.pos_ini = [0, 0, 1]
        self.vel_ini = [0, 0, 0]  # initial velocity (in m)
        self.E_ini = 4590000  # initial battery energy (in J)
        self.E_max = 4590000  # battery capacity (in J)
        self.KP = 0.1  # proportional gain for yaw angle control
        self.eff_sp = 0.2  # solar panel efficiency
        self.eff_prop = 0.72  # Propeller efficiency
        self.P_other = 10  # non-aerdynamic power draw  (in W)

        # Changing SUAV parameters
        self.alpha = 8  # initial angle of attack (in deg)
        self.P_prop = 0  # power to propellers (in W)
        # drag and lift co-efficient
        self.Cl, self.Cd = ac.get_ClCd(self.alpha)
        self.pa_demand = 0  # inital path angle demand

        # Mission parameters
        self.t_asc = 0  # initialise time to reach cruise altitude (in s)
        self.cr_alt = 0  # initialise cruise altitude (in m)
        self.t_cr = 0  # initialise time at cruise altitude (in s)
        # initially no weather data loaded for simulation
        # wdata is in the format [time elapsed(hrs), vel. magnitude(m/s),
        # vel. hor. angle(deg), vel. ver angle(deg), cc (%)]
        self.wdata = np.zeros(0)
        self.wind_x = 0  # wind velocity in x-direction (in m/s)
        self.wind_y = 0  # wind velocity in y-direction (in m/s)
        self.wind_z = 0  # wind velocity in z-direction (in m/s)
        self.cc = 0  # initial cloud cover, between 0 to 1
        self.state = 'asc'  # SUAV initially ascending
        self.mission = None  # initially no mission defined
        self.is_landing = False  # True when SUAV starts to descend for landing at end point
        self.store_PE = False  # initially UAV will not store excess net power as altitude
        # maximum cruise altitude (in m), relevant if storing excess net power
        self.cr_altmax = 8000
        self.asc_rate = 0  # Ascending rate as a multiplier of cruise power
        self.dsc_rate = 0  # Descending rate as a multiplier of cruise power

        # Other solver variables
        self.t_prev = 0  # previous time point (in s)
        self.roll_prev = 0  # roll angle at previous time point (in deg)
        self.flight_succ = False  # flight successful or not
        self.t_hrprev = 0  # previous time point (in hr)
        # Initially not collecting power data throughout simulation for lower run-time
        self.collect_Pdata = False
        self.t_prevint = 0  # previous time point (in s, as an integer)
        # time points in solver (in s, as an integer) for power data
        self.tx = np.zeros(0, dtype=int)
        # propelling power in simulation (in W)
        self.P_propx = np.zeros(0, dtype=int)
        self.P_netx = np.zeros(0, dtype=int)  # net power in simulation (in W)

        # Define coniditions for termination of simulation
        hit_ground.terminal = True
        hit_ground.direction = 0

    def calc_cfP(self):
        self.v_cruise = sqrt(
            2 * self.m * g / (rho * self.A * self.Cl))  # cruise velocity

        # Cruise power
        self.P_cruise = 0.5 * \
            (rho * self.A * self.Cd * self.v_cruise ** 3) / self.eff_prop

    # Function defining simulation for a point-to-point flight mission
    def p2p_fm(self, t, state_var):
        # State variables = [x, y, z, x', y', z', E]
        # State derivatives = [x'gr, y'gr, z'gr, x''gr, y''gr, z''gr, Pnet]
        state_der = np.zeros(7)

        if self.wdata.size:
            t_hr = np.floor(t / 3600) + self.t_start  # time in hours
            if int(t_hr - self.t_hrprev):
                t_hr = int(t_hr)
                self.t_hrprev = t_hr

                # Load weather data for next hour
                self.wind_vel(
                    self.wdata[t_hr, 1], self.wdata[t_hr, 2], self.wdata[t_hr, 3])
                self.cc = self.wdata[t_hr, 4]

        # Evaluate air density at current altitude
        global rho
        if state_var[2] > 0:
            alt_lower = np.floor(state_var[2] / 1000)
            rho_lower = rho_array[int(alt_lower), 1]
            alt_upper = np.ceil(state_var[2] / 1000)
            rho_upper = rho_array[int(alt_upper), 1]
            rho = (rho_upper - rho_lower) * \
                (state_var[2] / 1000 - alt_lower) + rho_lower

        # Compute power consumed
        if state_var[2] < self.cr_alt and self.state == 'asc':
            self.P_prop = self.P_cruise * self.asc_rate
            self.t_asc = t
        elif self.LOI_ncount >= 0 and self.state != 'cr':
            self.state = 'cr'
            # self.alpha = 1 # Uncomment to alter angle of attack when cruising
            self.calc_cfP()
            self.P_prop = self.P_cruise
            self.P_cruise2 = self.P_cruise  # Cruise power at cruise altitude
        elif self.LOI_ncount == -1 and self.state != 'dsc':
            self.t_cr = (t - self.t_asc) / 3600  # in hours
            self.state = 'dsc'
            self.alpha = 2
            self.calc_cfP()
            self.P_prop = self.P_cruise * self.dsc_rate

        # Compute current air speeds
        x_air = state_var[3] - self.wind_x
        y_air = state_var[4] - self.wind_y
        z_air = state_var[5] - self.wind_z
        v_airhor = sqrt(x_air ** 2 + y_air ** 2)  # horizontal airspeed
        v_air = sqrt(x_air ** 2 + y_air ** 2 +
                     z_air ** 2)  # Combined air speed

        # Compute angles
        yaw = -np.arctan2(x_air, y_air)
        climb_ang = np.arctan2(z_air, v_airhor)
        pa = -np.arctan2(state_var[3], state_var[4])  # path angle

        # Compute position difference between current and LOI (location of interest)
        if self.LOI_ncount and not self.is_landing:
            # Travel to locations of interests in specified order
            x_diff = self.LOI[-self.LOI_ncount, 0] - state_var[0]
            y_diff = self.LOI[-self.LOI_ncount, 1] - state_var[1]
        elif self.LOI_ncount == 0 and not self.is_landing:
            # Travel to end point
            x_diff = self.pos_end[0] - state_var[0]
            y_diff = self.pos_end[1] - state_var[1]
        else:
            # Land in a defined square path - change if conditions and self.is_landing only when needed
            if self.LOI_ncount == -5:
                self.LOI_ncount = -1
            x_diff = self.pos_endcirc[-self.LOI_ncount - 1, 0] - state_var[0]
            y_diff = self.pos_endcirc[-self.LOI_ncount - 1, 1] - state_var[1]

        # Check if LOI reached
        # difference in distance between current point and LOI
        d_diff = sqrt(x_diff ** 2 + y_diff ** 2)
        if int(np.floor(d_diff) / 100):
            pass
        else:
            # Move to next LOI when SUAV is within 100 m of LOI
            self.LOI_ncount -= 1

            # End point reached
            if self.LOI_ncount == -1:
                self.is_landing = True
                self.flight_succ = True
                print('End point reached.')

        # Compute path angle to LOI
        self.pa_demand = -np.arctan2(x_diff, y_diff)

        # Compute ground speeds
        state_der[0] = state_var[3]
        state_der[1] = state_var[4]
        state_der[2] = state_var[5]

        # Compute drag and lift
        D = 0.5 * rho * self.A * self.Cd * v_air ** 2
        L = 0.5 * rho * self.A * self.Cl * v_air ** 2

        # We compute the net power
        time_current = self.t_start + t / 3600  # in hours
        state_der[6] = SolarModel.Pslr(time_current, self.A_slrpnl, self.cc,
                                       ) * self.eff_sp - self.P_prop - self.P_other  # P net

        # Store propelling and net power data of simulation
        if self.collect_Pdata:
            seconds_elapsed = int(np.floor(t))
            dt_int = seconds_elapsed - self.t_prevint
            if dt_int > 0:
                self.tx = np.append(self.tx, seconds_elapsed)
                self.P_propx = np.append(
                    self.P_propx, round(self.P_prop, None))
                self.P_netx = np.append(self.P_netx, round(state_der[6], None))
                self.t_prevint = seconds_elapsed

        # Compute thrust
        if v_air != 0 and state_var[6] > 0:
            F = abs(self.P_prop) * self.eff_prop / \
                v_air  # thrust from propeller
        else:
            F = 0

        if self.store_PE and self.state == 'cr':
            if state_var[6] >= self.E_max and state_der[6] > 0:
                if state_var[2] < self.cr_altmax:
                    # convert +ve Pnet into thrust when battery capacity reached and UAV is cruising
                    F = abs(self.P_prop + state_der[6]) * self.eff_prop / v_air
                else:
                    self.calc_cfP()
                    self.P_prop = self.P_cruise

                # no increment to battery energy when battery capacity reached
                state_der[6] = 0
            elif state_der[6] < 0 and state_var[2] > self.cr_alt:
                # consume less power while losing altitude when Pnet is -ve
                self.P_prop = self.P_cruise2 * 0.8
            elif state_var[2] <= self.cr_alt:
                # cruise at cruise altitude
                self.P_prop = self.P_cruise2
        else:
            if state_var[6] >= self.E_max and state_der[6] > 0:
                # no increment to battery energy when battery capacity reached
                state_der[6] = 0

        global msg
        if state_var[6] < 0:
            # no decrement to battery energy when battery is flat
            state_der[6] = 0 if state_der[6] < 0 else state_der[6]
            if msg == '':
                msg = 'Battery out of power!'
                print(msg)

        # Proportional controller
        dt = t - self.t_prev  # current timestep
        if dt > 0:
            # Evaluate roll angle based on the smaller difference between actual and demand path angle
            if pa > 0 and self.pa_demand < 0:
                a = self.pa_demand - pa
                b = pi - pa + pi + self.pa_demand
                roll = a*self.KP if abs(a) < abs(b) else b*self.KP
            elif pa < 0 and self.pa_demand > 0:
                a = self.pa_demand - pa
                b = - pi - pa - pi + self.pa_demand
                roll = a*self.KP if abs(a) < abs(b) else b*self.KP
            else:
                roll = self.KP * (self.pa_demand - pa)
            # if roll < self.KP * 0.01:
            #     roll = 0
            self.roll_prev = roll  # roll angle
        else:
            roll = self.roll_prev
        self.t_prev = t

        # Compute accelerations
        state_der[3] = -((F - D) * cos(climb_ang) -
                         L * sin(climb_ang)) * sin(yaw) / self.m - L * sin(roll) * cos(yaw) / self.m  # x''gr
        state_der[4] = ((F - D) * cos(climb_ang) -
                        L * sin(climb_ang)) * cos(yaw) / self.m - L * sin(roll) * sin(yaw) / self.m  # y''gr
        W = self.m * g  # weight
        state_der[5] = (L * cos(climb_ang) * cos(roll) + (F - D) *
                        sin(climb_ang) - W) / self.m  # z''gr

        return state_der

    # Define point-to-point mission. Minimsum of two points - start and stop
    def mission_p2p(self, points, cr_alt, asc_rate=1.5, dsc_rate=0.8):
        self.mission = 'Point-to-point flight'
        self.calc_cfP()
        self.asc_rate = asc_rate  # cruise power multiplier for ascension
        self.dsc_rate = dsc_rate  # cruise power multiplier power for descension
        points_rows, temp = points.shape
        self.cr_alt = cr_alt  # cruise altitude throughout mission
        if points_rows < 2:
            print('Need two or more points to define point-to-point mission')
            return
        elif points_rows == 2:
            self.pos_ini[0:2] = points[0, :]  # start point
            self.pos_end = points[-1, :]  # end point
            self.LOI_ncount = 0
            self.LOI = np.array([])
        else:
            self.pos_ini[0:2] = points[0, :]  # start point
            self.pos_end = points[-1, :]  # end point
            self.LOI = points[1:-1, :]  # location of interest points
            self.LOI_ncount, temp = self.LOI.shape  # no. of location of interest points
        self.pos_endcirc = np.array((
            [self.pos_end[0], self.pos_end[1] + 2000],
            [self.pos_end[0] - 2000, self.pos_end[1]],
            [self.pos_end[0], self.pos_end[1] - 2000],
            [self.pos_end[0] + 2000, self.pos_end[1]]))  # points for circling descent at end point

    # define wind velocities - hor. angle is angle from y_gr, about z_gr, and ver. angle
    #  is resulting vector rotated from the horizontal plane
    def wind_vel(self, vel, hor_angle, ver_angle):
        hor_angle = hor_angle * pi / 180  # convert angle to rad
        ver_angle = ver_angle * pi / 180  # convert angle to rad
        self.wind_z = vel * sin(ver_angle)
        hor_wind = vel * cos(ver_angle)
        self.wind_x = - hor_wind * sin(hor_angle)
        self.wind_y = hor_wind * cos(hor_angle)

    # Define cloud cover - pct is percentage cc
    def cloud_cov(self, pct):
        self.cc = pct

    # Simulate flight

    def sim_flight(self):
        # Initialise time points in seconds, for ODE solver (starts from 0)
        t_end = (self.dur + self.dt) * 3600
        self.t_ODE = np.arange(0, t_end, self.dt * 3600)
        # sactual intended flight time points in hours
        self.t = self.t_ODE / 3600 + self.t_start

        ini_state_var = np.zeros(7)
        # (x0, y0, z0) initial co-ordinates
        ini_state_var[0], ini_state_var[1], ini_state_var[2] = self.pos_ini
        # initial ground speeds
        ini_state_var[3] = self.vel_ini[0]
        ini_state_var[4] = self.vel_ini[1]
        ini_state_var[5] = self.vel_ini[2]
        # initial battery energy
        ini_state_var[6] = self.E_ini

        # Compute state variables by simulating flight using the respective functions
        if self.mission != 'Point-to-point flight':
            sol = spi.solve_ivp(self.cruise_fm, (0, t_end), ini_state_var,
                                t_eval=self.t_ODE, events=hit_ground,
                                dense_output=True, rtol=1e-8)
        else:
            sol = spi.solve_ivp(self.p2p_fm, (0, t_end), ini_state_var,
                                t_eval=self.t_ODE, events=hit_ground,
                                dense_output=False, rtol=1e-8)
        self.state_var = sol.y

        # Convert solution time points from second to hour
        self.sol_t = sol.t / 3600 + self.t_start
        self.tx = self.tx / 3600 + self.t_start

        global msg
        msg = ''
