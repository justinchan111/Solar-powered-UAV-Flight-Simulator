import pandas as pd
from pvlib.location import Location
import numpy as np
import math

# Default solar irradiance data
# Sourced from https://www.ijser.org/paper/Estimation-of-hourly-solar-radiation-on-an-incline-south-facing-surface-in-Port-Harcourt-city-Nigeria.html
# Solar irradiance (time (in hrs), solar irradiance (W/m2))
slr_irr = np.array([
    [0, 0.],
    [1, 0.],
    [2, 0.],
    [3, 0.],
    [4, 0.],
    [5, 0.],
    [6, 132.],
    [7, 400.],
    [8, 700.],
    [9, 950.],
    [10, 1100.],
    [11, 1200.],
    [12, 1260],
    [13, 1200.],
    [14, 1100.],
    [15, 950.],
    [16, 700.],
    [17, 400.],
    [18, 132.],
    [19, 0.],
    [20, 0.],
    [21, 0.],
    [22, 0.],
    [23, 0.],
    [24, 0.]
])


# Determine solar power available at specified time, taking consideration of cloud cover.
# Time in hours
def Pslr(time,  A_slrpnl, cc):
    # lookup table
    while time >= 24:
        time -= 24
    time_upper = math.ceil(time)
    slr_irr_upper = slr_irr[time_upper, 1]
    time_lower = math.floor(time)
    slr_irr_lower = slr_irr[time_lower, 1]
    grad = slr_irr_upper - slr_irr_lower
    I0 = grad * (time - time_lower) + \
        slr_irr_lower  # irradiance w/ clear sky
    I = I0 * (1 - 0.75 * cc**3.4)  # irradiance w/ cloud cover
    # Return absorbed solar power
    return I * A_slrpnl


# Replace the solar irradiance data based on selected co-ordinates, time-zone, and month.
# Data from the 1st of selected month is used.
def repslr_irr(lat, long, timez, month):
    loc = Location(lat, long, timez)
    start_date = '2020-' + str(month) + '-01'
    end_date = '2020-' + str(month) + '-02'
    times = pd.date_range(start=start_date, end=end_date, freq='H', tz=timez)
    cs = loc.get_clearsky(times)
    ghi = np.array(cs['ghi'])
    global slr_irr
    slr_irr[:, 1] = ghi[0:25]
