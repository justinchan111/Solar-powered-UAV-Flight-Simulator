import numpy as np

# Set weather data generation parameters
max_windvel = 7
cc100dur_mean = 0
cc100dur_sd = 2
cc0dur_mean = 6
cc0dur_sd = 7
veldiff_mean = -0.00307
veldiff_sd = 1.08
angdiff_mean = 0
angdiff_sd = 10


# Generate sample weather data based on weather data generation parameters,
# and the initial wind velocity, wind angle, and cloud cover.
# t_start is in hours
# days defines the time-frame at which the sample data is generated for
# wind_ang accouts only for horizontal angle. +ve direction follows model
def wdata_sampling(t_start, days, windvel_ini, windang_ini, cc_ini):
    tot_hours = days*24  # total hours

    # initialise weather data array
    wdata = np.zeros((tot_hours, 5))
    wdata[:, 0] = np.arange(t_start, t_start + tot_hours)
    wdata[0, 1] = windvel_ini
    wdata[0, 2] = windang_ini
    wdata[0, 4] = cc_ini

    # generate sample for hourly difference in wind velocity and angle
    veldiff_sample = np.random.normal(veldiff_mean, veldiff_sd, tot_hours-1)
    angdiff_sample = np.random.normal(angdiff_mean, angdiff_sd, tot_hours-1)

    # build up wind velocity and angle values
    for i in range(1, tot_hours):
        wdata[i, 1] = wdata[i-1, 1] + veldiff_sample[i-1]
        wdata[i, 2] = wdata[i-1, 2] + angdiff_sample[i-1]

        # wind magnitude can't be negative
        if wdata[i, 1] < 0:
            wdata[i, 1] = 0

        # wind magnitude cant exceed max wind velocity
        if wdata[i, 1] > max_windvel:
            wdata[i, 1] = max_windvel

        if wdata[i, 2] // 360:
            wdata[i, 2] = wdata[i, 2] - np.sign(wdata[i, 2]) * 360

    ccdurs = np.zeros(0)

    # initial sampling of cloud cover duration
    if cc_ini == 1:
        cc100dur_sample = np.random.normal(cc100dur_mean, cc100dur_sd, 1)
        cc100dur_sample = np.round(cc100dur_sample[0])
        if cc100dur_sample < 0:
            cc100dur_sample = 0
        ccdurs = np.append(ccdurs, cc100dur_sample)
        cc_next = 0
    else:
        cc0dur_sample = np.random.normal(cc0dur_mean, cc0dur_sd, 1)
        cc0dur_sample = np.round(cc0dur_sample[0])
        if cc0dur_sample < 0:
            cc0dur_sample = 0
        ccdurs = np.append(ccdurs, cc0dur_sample)
        cc_next = 1

    # subsequent sampling of cloud cover durations
    while sum(ccdurs) < tot_hours:
        if cc_next:
            cc100dur_sample = np.random.normal(
                cc100dur_mean, cc100dur_sd, 1)
            cc100dur_sample = round(cc100dur_sample[0])
            if cc100dur_sample < 0:
                cc100dur_sample = 0
            ccdurs = np.append(ccdurs, cc100dur_sample)
            cc_next = 0

        else:
            cc0dur_sample = np.random.normal(cc0dur_mean, cc0dur_sd, 1)
            cc0dur_sample = round(cc0dur_sample[0])
            if cc0dur_sample < 0:
                cc0dur_sample = 0
            ccdurs = np.append(ccdurs, cc0dur_sample)
            cc_next = 1

    # Build-up cloud cover durations
    is_cc100 = True if cc_ini == 1 else False
    i = 0
    tot_dur = 0  # total duration on build-up
    for dur in ccdurs:
        tot_dur += dur

        # when total duration exceeds defined time-frame, dur is reduced to
        # build-up the sample to the final hour of the defined time-frame
        if tot_dur > tot_hours:
            dur = dur + tot_hours - tot_dur

        if is_cc100:
            wdata[i:i+int(dur), 4] = 1
        else:
            wdata[i:i+int(dur), 4] = 0

        is_cc100 = not is_cc100  # alternate between building 100% and 0% cloud cover
        i += int(dur)
    return wdata
