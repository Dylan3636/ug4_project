#!/home/dylan/miniconda3/envs/mlp/bin/python

import numpy as np
from scipy.signal import medfilt


def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(np.deg2rad, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r


def bearing(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(np.deg2rad, [lon1, lat1, lon2, lat2])
    _bearing = np.arctan2(np.sin(lon2-lon1)*np.cos(lat2), np.cos(lat1)*np.sin(lat2)-np.sin(lat1)*np.cos(lat2)*np.cos(lon2-lon1))
    return _bearing


def heading_norm(heading):
    while heading<0:
        heading = heading+360
    while heading>360:
        heading = heading-360
    return heading


def geo2coord(ref_lon, ref_lat, lon, lat, ycoord=False):
    distance = haversine(ref_lon, ref_lat, lon, lat)
    _bearing = bearing(ref_lon, ref_lat, lon, lat)
    if ycoord:
        coord = distance*np.cos(_bearing)
    else:
        coord = distance*np.sin(_bearing)
    return coord


def cs2vectorcoord(speed, heading, ycoord=True):
    speed = speed*1000.0/3600.0 # Convert speed from km/h to m/s
    heading = np.deg2rad(heading) # Convert heading from deg to rad
    if ycoord:
        vectorcoord = speed*np.sin(heading)
    else:
        vectorcoord = speed*np.cos(heading)

    return vectorcoord


def angle_between(x1, x2, y1, y2):
    return np.arctan2(y2-y1, x2-x1)


def smooth(x,window_len=11,window='hanning'):
    import numpy
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    if x.size < window_len:
        raise ValueError("Input vector needs to be bigger than window size.")

    if window_len<3:
        return x

    if window == 'median':
        return medfilt(x, window_len)

    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")

    s=numpy.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
    if window == 'flat': # moving average
        w=numpy.ones(window_len,'d')
    else:
        w=eval('numpy.'+window+'(window_len)')

    y=numpy.convolve(w/w.sum(), s, mode='valid')[int((window_len-1)/2):-int((window_len-1)/2)]
    return y


def smooth_vessel(vessel):
    window_size=21
    window_type='blackman'
    vessel = vessel.copy()
    shifteddata = vessel.shift(periods=1)
    vessel.loc[:, 'PointwiseHeading'] = np.rad2deg(angle_between(shifteddata.Xcoord, vessel.Xcoord, shifteddata.Ycoord, vessel.Ycoord))
    vessel.loc[:, 'PointwiseHeading'] = vessel.loc[:, 'PointwiseHeading'].apply(heading_norm)
    vessel.loc[:, 'PointwiseVectorXcoord'] = cs2vectorcoord(vessel.SOG, vessel.loc[:, 'PointwiseHeading'], False)
    vessel.loc[:, 'PointwiseVectorYcoord'] = cs2vectorcoord(vessel.SOG, vessel.loc[:, 'PointwiseHeading'], True)
    vessel = vessel.dropna()
    window_size=min(window_size, len(vessel)-5)
    if window_size <15:
        return
    try:
        vessel.loc[:, 'SmoothedPointwiseHeading'] = smooth(vessel.loc[:, 'PointwiseHeading'], window_size, window_type)
    except Exception as e:
        print(e)
        print(window_size, len(vessel))
        print(smooth(vessel.loc[:, 'PointwiseHeading'], window_size, window_type))
    vessel.loc[:, 'SmoothedVectorXcoord'] = cs2vectorcoord(vessel.SOG, vessel.loc[:, 'SmoothedPointwiseHeading'], False)
    vessel.loc[:, 'SmoothedVectorYcoord'] = cs2vectorcoord(vessel.SOG, vessel.loc[:, 'SmoothedPointwiseHeading'], True)
    return vessel

