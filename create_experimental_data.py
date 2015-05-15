import os
import numpy as np
from scipy import interpolate
import pylab as pl
from perimysium.dataman import TRCFile, storage2numpy, dict2storage
from perimysium.postprocessing import filter_critically_damped
from collections import OrderedDict

# Create motion capture data.
# ===========================

# Load in forward simulation results.
# -----------------------------------
def create_array(column_prefix, fname):
    pointkin = storage2numpy(fname)
    array = np.empty((pointkin.shape[0], 3))
    array[:, 0] = pointkin[column_prefix + '_X']
    array[:, 1] = pointkin[column_prefix + '_Y']
    array[:, 2] = pointkin[column_prefix + '_Z']
    return pointkin['time'], array

t, pelvis = create_array('pelvis', 'dynhop_pelviskin_pelvis_pos.sto')
t, knee = create_array('knee', 'dynhop_kneekin_knee_pos.sto')
t, foot = create_array('foot', 'dynhop_footkin_foot_pos.sto')

# Interpolate.
# ------------
def interp(x, xp, fp):
    num_columns = fp.shape[1]
    array = np.empty((len(x), num_columns))
    for i in range(num_columns):
        array[:, i] = np.interp(x, xp, fp[:, i])
    return array
t_max = 1
trc_rate = 100.0
trc_time = np.linspace(0, t_max, t_max * trc_rate)
trc_pelvis = interp(trc_time, t, pelvis)
trc_knee = interp(trc_time, t, knee)
trc_foot = interp(trc_time, t, foot)

# Filter.
# -------
# TODO


# Write out TRC file.
# -------------------
trc = TRCFile(
        data_rate=trc_rate,
        camera_rate=trc_rate,
        num_frames=len(trc_time),
        num_markers=0,
        units='mm',
        orig_data_rate=trc_rate,
        orig_data_start_frame=1,
        orig_num_frames=len(trc_time),
        time=trc_time,
        )

meters_to_mm = 1000.0
def add_marker(trc, name, data):
    trc.add_marker(name, meters_to_mm * data[:, 0],
                         meters_to_mm * data[:, 1],
                         meters_to_mm * data[:, 2])
add_marker(trc, "Pelvis", trc_pelvis)
add_marker(trc, "LKnee", trc_knee)
add_marker(trc, "LFoot", trc_foot)

trc.write("dynhop.trc")


# Create external loads files.
# ============================
forces = storage2numpy('dynhop_forces_forces.sto')
grf = OrderedDict()

# Negate.
# -------
grf_rate = 2000.
grf['time'] = np.linspace(0, t_max, t_max * grf_rate)
grf['ground_force_vx'] = -forces['LFootContactPlatformforceX']
grf['ground_force_vy'] = -forces['LFootContactPlatformforceY']
grf['ground_force_vz'] = -forces['LFootContactPlatformforceZ']
grf['ground_torque_x'] = -forces['LFootContactPlatformtorqueX']
grf['ground_torque_y'] = -forces['LFootContactPlatformtorqueY']
grf['ground_torque_z'] = -forces['LFootContactPlatformtorqueZ']

# Resample at a constant frequency.
# ---------------------------------
def resample(entry):
    grf[entry] = np.interp(grf['time'], forces['time'], grf[entry])

resample('ground_force_vx')
resample('ground_force_vy')
resample('ground_force_vz')
resample('ground_torque_x')
resample('ground_torque_y')
resample('ground_torque_z')


# Filter.
# -------
def filt(entry):
    cutoff_frequency = 50
    order = 2
    grf[entry] = filter_critically_damped(grf[entry],
            grf_rate, cutoff_frequency, order)

#filt('ground_force_vx')
#filt('ground_torque_z')


# Compute center of pressure.
# ---------------------------
# TODO choose a better interpolater; maybe a spline.
# s = 0 means no smoothing.
copx_spline = interpolate.splrep(t, foot[:, 0], s=0) 
# der is order of derivative.
copx = interpolate.splev(grf['time'], copx_spline, der=0)

copy_spline = interpolate.splrep(t, foot[:, 1], s=0)
copy = interpolate.splev(grf['time'], copy_spline, der=0)
copz_spline = interpolate.splrep(t, foot[:, 2], s=0)
copz = interpolate.splev(grf['time'], copz_spline, der=0)
grf['ground_force_px'] = copx
grf['ground_force_py'] = copy
grf['ground_force_pz'] = copz

# Convert to ndarray.
# -------------------

dict2storage(grf, 'ground_reaction.mot')

os.system('%s/bin/ik -S ik_setup.xml' % os.environ['OPENSIM_HOME'])
os.system('%s/bin/id -S id_setup.xml' % os.environ['OPENSIM_HOME'])
#os.system('%s/bin/rra -S rra_setup.xml' % os.environ['OPENSIM_HOME'])
#os.system('./invdyn invdyn_setup.xml')





















