import os
import pylab as pl
import numpy as np
from perimysium.dataman import TRCFile, storage2numpy, dict2storage
#os.system('./invdyn invdyn_setup.xml')

forces = storage2numpy('dynhop_forces_forces.sto')
invdyn = storage2numpy('invdyn_solution.sto')
invdynf = storage2numpy('invdyn_solution_f.sto')
invdynv = storage2numpy('invdyn_solution_v.sto')
invdyna = storage2numpy('invdyn_solution_a.sto')

pl.figure()
pl.subplot(131)
pl.title('knee')
pl.plot(forces['time'], forces['LKneeSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['LKnee_rz'], label='invdyn')
pl.plot(invdynf['time'], invdynf['LKnee_rz'], label='invdyn_f')
pl.plot(invdynv['time'], invdynv['LKnee_rz'], label='invdyn_v')
pl.plot(invdyna['time'], invdyna['LKnee_rz'], label='invdyn_a')
#pl.plot(invdyna['time'],
#        invdynf['LKnee_rz'] + invdynv['LKnee_rz'] + invdyna['LKnee_rz'],
#        label='invdyn_sum')
pl.legend(frameon=False)
pl.subplot(132)
pl.title('hip')
pl.plot(forces['time'], forces['LHipSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['LHip_rz'], label='invdyn')
pl.plot(invdynf['time'], invdynf['LHip_rz'], label='invdyn_f')
pl.plot(invdynv['time'], invdynv['LHip_rz'], label='invdyn_v')
pl.plot(invdyna['time'], invdyna['LHip_rz'], label='invdyn_a')
#pl.plot(invdyna['time'],
#        invdynf['LHip_rz'] + invdynv['LHip_rz'] + invdyna['LHip_rz'],
#        label='invdyn_sum')
pl.legend(frameon=False)
pl.subplot(133)
pl.title('pelvis')
#pl.plot(forces['time'], forces['LHipSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['Pelvis_ty'], label='invdyn')
pl.plot(invdynf['time'], invdynf['Pelvis_ty'], label='invdyn_f')
pl.plot(invdynv['time'], invdynv['Pelvis_ty'], label='invdyn_v')
pl.plot(invdyna['time'], invdyna['Pelvis_ty'], label='invdyn_a')
#pl.plot(invdyna['time'],
#        invdynf['Pelvis_ty'] + invdynv['Pelvis_ty'] + invdyna['Pelvis_ty'],
#        label='invdyn_sum')
pl.legend(frameon=False)
pl.tight_layout()
pl.savefig('dynhop_forward_vs_invdyn.png')
pl.show()
