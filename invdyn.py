import os
import pylab as pl
import numpy as np
from perimysium.dataman import TRCFile, storage2numpy, dict2storage
#os.system('./invdyn invdyn_setup.xml')

forces = storage2numpy('dynhop_forces_forces.sto')
invdyn = storage2numpy('invdyn_solution.sto')

pl.figure()
pl.subplot(131)
pl.title('knee')
pl.plot(forces['time'], forces['LKneeSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['LKnee_rz'], label='invdyn')
pl.legend(frameon=False)
pl.subplot(132)
pl.title('hip')
pl.plot(forces['time'], forces['LHipSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['LHip_rz'], label='invdyn')
pl.legend(frameon=False)
pl.subplot(133)
pl.title('pelvis')
#pl.plot(forces['time'], forces['LHipSpring_Force'], label='forward')
pl.plot(invdyn['time'], invdyn['Pelvis_ty'], label='invdyn')
pl.legend(frameon=False)
pl.show()
