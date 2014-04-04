from openhubo.urdf import URDF,HUBO_JOINT_SUFFIX_MASK
import sys
from openhubo import startup
from numpy import pi
from openhubo import hubo_util

try:
    filename=sys.argv[1]
except IndexError:
    print "Need a URDF model to work on! Rerun with model file as first argument"
    sys.exit(1)

rightmask='R'+HUBO_JOINT_SUFFIX_MASK
leftmask=r'L\1'

model=URDF.load_xml_file(filename)
#Mirror arm and leg chains
model.rename_link('rightPalm','Body_RE1')
model.rename_link('rightFoot','Body_RE2')
model.copy_chain_with_rottrans('Body_Torso','Body_RE1',[0,0,0],[0,0,0],rightmask,leftmask,'y')
model.copy_chain_with_rottrans('Body_TSY','Body_RE2',[0,0,0],[0,0,0],rightmask,leftmask,'y')

#Copy right fingers to left
model.copy_chain_with_rottrans('Body_RWR','Body_RF13',[0,0,0],[0,0,0],rightmask,leftmask,'z')
model.copy_chain_with_rottrans('Body_RWR','Body_RF23',[0,0,0],[0,0,0],rightmask,leftmask,'z')
model.copy_chain_with_rottrans('Body_RWR','Body_RF33',[0,0,0],[0,0,0],rightmask,leftmask,'z')

model.joints['LF11'].axis*=-1
model.joints['LF21'].axis*=-1
model.joints['LF31'].axis*=-1

#Rename to match hubo-ach conventions
model.rename_joint('LF11','LF1')
model.rename_joint('RF11','RF1')
model.rename_joint('RF41','RF2')

model.rename_link('Body_RE1','rightPalm')
model.rename_link('Body_RE2','rightFoot')
model.rename_link('Body_LE1','leftPalm')
model.rename_link('Body_LE2','leftFoot')

#Read in actual model limits from file
model.apply_default_limits(25,2*pi,-pi,pi)
adjuster=hubo_util.LimitProcessor('/etc/hubo-ach/drc-hubo.joint.table','/etc/hubo-ach/drc-hubo-beta-2-default.home')
adjuster.apply_limits_to_urdf(model)

#Shrink finger limits due to mimic:
model.joints['RF2'].limits.upper*=0

model.joints['LF1'].limits.lower/=3
model.joints['RF1'].limits.lower/=3

model.joints['LF1'].limits.upper*=0
model.joints['RF1'].limits.upper*=0
model.joints['LF1'].limits.upper+=.2
model.joints['RF1'].limits.upper+=.2

model.write_xml('drchubo-v3.urdf')
model.write_openrave_files('drchubo-v3')

for n in ['RSP','RSR','RSY','REP','RWY','RWP','RWR']:
    print '{}:'.format(n),model.joints[n].origin.position

for n in ['RHY','RHR','RHP','RKP','RAP','RAR']:
    print '{}:'.format(n),model.joints[n].origin.position
