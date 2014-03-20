from Calibrator import Calibrator
from pymavlink import mavutil
from optparse import OptionParser
from pymavlink.rotmat import Vector3
import sys

parser = OptionParser("magfit_scaling.py [options]")
parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("--noise", type='float', default=0, help="noise to add")

(opts, args) = parser.parse_args()

if len(args) < 1:
    print("Usage: magfit.py [options] <LOGFILE...>")
    sys.exit(1)

def noise():
    '''a noise vector'''
    from random import gauss
    v = Vector3(gauss(0, 1), gauss(0, 1), gauss(0, 1))
    v.normalize()
    return v * opts.noise

def graph(samples, offset = Vector3(0.0,0.0,0.0), scaling = Vector3(1.0,1.0,1.0)):
    from numpy import *
    import pylab as p
    from mpl_toolkits.mplot3d import Axes3D
    
    x1 = []
    y1 = []
    z1 = []
    
    x2 = []
    y2 = []
    z2 = []
    
    for i in samples:
        x1.append(i[0])
        y1.append(i[1])
        z1.append(i[2])
        x2.append((i[0]+offset.x)*scaling.x)
        y2.append((i[1]+offset.y)*scaling.y)
        z2.append((i[2]+offset.z)*scaling.z)
    
    fig=p.figure()
    ax1 = fig.add_subplot(1,1,1, projection='3d')
    ax1.scatter(x1,y1,z1,c='r')
    ax1.scatter(x2,y2,z2,c='b')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    p.show()

def magfit(logfile):
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps)
    cal = Calibrator()
    
    offsets = Vector3(0,0,0)
    
    while True:
        m = mlog.recv_match(condition=opts.condition)
        if m is None:
            break
        if m.get_type() == "SENSOR_OFFSETS":
            # update current offsets
            offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if m.get_type() == "RAW_IMU":
            mag = Vector3(m.xmag, m.ymag, m.zmag)
            # add data point after subtracting the current offsets
            cal.add_sample(mag - offsets + noise())
        if m.get_type() == "MAG":
            offsets = Vector3(m.OfsX, m.OfsY, m.OfsZ)
            mag = Vector3(m.MagX, m.MagY, m.MagZ)
            cal.add_sample(mag - offsets + noise())
    
    (offsets, scaling) = cal.calibrate()
    print len(cal.samples)
    print offsets, scaling
    graph(cal.samples, offsets, scaling)
    print "mavgraph.py %s mag_field(RAW_IMU,SENSOR_OFFSETS,(%f,%f,%f),(%f,%f,%f)" % (filename,offsets.x,offsets.y,offsets.z,scaling.x,scaling.y,scaling.z)
    
for filename in args:
    magfit(filename)