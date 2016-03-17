#!/usr/bin/env python

import numpy
import cv2
import yaml
import sys
from sensor_msgs.msg import CameraInfo
import argparse

class StereoCameraModel():
    def __init__(self):
        self.cam1 = CameraModel()
        self.cam2 = CameraModel()

    def fromCamchain(camchain, cam1_no, cam2_no, alpha=0.0):
        res = StereoCameraModel()
        res.cam1 = CameraModel.fromCamchain(camchain, cam1_no)
        res.cam2 = CameraModel.fromCamchain(camchain, cam2_no)

        if res.cam2.T is not None:
            R = res.cam2.T[0:3,0:3]
            T = res.cam2.T[0:3,3]
            #print res.cam2.T, R,T
        else:
            print 'error'
            return None

        # construct R
        cv2.stereoRectify(res.cam1.K,
                 res.cam1.D,
                 res.cam2.K,
                 res.cam2.D,
                 res.cam1.size,
                 R,
                 T,
                 res.cam1.R, res.cam2.R, res.cam1.P, res.cam2.P,
                 alpha = alpha)

        return res

    fromCamchain = staticmethod(fromCamchain)

class CameraModel():
    def __init__(self):
        self.camera_name = ""
        self.distortion_model = None
        self.size = (0,0)
        self.D = numpy.zeros((1,0), numpy.float64)
        self.K = numpy.zeros((3,3), numpy.float64)
        self.R = numpy.identity(3, numpy.float64)
        self.P = numpy.zeros((3,4), numpy.float64)

        self.T = None

    def fromCamchain(camchain, cam_no, alpha=0.0):
        d = {}

        cc = camchain["cam%d"%cam_no]

        res = CameraModel()

        res.camera_name = cc['rostopic']

        res.size = (cc['resolution'][0],cc['resolution'][1])
        res.K = intrinsicsToMatrix(cc['intrinsics'])
        res.P = getNewCameraMatrix(camchain, cam_no, alpha)
        res.D = numpy.array(cc['distortion_coeffs'] + [0,])
        res.distortion_model = 'plumb_bob'

        t_name = 'T_cn_cnm%d' % cam_no
        if t_name in cc:
            res.T = numpy.array(cc[t_name])

        return res

    def fromCameraInfo(msg):

        res = CameraModel()

        res.size = msg.size
        res.K = numpy.array(msg.K).reshape((3,3))
        res.D = numpy.array(msg.P)
        res.R = numpy.array(msg.R).reshape((3,3))
        res.P = numpy.array(msg.P).reshape((3,4))
        res.distortion_model = msg.distortion_model

        return res

    def fromROS(d):
        res = CameraModel()

        res.size = (d['image_width'], d['image_height'])
        res.camera_name = d['camera_name']


        res.K = numpy.array(d['camera_matrix']['data']).reshape((3,3))
        res.D = numpy.array(d['distortion_coefficients']['data'])
        res.R = numpy.array(d['rectification_matrix']['data']).reshape((3,3))
        res.P = numpy.array(d['projection_matrix']['data']).reshape((3,4))

        res.distortion_model = d['distortion_model']

        return res

    fromCamchain = staticmethod(fromCamchain)
    fromCameraInfo = staticmethod(fromCameraInfo)
    fromROS = staticmethod(fromROS)

    def toCameraInfo(self):
        msg = CameraInfo()

        (msg.width, msg.height) = self.size

        if self.D.size > 5:
            msg.distortion_model = "rational_polynomial"
        else:
            msg.distortion_model = "plumb_bob"

        msg.D = numpy.ravel(self.D).copy().tolist()
        msg.K = numpy.ravel(self.K).copy().tolist()
        msg.R = numpy.ravel(self.R).copy().tolist()
        msg.P = numpy.ravel(self.P).copy().tolist()

        return msg

    def toROS(self):
        d = {}

        (d['image_width'],d['image_height']) = self.size

        d['camera_name'] = self.camera_name

        d['camera_matrix'] = {'rows': 3, 'cols':4}
        d['camera_matrix']['data'] = numpy.ravel(self.K).copy().tolist()

        d['distortion_model'] = self.distortion_model

        d['distortion_coefficients'] = {'rows': 1, 'cols':5}
        d['distortion_coefficients']['data'] = numpy.ravel(self.D).copy().tolist()

        d['rectification_matrix'] = {'rows': 3, 'cols':3}
        d['rectification_matrix']['data'] = numpy.ravel(self.R).copy().tolist()

        d['projection_matrix'] = {'rows': 3, 'cols':4}
        d['projection_matrix']['data'] = numpy.ravel(self.P).copy().tolist()

        return d

    def toCamchain(self):
        pass

def intrinsicsToMatrix(arr):
    return numpy.array([[arr[0], 0, arr[2]],[0, arr[1], arr[3]],[0, 0, 1]], numpy.float64)


def getNewCameraMatrix(camchain, cam_no, alpha):
    cc = camchain["cam%d"%cam_no]
    intrinsics = numpy.array([699.8135038737817, 0, 659.1282621673959, 0, 701.6007034932228, 364.7560976146094, 0, 0, 1]).reshape((3,3))

    distortion = numpy.array([-0.1775119562089178, 0.03283556548814805, -0.0001889212176202785, 0.002607655892060457, 0])


    intrinsics = intrinsicsToMatrix(cc['intrinsics'])
    distortion = numpy.array(cc['distortion_coeffs'] + [0,])

    size = (cc['resolution'][0],cc['resolution'][1])

    P = numpy.zeros((3, 4), dtype=numpy.float64)
    ncm, _ = cv2.getOptimalNewCameraMatrix(intrinsics, distortion, size, alpha)
    for j in range(3):
        for i in range(3):
            P[j,i] = ncm[j, i]

    return P


def detectFormat(filename):
    return 'camchain'

def readYaml(filename):
    print filename
    f = open(filename,'r')
    d = yaml.load(f)
    return d

    with open(filename,'r') as f:
        d = yaml.load(f)
        return d
    return None

def writeYaml(filename, d):
    with open(filename,'w') as f:
        f.write(yaml.dump(d))
        return True
    return False

formats = ['camchain','camerainfo','ros']

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--input', type=str, nargs='+', help='an integer for the accumulator')
parser.add_argument('--output',  type=str, nargs='+', help='an integer for the accumulator')
parser.add_argument('--input-format', choices=formats, default='camchain')
parser.add_argument('--format', choices=formats, default='ros')
parser.add_argument('--cam', type=int, default=[0,], nargs='*')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument('--mono',dest='stereo',action='store_false')
group.add_argument('--stereo',dest='stereo',action='store_true')
args = parser.parse_args()

if args.stereo and len(args.output) != 2:
    print 'Wrong number of output '
print args

if args.input_format=='camchain':
    camchain = readYaml(args.input[0])
    if args.stereo:
        m = StereoCameraModel.fromCamchain(camchain, args.cam[0], args.cam[1])
    else:
        m = CameraModel.fromCamchain(camchain, args.cam[0])
elif args.input_format=='ros':
    if args.stereo:
        f1 = readYaml(args.input[0])
        f2 = readYaml(args.input[1])
        m = StereoCameraModel.fromROS(f1,f2)
    else:
        f1 = readYaml(file1)
        m = CameraModel.fromROS(f1)
else:
    pass

if args.format=='ros':
    if args.stereo:
        writeYaml(args.output[1], m.cam1.toROS())
        writeYaml(args.output[2], m.cam2.toROS())
    else:
        writeYaml(args.output[1], m.toROS())
