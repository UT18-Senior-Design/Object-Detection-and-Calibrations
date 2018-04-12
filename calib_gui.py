from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import os
import argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from skimage.io import imread
from utils_data import *
from utils_data import CalibFields
import tkinter as tk
import functools
import pandas as pd

class CalibParam:
    def __init__(self,dict_calib,R_0=None):
        if R_0 is None:
            self.R_0 = np.array([[0., 0., -1.],
                                 [0., -1., 0.],
                                 [1., 0., 0.]])
        else:
            self.R_0 = R_0

        self.dict_calib = dict_calib

        # Extrinsic Rotation matrix before changing Coordiate System.
        # (Initial value before applying rotation inputs)
        self.R_l = np.dot(self.R_0.T,
                          self.dict_calib[CalibFields.extrinsic][:3,:3])
        
        # Translation vector
        self.v_t = np.copy(self.dict_calib[CalibFields.extrinsic][:3,3])
    
    def set_Rmat(self,R_in=np.eye(3),v_t=np.zeros(3)):
        self.dict_calib[CalibFields.extrinsic][:3,3] = self.v_t + v_t
        self.dict_calib[CalibFields.extrinsic][:3,:3] = np.dot(self.R_0,
                                    np.dot(R_in,self.R_l))

    def save_Rmat(self,dpath):
        f_int = os.path.join(dpath,'calib_intrinsic.txt')
        f_ext = os.path.join(dpath,'calib_extrinsic.txt')
        np.savetxt(f_int,
                   self.dict_calib[CalibFields.intrinsic],
                   fmt='%.3f',
                   delimiter=' ')
        np.savetxt(f_ext,
                   self.dict_calib[CalibFields.extrinsic],
                   fmt='%.3f',
                   delimiter=' ')

class CalibGUI(tk.Tk):
    def __init__(self,im,points,cparam,outdir,_D_MAX=75.0,_D_MIN=2.0):
        tk.Tk.__init__(self)
        self.title('Lidar-Cam calibrator')
        self.im = im
        self.points = points
        self.im_height,self.im_width = np.shape(im)[:2]
        self.cparam = cparam
        self.outdir = outdir
        self.d_max = _D_MAX
        self.d_min = _D_MIN

        # Define Variables in GUI
        self.var_xr = tk.DoubleVar()
        self.var_yr = tk.DoubleVar()
        self.var_zr = tk.DoubleVar()
        self.var_xt = tk.DoubleVar()
        self.var_yt = tk.DoubleVar()
        self.var_zt = tk.DoubleVar()

        # Define Widgets GUI
        self.label_rot = tk.Label(self, text="\nRotations (degree)")
        self.scale_xr = tk.Scale(self, variable=self.var_xr,
                                 from_=-5, to=5, 
                                 resolution=0.01,
                                 length=300)
        self.label_xr = tk.Label(self, text="x (forward)")
        self.scale_yr = tk.Scale(self, variable=self.var_yr,
                                 from_=-5, to=5,
                                 resolution=0.01,
                                 length=300)
        self.label_yr = tk.Label(self, text="y (left)")
        self.scale_zr = tk.Scale(self, variable=self.var_zr,
                                 from_=-5, to=5, 
                                 resolution=0.01, 
                                 length=300)
        self.label_zr = tk.Label(self, text="z (up)")
        self.label_trans = tk.Label(self, text="\nTranslation (m)")
        self.scale_xt = tk.Scale(self, variable=self.var_xt,
                                 from_=-2, to=2, 
                                 resolution=0.01, 
                                 length=300)
        self.label_xt = tk.Label(self, text="x (down)")
        self.scale_yt = tk.Scale(self, variable=self.var_yt,
                                 from_=-2, to=2, 
                                 resolution=0.01, 
                                 length=300)
        self.label_yt = tk.Label(self, text="y (right)")
        self.scale_zt = tk.Scale(self, variable=self.var_zt,
                                 from_=-2, to=2, 
                                 resolution=0.01, 
                                 length=300)
        self.label_zt = tk.Label(self, text="z (forward)")
        self.button_plot = tk.Button(self, text="Update and Plot Result",
                                     command=self.imlidar_plot)
        self.button_save = tk.Button(self, text="Save Result",
                                     command=self.matsave)

        # Locate widgets
        self.label_rot.grid(row=0,columnspan=6)
        self.scale_xr.grid(row=1,column=0)
        self.label_xr.grid(row=1,column=1)
        self.scale_yr.grid(row=1,column=2)
        self.label_yr.grid(row=1,column=3)
        self.scale_zr.grid(row=1,column=4)
        self.label_zr.grid(row=1,column=5)
        self.label_trans.grid(row=2,columnspan=6)
        self.scale_xt.grid(row=3,column=0)
        self.label_xt.grid(row=3,column=1)
        self.scale_yt.grid(row=3,column=2)
        self.label_yt.grid(row=3,column=3)
        self.scale_zt.grid(row=3,column=4)
        self.label_zt.grid(row=3,column=5)
        self.button_plot.grid(row=4,columnspan=6)
        self.button_save.grid(row=5,columnspan=6)

        plt.imshow(self.im)
        plt.show()

    def imlidar_plot(self,cmap='brg'):
        # Rotation matrix
        r_x = self.var_xr.get()*np.pi/180.0
        r_y = self.var_yr.get()*np.pi/180.0
        r_z = self.var_zr.get()*np.pi/180.0
        
        R_x = np.eye(3)
        R_x[1,1] = np.cos(r_x)
        R_x[2,2] = np.cos(r_x)
        R_x[1,2] = -np.sin(r_x)
        R_x[2,1] = np.sin(r_x)

        R_y = np.eye(3)
        R_y[0,0] = np.cos(r_y)
        R_y[2,2] = np.cos(r_y)
        R_y[0,2] = np.sin(r_y)
        R_y[2,0] = -np.sin(r_y)

        R_z = np.eye(3)
        R_z[0,0] = np.cos(r_z)
        R_z[1,1] = np.cos(r_z)
        R_z[0,1] = -np.sin(r_z)
        R_z[1,0] = np.sin(r_z)

        # Translation vector
        vt_add = np.zeros(3)
        vt_add[0] = self.var_xt.get()
        vt_add[1] = self.var_yt.get()
        vt_add[2] = self.var_zt.get()

        R_in = functools.reduce(np.dot,[R_x,R_y,R_z])

        self.cparam.set_Rmat(R_in,vt_add)

        points2D, pointsDist = project_lidar_to_img(self.cparam.dict_calib,
                                                    self.points,
                                                    self.im_height,
                                                    self.im_width)
        # Clip distance with min/max values
        for i_pt,pdist in enumerate(pointsDist):
            pointsDist[i_pt] = self.d_max if pdist>self.d_max \
                               else pdist if pdist>self.d_min \
                               else self.d_min
        # Rescale to 1~255 with 1/d value
        pointsDist = np.round(minmax_scale(1.0/pointsDist,
                                           1.0/self.d_max,1.0/self.d_min,
                                           1,255)).astype('uint8')
        # Color points w.r.t inversed distance values
        _CMAP = plt.get_cmap(cmap)
        pointsColor = np.array([_CMAP(1.0-pdist/255.0)[:3] \
                                for pdist in pointsDist])
        plt.clf()
        plt.imshow(self.im)
        plt.scatter(points2D[:,1],points2D[:,0],
                    c=pointsColor,
                    s=1)
        plt.xlim(0,self.im_width-1)
        plt.ylim(self.im_height-1,0)
        plt.draw()

    def matsave(self):
        self.cparam.save_Rmat(self.outdir)


def main(_):
    dict_calib = loadCalib(args.f_int,args.f_ext,ltype=args.ltype)

    cparam = CalibParam(dict_calib)
    im = imread(args.f_img)
    # im_height,im_width = np.shape(im)[:2]
    points = None
    if args.f_lidar.lower().endswith('.csv'):
        df = pd.read_csv(args.f_lidar)
        points = df.as_matrix()[:,:3]
    else:
        points = np.fromfile(args.f_lidar).reshape(-1,3)

    calib_app = CalibGUI(im,points,cparam,args.outdir)
    calib_app.mainloop()

def parse_args():
    parser = argparse.ArgumentParser(description=
                        'GUI LIDAR-CAM calibration ')
    parser.add_argument('-image', dest='f_img',
                        help='File path to image',
                        default=None, type=str)
    parser.add_argument('-lidar', dest='f_lidar',
                        help='File path to lidar',
                        default=None, type=str)
    parser.add_argument('-outdir', dest='outdir',
                        help='Path to the output file.(excluding file name)',
                        default='', type=str)
    parser.add_argument('-ext_init', dest='f_ext',
                        help='Path to the initial extrinsic matrix file.',
                        default = None, type=str)
    parser.add_argument('-int_init', dest='f_int',
                        help='Path to the initial intrinsic matrix file.',
                        default = None, type=str)
    parser.add_argument('-ltype', dest='ltype',
                        help='Type of Lidar (velo, m8)',
                        default = 'm8', type=str)
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    print("Called with args:")
    print(args)
    sys.exit(main(args))
