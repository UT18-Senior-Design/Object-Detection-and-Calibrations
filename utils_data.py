from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
#import tensorflow as tf

# Note: float32 is used to meet other tensorflow functions

# 'P_cam': Intrinsic matrix (3x4)
# 'R_lidar2cam': Extrinsic matrix (4x4)
class CalibFields(object):
    intrinsic = 'P_cam'
    extrinsic = 'R_lidar2cam'
    
_D_MAX = 75.0
_D_MIN = 2.0
_CMAP = plt.get_cmap('brg')
_MODE = 'inverse'

init_v_t = np.array([-0.5,0.02,-0.98],dtype=np.float32)
init_R_rot = np.array([[0., 0., -1.],
                       [0., -1., 0.],
                       [1., 0., 0.]],dtype=np.float32)
# init_R_int = np.array([[898.7, 0., 359., 0.],
#                        [0., 901.4, 652., 0.],
#                        [0., 0., 1., 0.]],dtype=np.float32)

init_R_int = np.array([[935.0, 0., 359., 0.],
                       [0., 935., 640., 0.],
                       [0., 0., 1., 0.]],dtype=np.float32)

""" 
<Coordinate system (Lidar)>
x: forward 
y: left
z: up

<Coordinate system (Camera/image)>
x: down
y: right
"""

def loadCalib(f_int,f_ext,R_int=init_R_int,v_t=init_v_t,R_rot=init_R_rot,
              ltype='m8'):
    dict_calib = {}
    # Intrinsic matrix (3x4)
    if f_int:
        dict_calib[CalibFields.intrinsic] = np.loadtxt(f_int, delimiter=' ')\
                                                        .astype(np.float32)
    else:
        dict_calib[CalibFields.intrinsic] = R_int
    # Extrinsic Matrix (4x4)
    if f_ext:
        dict_calib[CalibFields.extrinsic] = np.loadtxt(f_ext, delimiter=' ')\
                                                        .astype(np.float32)
    else:
        dict_calib[CalibFields.extrinsic] = np.eye(4,dtype=np.float32)
        dict_calib[CalibFields.extrinsic][:3,3] = v_t
        if ltype == 'm8':
            dict_calib[CalibFields.extrinsic][:3,:3] = R_rot
        elif ltype == 'velo':
            rot90 = np.zeros((3,3))
            rot90[0,1] = -1.0
            rot90[1,0] = 1.0
            rot90[2,2] = 1.0
            dict_calib[CalibFields.extrinsic][:3,:3] = np.dot(R_rot,rot90)
        else:
            dict_calib[CalibFields.extrinsic][:3,:3] = R_rot
    return dict_calib

def minmax_scale(x,i_min,i_max,o_min,o_max):
    # MinMax scaling of x
    # i_min<= x <= i_max to o_min<= x_new <= o_max
    return (x-i_min)/float(i_max-i_min)*(o_max-o_min)+o_min

def coord_transform(points, t_mat):
    # Change to homogeneous form
    points = np.hstack([points,np.ones((np.shape(points)[0],1))])
    t_points = np.dot(points,t_mat.T)
    print(t_points.shape)
    # Normalize
    t_points = t_points[:,:-1]/t_points[:,[-1]]
    return t_points

def project_lidar_to_img(dict_calib,points,im_height,im_width):
    # Extract depth data first before projection to 2d image space
    trans_mat = dict_calib[CalibFields.extrinsic]
    points3D = coord_transform(points,trans_mat)
    print('points3d shape:', points3D.shape)
    pointsDist = points3D[:,2]

    # Project to image space
    trans_mat = np.dot(dict_calib[CalibFields.intrinsic],trans_mat)
    points2D = coord_transform(points,trans_mat)
    print('points2d shape:', points2D.shape)


    # Find only feasible points
    idx1 = (points2D[:,0]>=0) & (points2D[:,0] <=im_height-1)
    idx2 = (points2D[:,1]>=0) & (points2D[:,1] <=im_width-1)
    idx3 = (pointsDist>=0)
    idx_in = idx1 & idx2 & idx3

    return points2D[idx_in,:], pointsDist[idx_in]

def dist_to_pixel(val_dist, mode,
                  d_max=_D_MAX, d_min=_D_MIN):
    """ Returns pixel value from distance measurment
    Args:
        val_dist: distance value (m)
        mode: 'inverse' vs 'standard'
        d_max: maximum distance to consider
        d_min: minimum distance to consider
    Returns:
        pixel value in 'uint8' format
    """
    val_dist = d_max if val_dist>d_max else val_dist if val_dist>d_min else d_min
    if mode == 'standard':
        return np.round(minmax_scale(val_dist,
                                     d_min,d_max,
                                     1,255)).astype('uint8')
    elif mode == 'inverse':
        return np.round(minmax_scale(1.0/val_dist,
                                     1.0/d_max,1.0/d_min,
                                     1,255)).astype('uint8')
    else:
        # Default is inverse
        return np.round(minmax_scale(1.0/val_dist,
                                     1.0/d_max,1.0/d_min,
                                     1,255)).astype('uint8')

def points_to_img(points2D,pointsDist,im_height,im_width,mode=_MODE):
    points2D = np.round(points2D).astype('int')
    im_depth = np.zeros((im_height,im_width),dtype=np.uint8)
    for i,point in enumerate(points2D):
        im_depth[point[0],point[1]] = dist_to_pixel(pointsDist[i],mode=mode)

    return im_depth.reshape(im_height,im_width,1)

# --------------------------------------------------------------
#                       Functions for Tensorflow
# --------------------------------------------------------------

def tf_coord_transform(points, t_mat):
    # Change to homogeneous form
    points = tf.concat([points,tf.ones([tf.shape(points)[0],1],tf.float32)], 1)
    t_points = tf.matmul(points,tf.transpose(t_mat))
    # Normalize
    t_points = tf.div(t_points[:,:-1],tf.expand_dims(t_points[:,-1],1))
    return t_points

def tf_project_lidar_to_img(dict_calib,points,im_height,im_width):
    # Extract depth data first before projection to 2d image space
    trans_mat = dict_calib[CalibFields.extrinsic]
    points3D = tf_coord_transform(points,trans_mat)
    pointsDist = points3D[:,2]

    # Project to image space
    trans_mat = tf.matmul(dict_calib[CalibFields.intrinsic],trans_mat)
    points2D = tf_coord_transform(points,trans_mat)

    # Find only feasible points
    idx1 = (points2D[:,0]>=0) & (points2D[:,0] <=tf.to_float(im_height)-1)
    idx2 = (points2D[:,1]>=0) & (points2D[:,1] <=tf.to_float(im_width)-1)
    idx3 = (pointsDist>=0)
    idx_in = idx1 & idx2 & idx3

    return tf.boolean_mask(points2D,idx_in), tf.boolean_mask(pointsDist,idx_in)

def tf_dist_to_pixel(val_dist, mode,
                     d_max=_D_MAX, d_min=_D_MIN):
    """ Returns pixel value from distance measurment
    Args:
        val_dist: distance value (m)
        mode: 'inverse' vs 'standard'
        d_max: maximum distance to consider
        d_min: minimum distance to consider
    Returns:
        pixel value in 'uint8' format
    """
    val_dist = tf.maximum(val_dist,d_min)
    val_dist = tf.minimum(val_dist,d_max)
    if mode == 'standard':
        return tf.cast(tf.round(minmax_scale(val_dist,
                                             d_min,d_max,
                                             1,255)),tf.uint8)
    elif mode == 'inverse':
        return tf.cast(tf.round(minmax_scale(1.0/val_dist,
                                             1.0/d_max,1.0/d_min,
                                             1,255)),tf.uint8)
    else:
        # Default is inverse
        return tf.cast(tf.round(minmax_scale(1.0/val_dist,
                                             1.0/d_max,1.0/d_min,
                                             1,255)),tf.uint8)

def tf_points_to_img(points2D,pointsDist,im_height,im_width,mode=_MODE):
    pointsPixel = tf_dist_to_pixel(pointsDist,mode=mode)
    points2D_yx = tf.cast(tf.round(points2D),tf.int32)
    img = tf.scatter_nd(points2D_yx,pointsPixel,[im_height,im_width])

    return tf.expand_dims(img, 2)

def imlidarwrite(fname,im,im_depth):
    """Write image with RGB and depth
    Args:
        fname: file name
        im: RGB image array (h x w x 3)
        im_depth: depth image array (h x w)
    """
    im_out = im.copy()
    im_depth = np.squeeze(im_depth,axis=2)
    idx_h, idx_w = np.nonzero(im_depth)
    for hi,wi in zip(idx_h,idx_w):
        im_out[hi,wi,:] = (255*np.array(
                        _CMAP(im_depth[hi,wi]/255.0)[:3]))\
                        .astype(np.uint8)
    imsave(fname,im_out)
    print("   ... Write:{}".format(fname))
