#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Author LRM
# Email lrmicmc@gmail.com

#!/usr/bin/env python

import sys 
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import math

from shapely.geometry import Point,Polygon,MultiPolygon

################################################################################
# CONSTANTS
grid_size_m = 60
cell_size_m = 0.075
image_size  = int(grid_size_m/cell_size_m);
center_col = image_size / 2
center_row = image_size / 2

# Create a black image
img = np.zeros((image_size,image_size,3), np.uint8)
img_tmp = None
img_rst = None
img_map = None

exit_flag = False
################################################################################
ready = False
drawing = False # true if mouse is pressed
mode    = True  # if True, draw rectangle
ix,iy = -1,-1

points = []

# record the ground truth in a txt file
ground_truth = open("ground_truth.txt", "w")
ground_truth_camera = open("ground_truth_camera.txt", "w")


# mouse callback function
def draw_object(event,x,y,flags,param):
    global ix,iy,drawing,mode,img,img_tmp,img_rst,ready

    if event == cv2.EVENT_LBUTTONDOWN:
      if ready == True:
        ready = False
        img = img_rst.copy()
      else:
        img_rst = img.copy()
        
      drawing = True
      ix,iy = x,y
      points.append( (x,y) )
      img_tmp = img.copy()
      
      
    elif event == cv2.EVENT_MOUSEMOVE:
      if len(points) > 0 and drawing == True:
        img = img_tmp.copy()
        pts = np.array([points])
        cv2.polylines(img, [pts], False, (255,255,0))
    
    elif event == cv2.EVENT_RBUTTONDOWN:
      drawing = False
      ready = True
      img = img_tmp.copy()
      pts = np.array([points])
      cv2.polylines(img, [pts], True, (0,255,0))
      #cv2.fillConvexPoly(img, pts, (0,255,0))
################################################################################
def draw_points( msg ):
  global img
  data = pc2.read_points(msg, field_names = None, skip_nans = False)
  for p in data:
    if p[2] < 0:
      px = center_col + int(p[0]/cell_size_m);
      py = center_row - int(p[1]/cell_size_m);
      cv2.circle(img,(px,py),1,(0,0,int(p[3])),-1)

################################################################################
def draw_points_camera( msg ):
  global img
  data = pc2.read_points(msg, field_names = None, skip_nans = False)
  quaternion = (-0.499,0.5,-0.499,0.5)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  for p in data: 
    if p[2] > 0:     
      x_ = p[2] * math.cos(math.pi) - p[0]*math.sin(math.pi)
      y_ = p[2] * math.sin(math.pi) + p[0]*math.cos(math.pi)          
      px = center_col + int(x_/cell_size_m);
      py = center_row - int(y_/cell_size_m);
      #cv2.circle(img,(px,py),1,(0,0,int(p[3])),-1)
      cv2.circle(img,(px,py),1,(100,100,100),-1)
################################################################################
#MAIN PROGRAM STARTS HERE
################################################################################
if len(sys.argv) < 2:
  print "usage: \n\t"+sys.argv[0]+" file.bag"
  sys.exit()
  
bag = rosbag.Bag(sys.argv[1])

################################################################################
# Create a black image, a window and bind the function to window
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_object)

################################################################################
# FOR EACH MSG EXECUTE THE INTERFACE TO SELECT OBSTACLE

# read camera bag to make the mask
cameraBag = rosbag.Bag("/home/lrm/Desktop/NOVOSBAGS/novo/camera.bag")
camera_msg = []
for topic, camera_msg, t in cameraBag.read_messages(topics=['/stereo/narrow/points2']):
  pass
  
draw_points_camera(camera_msg)
mask_inImage = []
mask_inWorld = []

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('n'):
      print("image points ", points)
      mask_inImage = np.copy(np.asarray(points, dtype=np.int))

      pts = []
      for p in points:
        x = (p[0]-center_col)*cell_size_m;
        y = (p[1]-center_row)*cell_size_m;
        pts.append((x,y))
      pts = np.array(pts)
      print pts
      mask_inWorld = np.copy(np.array(pts, dtype=np.double))      
      data = np.asarray(pts, dtype=np.float64)
      ground_truth.write(','.join(map(str, data.ravel())) + "\n")
      points = []
      break

pol_mask_inImage = Polygon(mask_inImage)
pol_mask_inWorld = Polygon(mask_inWorld)

print pol_mask_inImage
print pol_mask_inWorld
# end mask


# lets make ground truth
for topic, msg, t in bag.read_messages(topics=['/velodyne_points']):
  print "new msg t="+str(t)
  img = np.zeros((image_size,image_size,3), np.uint8)
  draw_points(msg)
  #draw_points_camera(camera_msg)
  ready = False
  while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('n'):
      print points
      pts = []
      for p in points:
        x = (p[0]-center_col)*cell_size_m;
        y = (p[1]-center_row)*cell_size_m;
        pts.append((x,y))
      pts = np.array(pts)
      print pts      
      data = np.asarray(pts, dtype=np.float64)
      ground_truth.write(','.join(map(str, data.ravel())) + "\n")      

      # intersection
      polIntersectImage = pol_mask_inImage.intersection(Polygon(np.asarray(points, dtype=np.int)))
      polIntersectWorld = pol_mask_inWorld.intersection(Polygon(np.asarray(pts, dtype=np.float64)))
      print polIntersectImage, polIntersectWorld
      
      # extract image points of the polygon to create a numpy
      points = []
      pts_tmp = []
      pts_tmp = polIntersectImage.exterior.coords.xy
      ptsIntersectionImage = []
      for i in range(0, len(pts_tmp[0])):
        ptsIntersectionImage.append((np.int(pts_tmp[0][i]),np.int(pts_tmp[1][i])))              
      ptsIntersectionImage = np.array(ptsIntersectionImage) 
      
      # extract world points of the polygon to create a numpy      
      pts_tmp = []
      pts_tmp = polIntersectWorld.exterior.coords.xy
      ptsIntersectionWorld = []
      for i in range(0, len(pts_tmp[0])):
        ptsIntersectionWorld.append((pts_tmp[0][i],pts_tmp[1][i]))              
      ptsIntersectionWorld = np.array(ptsIntersectionWorld) 
      print ptsIntersectionWorld

      # save camera intersection points in the file (sorry, bad english)
      data = np.asarray(ptsIntersectionWorld, dtype=np.float64)
      ground_truth_camera.write(','.join(map(str, data.ravel())) + "\n")                          
      break

    elif k == 27:
      exit_flag = True
      print "exiting..."
      break      

    elif k == ord('r'): # debug only, 
      cv2.polylines(img, [ptsIntersectionImage], False, (255,140,90))     
      #break  
  
  if exit_flag == True:
    break

    
bag.close()
ground_truth.close()
cv2.destroyAllWindows()








