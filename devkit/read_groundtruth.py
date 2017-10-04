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
import numpy as np
from shapely.geometry import Point,Polygon,MultiPolygon

####### USAGE #######

# The np_ground_truth variable is an array of 2D numpy arrays, with all polygons.
# Each np_ground_truth[i] is a polygon in a frame i, np_ground_truth[i][j] is a point that belongs to the polygon.

# read ground truth
data = []
np_ground_truth = [] # load ground truth
pol_ground_truth = [] # ground truth coverted to shapely polygon 

np_method_output = [] # this is the output of generic method used to compare with the ground truth
pol_method_output = [] # method output coverted to shapely polygon 

compare_list = []

with open('ground_truth.txt') as f:
    for line in f:
        inner_list = [elt.strip() for elt in line.split(',')]
        data.append(inner_list)

for i in range(0, len(data)):
    tmp = np.array(data[i], dtype=np.float64)
    np_ground_truth.append(np.reshape(tmp, (-1,2)))
    pol_ground_truth.append(Polygon(np_ground_truth[i]))

data = []
with open('method_output.txt') as f:
    for line in f:
        inner_list = [elt.strip() for elt in line.split(',')]
        data.append(inner_list)

for i in range(0, len(data)):
    tmp = np.array(data[i], dtype=np.float64)
    np_method_output.append(np.reshape(tmp, (-1,2)))
    pol_method_output.append(Polygon(np_method_output[i]))

### compare ###

for i in range(0, len(data)):
    compare_list.append(pol_ground_truth[i].intersection(pol_method_output[i]).area/pol_ground_truth[i].union(pol_method_output[i]).area)

