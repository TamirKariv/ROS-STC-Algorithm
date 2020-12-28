#!/usr/bin/python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion,Twist,Point
from math import pi
import rospy
import math
import time
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
in_loop = 0




def main():
    rospy.init_node('coverage_node')
    grid,service_response = load_grid_from_map()
    write_grid_to_file(grid)
    grid = get_map_from_file()
    tf_listener =  tf.TransformListener()
    rospy.Rate(2)
    tf_listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
    (tr, rot) = tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    # get start points on the grid
    start_x,start_y = get_start_point_on_grid(service_response,tr,rot)
    d_grid = convert_to_d_grid(grid)
    # get the path on the d grid write to the file
    path_d_grid = get_path_for_grid(d_grid,start_x,start_y)	
    write_path_to_file(path_d_grid)
    # build the four grid and get it's path
    four_d_grid = convert_to_four_grid(grid)
    four_grid_path = get_path_for_grid(four_d_grid,start_x / 4,start_y / 4)
    # send the path to the robot
    move_via_path(path_d_grid)

# get start points on the grid
def get_start_point_on_grid(service_response,tr,rot):
	map_x = float(service_response.map.info.origin.position.x)
	map_y = (service_response.map.info.height - 1) * service_response.map.info.resolution + float(service_response.map.info.origin.position.y)
	cordinate_x = round(tr[0] * 2) / 2
	cordinate_y = round(tr[1]  * 2) / 2
	start_x = int((cordinate_x - map_x) / service_response.map.info.resolution / (0.35 / service_response.map.info.resolution)) + 1
	start_y = int((map_y - cordinate_y) / service_response.map.info.resolution / (0.35 / service_response.map.info.resolution)) + 1
	return start_x,start_y
	
	 
# get the path on the grid
def get_path_for_grid(d_grid,start_x,start_y):
    g = Graph(d_grid,start_x,start_y)
    g.addEdges()
    return  g.DFS()
    

# convert the given map to a grid
def convert_map_to_grid(map):
    c = int(math.ceil(0.35 / map.info.resolution))
    w = int(math.floor(map.info.width / c))
    h = int(math.floor(map.info.height / c))
    grid = [[None] * w for _ in xrange(h)]
    old_h = 0
    old_w = 0
    for i in xrange(h):
	row = []
        for j in xrange(w):
            clear = True
            for k in xrange(old_h, old_h + c):
                if clear:
                    for l in xrange(old_w, old_w + c):
                        if map.data[k * map.info.width + l] != 0:
                            grid[i][j] = 1
                            clear = False
                            break
            if clear:
                grid[i][j] = 0
            old_w += c
        old_w = 0
        old_h += c
    return grid
# write the grid to the file
def write_grid_to_file(grid, file_path = "new_grid.txt"):
    with open(file_path, "w+") as f:
        for row in reversed(grid):
            for col in row:
                f.write(str(col))
            f.write("\n")

# write the path on the grid to the file
def write_path_to_file(path, file_path = "Coverage_path.txt"):
    with open(file_path, "w+") as f:
        for i in range(1,len(path)):
		f.write("row:{}  col{}\n".format(path[i][0],path[i][1]))
		
# get the map from the file
def get_map_from_file(file_path='new_grid.txt'):
    lines = open(file_path).read()
    return [item.split() for item in lines.split('\n')[:-1]]

# convert the grid to d grid
def convert_to_d_grid(old_grid):
    d_grid = []
    for i in range(len(old_grid)):
        row = []
        for j in range(len(old_grid[i][0])):
            row.append(int(old_grid[i][0][j]))
        d_grid.append(row)
    return d_grid

# convert the grid to a four grid
def convert_to_four_grid(old_grid):
    four_grid = []
    for i in range(0, len(old_grid) - 1, 2):
        row = []
        for j in range(0, len(old_grid[i][0]) - 1, 2):
            if old_grid[i][0][j] != '1' and old_grid[i][0][j+1] != '1' and old_grid[i+1][0][j] != '1' and old_grid[i+1][0][j+1] != '1':
	        row.append(0)
            else: row.append(1)
        four_grid.append(row)
    return four_grid

# load the grid from the map service
def load_grid_from_map():
        rospy.wait_for_service('static_map')
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        service_response = get_static_map()
        grid_from_map = convert_map_to_grid(service_response.map)
	return grid_from_map,service_response


# moving the robot via the points in the path
def move_via_path(points):
    for i in range(len(points)-1):
	    move_by_point(points[i][0],points[i][1],points[i+1][0],points[i+1][1])

# move the robot by a single point
def move_by_point(x1,y1,x2,y2):
	global in_loop	
	print ("Going from Point [{},{}] to Point [{},{}] on the Grid".format(x1,y1,x2,y2))
        if y1>y2:
	    move_left()
	    move_up()
	if y1<y2:
	    move_right()
	    move_up()
	    in_loop += 1
	    #escpae loops if found
	    if in_loop == 4:
		move_left()
                move_up()
                move_left()
		in_loop = 0
	if x1<x2:
	    move_right()
	    move_right()
	    move_up()
	if x1>x2:
	    move_up()

# move the robot up
def move_up():
	time.sleep(0.1)
        msg = Twist()
	distance = 0.35*3
	cur_time = rospy.Time.now().to_sec()
	cur_distance = 0
	msg.linear.x = 0.3
	while(cur_distance < distance):
		command_pub.publish(msg)
		updated_time=rospy.Time.now().to_sec()
		cur_distance= 0.5*(updated_time-cur_time)
	msg.linear.x = 0
	command_pub.publish(msg)
# move the robot right
def move_right():
	time.sleep(0.1)
	PI = 3.1415926535897			
	relative_angle = 90*2*PI/360
	msg = Twist()
	msg.angular.z = 0.5
	cur_time = rospy.Time.now().to_sec()
	cur_angle = 0
	while(cur_angle < relative_angle):
		command_pub.publish(msg)
		updated_time=rospy.Time.now().to_sec()
		cur_angle = 0.5*(updated_time-cur_time)
	msg.angular.z = 0
	command_pub.publish(msg)
#move the robot left
def move_left():
	time.sleep(0.1)
	move_msg = Twist()
	PI = 3.1415926535897
	relative_angle = 90*2*PI/360
	msg = Twist()
	msg.angular.z = -0.5
	cur_time = rospy.Time.now().to_sec()
	cur_angle = 0
	while(cur_angle < relative_angle):
		command_pub.publish(move_msg)
		updated_time=rospy.Time.now().to_sec()
		cur_angle = 0.5*(updated_time-cur_time)
	move_msg.angular.z = 0
	command_pub.publish(move_msg)



# find the spanning tree using DFS	
from collections import defaultdict 
class Graph: 
    def __init__(self,d_grid,start_x,start_y): 
        self.graph = defaultdict(list)
	self.d_grid = d_grid
	self.start_x = start_x
	self.start_y = start_y
	self.visited = list()

  
    def addEdge(self, u, v, val):
        if val == 0: 
            self.graph[u].append(v)
  
    def DFSUtil(self, v):
        self.visited.append(v)
        for i in self.graph[v]:
            if i not in self.visited:
                self.DFSUtil(i) 
  

    def DFS(self):
        self.DFSUtil((self.start_x,self.start_y)) 
        return self.visited

    def addEdges(self):
    	for i in range(len(self.d_grid)):
            for j in range(len(self.d_grid[i])):
                if (i == 0):
		    self.addUpperEdges(i,j)
		elif (i == len(self.d_grid) - 1):
		    self.addButtomEdges(i,j)
		else:
		    self.addRegularEdges(i,j)
			
    def addUpperEdges(self,i,j):
        if j == 0:
            self.addEdge((i, j), (i, j + 1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])
            self.addEdge((i, j), (i+1, j+1), self.d_grid[i+1][j+1])
        elif j == len(self.d_grid[i]) - 1:
            self.addEdge((i, j), (i, j - 1), self.d_grid[i][j-1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])
            self.addEdge((i, j), (i+1, j-1), self.d_grid[i+1][j-1])
        else:
            self.addEdge((i, j), (i, j + 1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])
            self.addEdge((i, j), (i+1, j+1), self.d_grid[i+1][j+1])
            self.addEdge((i, j), (i, j - 1), self.d_grid[i][j-1])
            self.addEdge((i, j), (i+1, j-1), self.d_grid[i+1][j-1])
    def addButtomEdges(self,i,j):
        if j == 0:
            self.addEdge((i, j), (i, j + 1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j+1), self.d_grid[i-1][j+1])
        elif j == len(self.d_grid[i]) - 1:
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j-1), self.d_grid[i-1][j-1])
            self.addEdge((i, j), (i, j-1), self.d_grid[i][j-1])
        else:
            self.addEdge((i, j), (i, j + 1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j+1), self.d_grid[i-1][j+1])
            self.addEdge((i, j), (i-1, j-1), self.d_grid[i-1][j-1])
            self.addEdge((i, j), (i, j-1), self.d_grid[i][j-1])
    def addRegularEdges(self,i,j):
        if j == 0:
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j+1), self.d_grid[i-1][j+1])
            self.addEdge((i, j), (i, j+1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i+1, j+1), self.d_grid[i+1][j+1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])
        elif j == len(self.d_grid[i]) - 1:
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j-1), self.d_grid[i-1][j-1])
            self.addEdge((i, j), (i, j-1), self.d_grid[i][j-1])
            self.addEdge((i, j), (i+1, j-1), self.d_grid[i+1][j-1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])
        else:
            self.addEdge((i, j), (i-1, j), self.d_grid[i-1][j])
            self.addEdge((i, j), (i-1, j+1), self.d_grid[i-1][j+1])
            self.addEdge((i, j), (i, j+1), self.d_grid[i][j+1])
            self.addEdge((i, j), (i+1, j+1), self.d_grid[i+1][j+1])
            self.addEdge((i, j), (i-1, j-1), self.d_grid[i-1][j-1])
            self.addEdge((i, j), (i, j-1), self.d_grid[i][j-1])
            self.addEdge((i, j), (i+1, j-1), self.d_grid[i+1][j-1])
            self.addEdge((i, j), (i+1, j), self.d_grid[i+1][j])

 
if __name__ == "__main__":
	main()



	
