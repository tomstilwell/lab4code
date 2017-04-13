import rospy
import heapq
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
#import pdb
from geometry_msgs.msg import Quaternion

endX = 0
endY = 0
startX = 0
startY = 0
stuff = 0
things = 0

def callbackstart(data):
	print("i used callback start")
	global startX
	global startY	

	startX = (data.pose.pose.position.x)
	startY = data.pose.pose.position.y	
	print(startX)
	print(startY)

def callbackend(data):
	print("i used callback end")
	global endX	
	global endY
	global waypoints
	endX = (data.pose.position.x)
	endY = (data.pose.position.y)	
	print(endX)
	print(endY)
	s = AStar()
	print("made astar object")
	s.init_grid()
	print("init grid")	
	s.process()
	s.get_path()
	print("ran process")
	getWayPoints()
	del waypoints[0]
	del waypoints[0]
	print('waypoints')
	print(waypoints)
	
	print("ranlab3cells")
	global waypointTrack
	waypointTrack = []	
	n = 0
	for elements in waypoints:
		waypointTrack.append((startX, startY))
		global startX
		startX = waypoints[0][0]
		global startY
		startY = waypoints[0][1]
		s.init_grid()		
		s.process()
		s.get_path()
		print("ran process")
		getWayPoints()
		print 'waypoints %d' % (n+1)
		print(waypoints)
		n = n+1
	lab3cells()
def callbackMap(data):
	global maze
	maze = []	
	print("I used callback map")
	#print(maze)
	stuff = data.data
	#print(stuff)	
	i=0
	j=0
	
	while (i<37):
		while (j<37):
			if (stuff[37*i+j]>6):
				maze.append((j,i))
								
				maze.append((j+1,i))
				maze.append((j-1,i))
				maze.append((j,i+1))
				maze.append((j,i-1))
				maze.append((j-1,i-1))
				maze.append((j+1,i+1))
				maze.append((j-1,i+1))
				maze.append((j+1,i-1))
				
				
			j=j+1
		i=i+1
		j=0
	#print(maze)

def callbackCostMap(data):
	global cost
	cost = []
	things = data.data
	i=0
	j=0

	while (i<37):
		while (j<37): 
			
			cost.append(things[i*37+j])
			j=j+1
		i=i+1
		j=0
def callbackCostMapUpdate(data):
	global cost
	cost = []
	things = data.data
	i=0
	j=0

	while (i<37):
		while (j<37): 
			
			cost.append(things[i*37+j])
			j=j+1
		i=i+1
		j=0
	#print(things)
'''	
def callbackMap(data):
	global maze
	maze = []	
	print("I used callback map")
	#print(maze)
	stuff = data.data	
	i=70
	j=125
	
	while (i<380):
		while (j<400):
			if (stuff[480*i+j]<0):
				maze.append((i,j))
			j=j+1
		i=i+1
		j=125
	
	#print(maze)
'''

class Cell(object):
	def __init__(self, x, y, reachable):
		self.reachable = reachable
	        self.x = x
	        self.y = y
	        self.parent = None
	        self.g = 0
	        self.h = 0
	        self.f = 0

class AStar(object):
	def __init__(self):
	        self.opened = []
	        heapq.heapify(self.opened)
	        self.closed = set()
	        self.cells = []
	        self.grid_height = 37
	        self.grid_width = 37

	def init_grid(self):
		
		walls = maze
		for x in range(self.grid_width):
	       		for y in range(self.grid_height):
		    		if (x, y) in walls:
		        		reachable = False
		    		else:
		        		reachable = True
				self.cells.append(Cell(x, y, reachable))
		global endX	    	
		global endY
		global startX
		global startY
		self.start = self.get_cell(int(startX), int(startY))
	    	self.end = self.get_cell(int(endX), int(endY))
		print(self.end)
	def get_heuristic(self, cell):
		return (10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))+10*cost[cell.x*37+cell.y])

	def get_cell(self, x, y):
		#pdb.set_trace()
		return self.cells[x * self.grid_height + y]

	def get_adjacent_cells(self, cell):
	    	cells = []
	    	if cell.x < self.grid_width-1:
			cells.append(self.get_cell(cell.x+1, cell.y))
	    	if cell.y > 0:
	    	    	cells.append(self.get_cell(cell.x, cell.y-1))
	    	if cell.x > 0:
			cells.append(self.get_cell(cell.x-1, cell.y))
	    	if cell.y < self.grid_height-1:
			cells.append(self.get_cell(cell.x, cell.y+1))
	    	return cells
	def get_path(self):
		cell = self.end
		global path
		path = [(cell.x, cell.y)]
		while cell.parent is not self.start:
			cell = cell.parent
			path.append((cell.x, cell.y))
		path.append((self.start.x, self.start.y))
		path.reverse()
		#print(path)
		return path

	def display_path(self):
	    	cell = self.end
		print(cell)
	    	while cell.parent is not self.start:
			cell = cell.parent
			print 'path: cell: %d,%d' % (cell.x, cell.y)
			
	def update_cell(self, adj, cell):
	    	adj.g = cell.g + 10
	    	adj.h = self.get_heuristic(adj)
	    	adj.parent = cell
	    	adj.f = adj.h + adj.g

	def process(self):
	    	heapq.heappush(self.opened, (self.start.f, self.start))
	    	while len(self.opened):
			f, cell = heapq.heappop(self.opened)
			self.closed.add(cell)
			if cell is (self.end):
		    		self.display_path()
		    		break
			adj_cells = self.get_adjacent_cells(cell)
			for adj_cell in adj_cells:
		    		if adj_cell.reachable and adj_cell not in self.closed:
		        		if (adj_cell.f, adj_cell) in self.opened:
		        			if adj_cell.g > cell.g + 10:
		                			self.update_cell(adj_cell, cell)
		        		else:
		        			self.update_cell(adj_cell, cell)
		            			heapq.heappush(self.opened, (adj_cell.f, adj_cell))
		
def navToWayPoint():
	global waypoints	
	pub = rospy.Publisher("/move_base_simple/goal", GridCells, queue_size=1)
	post = PoseStamped()
	post.header.frame_id = 'navgoal'
	goal = Pose()
	point = Point()
	quat = Quaternion()
	quat.w = 1
	quat.x = 0
	quat.y = 0
	quat.z = 0
	point.z = 0
	point.x = waypoints[0[0]]
	point.y = waypoints[0[1]]
def lab3cells():
	pub = rospy.Publisher('colorcells', GridCells, queue_size=1)
	#rospy.init_node('lab3cells', anonymous=True)
	cell_width = 0.3
	cell_height = 0.3
	
	global waypointTrack
	global maze
	#header = 'header'
	global path	
	#cells.header.frame_id = 'map'
	grid = GridCells()
	grid.header.frame_id = 'map'
	grid.cell_width = 1
	grid.cell_height = 1	
	'''	
	for element in path:
		t=Point()
		t.x = element[0]+1
		t.y = element[1]+.5
		t.z = 0
		grid.cells.append(t)
	'''

	for element in waypointTrack:
		t=Point()
		t.x = element[0]+1
		t.y = element[1]+.5
		t.z = 0
		grid.cells.append(t)
	
#	for element in maze:
#		t=Point()
#		t.x = element[0]+1
#		t.y = element[1]+.5
#		t.z = 0
#		grid.cells.append(t)
	#grid = GridCells(header, 0.5, 0.5, a)
	
	while 1:
		pub.publish(grid)

def getWayPoints():
	pub = rospy.Publisher('wayPointTopic', Path, queue_size=1)	
	pathmsg = Path()
	pathmsg.header.frame_id = 'path'
	
	global waypoints
	waypoints = []
	poseStamped = PoseStamped()
	poseStamped.header.frame_id = 'poseStamped'
	MyPose = Pose()
	quat = Quaternion()
	quat.x = 0
	quat.y = 0
	quat.z = 0
	quat.w = 0
	p = Point()
	MyPose.orientation = quat
	global startX
	global startY
	currentX = startX
	currentY = startY
	nextX = startX
	nextY = startY
	for element in path:
		prevX = currentX		
		prevY = currentY
		currentX = nextX
		currentY = nextY
		nextX = element[0]
		nextY = element[1]
	
	
		if not((prevX==currentX)&(currentX==nextX) | (prevY==currentY)&(currentY==nextY)):		 
			waypoints.append((currentX,currentY))
			p.x = element[0]
			p.y = element[1]
			p.z = 0				
			MyPose.position = p
			poseStamped = MyPose 
			pathmsg.poses.append(poseStamped)
	
	
def main():
	
	rospy.init_node('A_star', anonymous = True)	
	rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callbackstart)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackend)
	rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callbackCostMap)
	rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGrid, callbackCostMapUpdate)
	rospy.Subscriber("/map", OccupancyGrid, callbackMap)
	rospy.spin()

if __name__ == '__main__':
	main()





