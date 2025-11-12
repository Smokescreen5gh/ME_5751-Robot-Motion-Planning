#Lab 3 Branch
import cv2
import numpy as np
import math
import warnings
from PIL import Image, ImageTk

from bsTree import *
from Path import *
# from Queue import Queue


class path_planner:
	def __init__(self,graphics):
		self.graphics = graphics
		# self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
		# self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
		#self.graphics.environment.width/height = 2

		self.costmap = self.graphics.map
		self.map_width = self.costmap.map_width
		self.map_height = self.costmap.map_height

		self._init_path_img()
		self.path = Path()

	
		self.set_start(world_x = 0, world_y = 0)
		#self.set_goal(world_x = -167.0, world_y = -9.0, world_theta = .0)

		#self.plan_path()
		self._show_path()


	def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
		self.start_state_map = Pose()
		map_i, map_j = self.world2map(world_x,world_y)
		print("Start with %d, %d on map"%(map_i,map_j))
		self.start_state_map.set_pose(map_i,map_j,world_theta)


	def set_goal(self, world_x, world_y, world_theta = 0):
		self.goal_state_map = Pose()
		map_i, map_j = self.world2map(world_x, world_y)
		print ("our new goal is %d, %d on map"%(map_i,map_j))
		self.goal_state_map.set_pose(map_i, map_j, world_theta)


	#convert a point a map to the actual world position
	def map2world(self,map_i,map_j):
		world_x = -self.graphics.environment.width/2*self.graphics.scale + map_j
		world_y = self.graphics.environment.height/2*self.graphics.scale - map_i
		return world_x, world_y

	#convert a point in world coordinate to map pixel
	def world2map(self,world_x,world_y):
		map_i = int(self.graphics.environment.width/2*self.graphics.scale - world_y)
		map_j = int(self.graphics.environment.height/2*self.graphics.scale + world_x)
		if(map_i<0 or map_i>=self.map_width or map_j<0 or map_j>=self.map_height):
			warnings.warn("Pose %f, %f outside the current map limit"%(world_x,world_y))

		if(map_i<0):
			map_i=int(0)
		elif(map_i>=self.map_width):
			map_i=self.map_width-int(1)

		if(map_j<0):
			map_j=int(0)
		elif(map_j>=self.map_height):
			map_j=self.map_height-int(1)

		return map_i, map_j

	def _init_path_img(self):
		self.map_img_np = 255*np.ones((int(self.map_width),int(self.map_height),4),dtype = np.int16)
		# self.map_img_np[0:-1][0:-1][3] = 0
		self.map_img_np[:,:,3] = 0

	def _show_path(self):
		for pose in self.path.poses:
			map_i = pose.map_i
			map_j = pose.map_j 
			self.map_img_np[map_i][map_j][1] =0
			self.map_img_np[map_i][map_j][2] =0
			self.map_img_np[map_i][map_j][3] =255

		self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
		self.graphics.draw_path(self.path_img)

		# If you want to save the path as an image, un-comment the following line:
		# self.path_img.save('Log\path_img.png')
		
		# If you want to output an image of map and path, un-comment the following two lines
		# self.path_img = toimage(self.map_img_np)
		# self.path_img.show()

	def plan_path(self):
		print("DEBUG: A* planner triggered from GUI right-click")
		
		self._init_path_img()
		self.path.clear_path()

		live_update_rate = 200   # update GUI every 200 nodes
		live_search_vis = True
		nodes_explored = 0
	
		print("Running A* path planning...")
		self.path.clear_path()

		# --- Step 1: Initialization ---
		costmap = self.costmap.costmap          # 2D numpy array
		start = (self.start_state_map.map_i, self.start_state_map.map_j)
		goal  = (self.goal_state_map.map_i,  self.goal_state_map.map_j)

		print(f"Start: {start}  Goal: {goal}")


		# --- Step 2: Helper functions ---

			# Heuristic: straight-line distance between node and goal
		def heuristic(a, b):
			return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

			# Get all valid 8-connected neighbors (diagonals included)
		def get_neighbors(node):
			i, j = node
			H, W = costmap.shape
			nbrs = []
			for di in [-1, 0, 1]:
				for dj in [-1, 0, 1]:
					if di == 0 and dj == 0:
						continue
					ni, nj = i + di, j + dj
					if 0 <= ni < H and 0 <= nj < W:
						nbrs.append((ni, nj))
			return nbrs
		
		print("Heuristic (0,0→3,4):", heuristic((0,0), (3,4))) 
		print("Neighbors of (0,0):", get_neighbors((0,0))[:5])

		# --- Step 3: Setup A* data structures ---
		import heapq  # make sure this is imported at the top of your file

		open_list = []           # priority queue → stores (f, node)
		heapq.heappush(open_list, (0, start))

		came_from = {}           # to reconstruct path
		g_score = {start: 0.0}   # actual cost from start
		closed_set = set()       # explored nodes
		mu = 1                # heuristic weight (1 = A*, 0 = Dijkstra)

		print("Open list initialized:", open_list)


		# --- Step 4: Main A* loop ---
		nodes_explored = 0

		while open_list:
			# get the node with the lowest f-score
			_, current = heapq.heappop(open_list)
			nodes_explored += 1

			if live_search_vis:
				i, j = current
				self.map_img_np[i, j, 0] = 0     # red
				self.map_img_np[i, j, 1] = 0     # green
				self.map_img_np[i, j, 2] = 255   # blue
				self.map_img_np[i, j, 3] = 255   # opacity

				# refresh GUI every N nodes
				if (nodes_explored % live_update_rate) == 0:
					self.path_img = Image.frombytes(
						'RGBA',
						(self.map_img_np.shape[1], self.map_img_np.shape[0]),
						self.map_img_np.astype('b').tobytes()
					)
					self.graphics.draw_path(self.path_img)
					try:
						self.graphics.canvas.update_idletasks()
						self.graphics.canvas.update()
					except:
						pass

			# skip if already processed
			if current in closed_set:
				continue
			closed_set.add(current)

			# check if we reached the goal
			if current == goal:
				print(f"Goal reached! Explored {nodes_explored} nodes.")
				break

			# explore all 8 neighbors
			for neighbor in get_neighbors(current):
				# skip obstacles (costmap values above ~250 = walls)
				if costmap[neighbor[0], neighbor[1]] > 250:
					continue

				# compute movement cost
				di = abs(neighbor[0] - current[0])
				dj = abs(neighbor[1] - current[1])
				step_cost = math.sqrt(2) if di and dj else 1.0

				# tentative g-score = cost so far + step + map cost
				tentative_g = g_score[current] + step_cost + costmap[neighbor[0], neighbor[1]]

				if tentative_g < g_score.get(neighbor, float('inf')):
					came_from[neighbor] = current
					g_score[neighbor] = tentative_g
					f_score = tentative_g + mu * heuristic(neighbor, goal)
					heapq.heappush(open_list, (f_score, neighbor))

		# --- Step 5: Reconstruct the final path ---
		print("Reconstructing final path...")

		path_nodes = []
		node = goal

		if node not in came_from:
			print("⚠️ No path found to goal.")
			return

		while node in came_from:
			path_nodes.append(node)
			node = came_from[node]

		# include the start node and reverse the list
		path_nodes.append(start)
		path_nodes.reverse()

		print(f"Final path length: {len(path_nodes)} nodes")

		# add each node to self.path for visualization
		for (i, j) in path_nodes:
			self.path.add_pose(Pose(map_i=i, map_j=j, theta=0))


		# save path to CSV (optional)
		self.path.save_path(file_name="Log/path_lab3.csv")

		print("✅ Path reconstruction complete and saved.")


		self.path.save_path(file_name="Log\path.csv")


