import cv2
import numpy as np
import math
from PIL import Image, ImageTk
from queue import Queue

 
class cost_map:
	def __init__(self,graphics):
		self.graphics = graphics

		self.inflation_radius = 18 # radius of our robot is 18 pixel or cm
		self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)

		self.map_width = int(self.graphics.environment.width*self.graphics.scale)
		self.map_height = int(self.graphics.environment.height*self.graphics.scale)
		
		# To adjust the map size, please go to E160_graphics.py line 18, and change the self.scale there.
		# self.scale shall be half of your image size, for example, if your map image is 500 x 500 pixel, set your self.scale = 250
		# The default map size is 500 x 500 pixel. In case you want a smaller size for debugging, you can change the value of self.scale.
		
		try:
			self.load_map(map = "Test_Map_3.png") #load map, put your own map here
		except:
			self.graphics.show_map_button.configure(state="disabled")
			print ("no map loaded") #if fail to find the map png
			return
		
		self.show_map()
		self.compute_costmap()
		self.get_vis_map()
		self.save_vis_map(map = "New_Maps/Test_Map_3.png")
		self.save_costmap(file_path= 'New_Maps/Cost_Test_Map_3.txt')
		self.save_distmap(file_path= 'New_Maps/Dist_Test_Map_3.txt')

	# load occupancy grid into self.map
	#self.map is a numpy 2d array
	#initialize self.costmap, a numpy 2d array, same as self.map
	def load_map(self,map="Test_Map_3.png"):
		self.map_img = Image.open(map).convert('L')
		self.map_img = self.map_img.resize((int(self.map_width),int(self.map_height)))
		# self.graphics.draw_map(map_img=self.map_img)
		self.map = cv2.imread(map,cv2.IMREAD_GRAYSCALE)
		print (self.map.dtype)
		print ("Loaded map dimension: %d x %d pixel"%(self.map.shape[0],self.map.shape[1]))
		self.map = cv2.resize(self.map, dsize=(int(self.map_width),int(self.map_height)), interpolation=cv2.INTER_CUBIC)
		
		self.distmap=np.full_like(self.map, -1, dtype=int) #map for saving distance to the nearest obstacle, initialize with an array with all -1

		self.costmap=np.copy(self.map).astype(float) #the cost map you are going to work on
		self.vis_map=np.copy(self.map) #map for visualization, intialize same as the map

	#save your costmap into a grayscale image
	def save_vis_map(self,map="New_Maps/vis_map_3.png"):
		save_img = Image.fromarray(self.vis_map)
		save_img.save(map)

	def show_vis_map(self):
		self.vis_map_img=Image.frombytes('L', (self.vis_map.shape[1],self.vis_map.shape[0]), self.vis_map.astype('b').tostring())
		self.graphics.draw_map(map_img=self.vis_map_img)

	#display costmap on the dialogue window, in most cases, the cost map cannot be properly shown
	def show_costmap(self):
		self.costmap_img=Image.frombytes('L', (self.costmap.shape[1],self.costmap.shape[0]), self.costmap.astype('b').tostring())
		self.graphics.draw_map(map_img=self.costmap_img)

	#display binary occupancy grid on the dialogue window 
	def show_map(self):
		self.graphics.draw_map(map_img=self.map_img)

	def save_costmap(self, file_path, fmt='%.3f', delimiter='\t'):
		np.savetxt(file_path, self.costmap, fmt=fmt, delimiter=delimiter)
		
	def save_distmap(self, file_path, fmt='%d', delimiter='\t'):
		np.savetxt(file_path, self.distmap, fmt=fmt, delimiter=delimiter)


	def compute_costmap(self):
		'''
		-------------------------------------
			Step 1 : SET THE DISTANCE MAP
		-------------------------------------
	
		'''

		# Distance Notation
		# -1 		: Unexplored Pixel
		# 0  		: Occupied Pixel
		# 1,2 ... : Distance from Occupied Pixel

		# Pixel Notaton
		# 0 = Black Pixel
		# 255 = White Pixel

		
		# Set the distances of every pixel to -1. This intializes the map to be unexplored
		self.distmap[:] = -1

		# Define thresholds...due to compression in the image, black pixels may appear gray
		obstacle_pix = 20
		free_pix = 220

		# Assign all black pixels to 0
		self.distmap[self.map < obstacle_pix] = 0
		
		# Assign the height/width of the array to the shape of the map
		height, width = self.map.shape

		'''
		---------------------------------------
		Step 2: Find the Obstacles in the Map
		---------------------------------------

		'''
		
		q = Queue() # Breadth Width Search Algo

		# Find all the obstacles. Goes through every pixel through the 2 for loops
		# If the value of the pixel is less than 20 it is denoted as 0 in the dist map array
		# That pixel is then put in the queue 
		for h in range(height):
			for w in range(width):
				current_value = int(self.map[h, w])
				if current_value < obstacle_pix:
					self.distmap[h,w] = 0
					q.put((h,w))

		'''
		-----------------------------------
		Step 3: Exapand from the Obstacles
		-----------------------------------

		'''
		neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

		while not q.empty():
			h, w = q.get() # Gets the coordinate points that is first in the queue and unpacks the tuple into h and w
			d = self.distmap[h, w] # Starting value of the starting point of our distmap generation. It should be 0 since we are starting at an obstacle point

		for dy , dx in neighbors:
			ny = h + dy # neighbor height
			nx = w + dx # neighbor weidth

			# we are now at coordinate ny, nx

			if ny < 0 or ny >= height: #ensure neighbor is not out of bounds in height (0 is top of the map and height is the utmost bottom of the map)
				continue
			if nx < 0 or nx >= width:  #ensure neighbor is not out of bounds in width
				continue

			if self.distmap[ny ,nx] != -1: #if this coordinate is not -1 means if this point in the array is not explored
				continue

			if int(self.map[ny, nx]) <= free_pix:  # if it isnt a free pixel, then continue and don't expand into it
				continue
			
			self.distmap[ny, nx] = d + 1
			q.put((ny,nx))

		'''
		------------------------------
		Step 4: Generate the Cost Map
		------------------------------

		'''

		# Cost Table
		obstacle = 255.0
		unknown = 150
		decay = 18
		self.costmap[:, :] = 0.0 ## Initalize the entire costmap to all 0

		for h in range(height):
			for w in range(width):
				d = int(self.distmap[h, w])

				if d == 0:
					self.costmap[h, w] = obstacle #if the value is 0, we assign the cost of an obstacle
				elif d == -1:
					self.costmap[h, w] = unknown
				elif d <= int(self.inflation_radius):
					self.costmap[h, w] = obstacle 
				else:
					dist_beyond = d - int(self.inflation_radius)

					t = dist_beyond / float(max(1, decay)) #the distance beyond the inflation walls becomes a smaller, basically controls the decay factor
					if t < 0.0:
						t = 0.0
					if t > 1.0:
						t = 1.0

					gamma = 0.6
					factor = (1.0 - t) ** gamma

					tapered = unknown * factor
					if tapered >= obstacle:
						tapered = obstacle - 1.0
					if tapered < 0.0:
						tapered = 0.0
					self.costmap[h, w] = tapered


	def get_vis_map(self):
		'''
		get the map for visualization
		'''

		min_val = min(np.min(self.costmap),0) 
		max_val = np.max(self.costmap)

		# Normalize the costmap
		self.vis_map = np.uint8(255 - (self.costmap - min_val) / (max_val - min_val) * 255)
