import numpy as np

centers = np.array([[196,215.5],[405,215.5]]) # upper, lower
lines   = np.array([[[196,95],[405,95]],[[196,336],[405,336]]])	
# lines = [left line, right line] with left/right line = [upper point, lower point]
radius  = 120.5

def nearest_point(given_point):
	x,y = given_point
	if x < centers[0,0]:	# upper circle
		vec_center_given = given_point - centers[0]
		vec_circle = vec_center_given *radius/np.linalg.norm(vec_center_given)
		return centers[0] + vec_circle

	elif x > centers[1,0]:	# lower circle
		vec_center_given = given_point - centers[1]
		vec_circle = vec_center_given *radius/np.linalg.norm(vec_center_given)
		return centers[1] + vec_circle

	elif y <=centers[0,1]: # left line
		return [given_point[0], lines[0,0,1]]

	elif y >centers[0,1]:	# right line
		return [given_point[0], lines[1,0,1]]

	else:
		print("ERROR in choice of track part!")
		stopTheScript()

p=[10,10]
print(nearest_point(p))