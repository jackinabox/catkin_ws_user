import numpy as np
import matplotlib.pyplot as plt

ort_mess = np.load("location.npy")

# Model
centers = np.array([[196, 215.5], [405, 215.5]]) / 100.0  # upper, lower
lines = np.array([[[196, 95], [405, 95]], [[196, 336], [405, 336]]]) / 100.0
radius = 120.5 / 100


def nearest_point(x, y):
	given_point = (x, y)

	if x < centers[0, 0]:  # upper semicircle
		vec_center_given = given_point - centers[0]
		vec_circle = vec_center_given * radius/np.linalg.norm(vec_center_given)
		return np.array(centers[0] + vec_circle)

	elif x > centers[1, 0]:  # lower semicircle
		vec_center_given = given_point - centers[1]
		vec_circle = vec_center_given * radius/np.linalg.norm(vec_center_given)
		return np.array(centers[1] + vec_circle)

	elif y <= centers[0, 1]:  # left line
		return np.array([given_point[0], lines[0, 0, 1]])

	elif y > centers[0, 1]:  # right line
		return np.array([given_point[0], lines[1, 0, 1]])

	else:
		print("ERROR in choice of track part!")


ort_mess = ort_mess.reshape((len(ort_mess)/2, 2))
ort_theo = np.zeros(ort_mess.shape)
# print(ort_theo.shape)

for i in range(len(ort_mess)):
	ort_theo[i, :] = nearest_point(ort_mess[i, 0], ort_mess[i, 1])

plt.scatter(ort_mess[:, 0], ort_mess[:, 1], s=1, marker="o")
plt.scatter(ort_theo[:, 0], ort_theo[:, 1], s=1, marker="+")
plt.show()

print("Avg. abs. dist.: %f" % np.mean(np.abs(ort_mess-ort_theo)))
print("Avg. squared dist. %f" % np.mean((ort_theo-ort_mess)**2))

print(nearest_point(0, 0))
print(nearest_point(2, 4))
print(nearest_point(1, 3))
print(nearest_point(2, 2))
