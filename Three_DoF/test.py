from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
theta = np.linspace(-4 * np.pi, 4 * np.pi, 100)

t = np.linspace(-2, 2, 100)
z = np.exp(t)
r = 1
x = r * np.sin(theta)
y = r * np.cos(theta)

#1 colored by value of `z`
# ax.scatter(x, y, z, c = plt.cm.jet(z/max(z))) 

#2 colored by index (same in this example since z is a linspace too)

for i in range(len(x)-1):
    ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=plt.cm.Blues(round(255 * t[i] / max(t))))

ax2 = fig.add_subplot()

plt.show()