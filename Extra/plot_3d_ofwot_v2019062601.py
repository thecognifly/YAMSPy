import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# >>> Create 5 array 
pix = []
piy = []
otx = []
oty = []
otz = []
f= open("data.txt","r")
data = f.readlines()
# print (data[22])
# print (data[22].split(',')[1])
# print (data[22].split(',')[2])
# print (data[22].split(',')[3])
# print (data[22].split(',')[4])
# print (data[22].split(',')[5])

# >>> Read the data and append to the array
for i in range (len(data)):
    # pix = (data[i].split(',')[1])
    # piy = (data[i].split(',')[2])
    # otx = (data[i].split(',')[3][1:])
    # oty = (data[i].split(',')[4])
    # otz = (data[i].split(',')[5][0:-2])

    pix.append((float(data[i].split(',')[1]))*0.1)
    piy.append((float(data[i].split(',')[2]))*0.1)
    otx.append((float(data[i].split(',')[3][1:]))-2)
    oty.append((float(data[i].split(',')[4]))+1)
    otz.append(float(data[i].split(',')[5][0:-2]))

    # print (pix,piy  )

# plt.suptitle('Total amount of data: %d'%(len(data)), fontsize=16)
# plt.plot(pix, piy, 'ro')
# plt.plot(otx, oty,'bo')
# # plt.axis([-10, 10, -10, 10])
# plt.show()

# >>> Plot 3D Graph
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(otx, oty, otz, c='r', marker='o')
ax.scatter(pix, piy, .5, c='b', marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
