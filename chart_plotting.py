import matplotlib.pyplot as plt
import numpy as np

File_object = open(r"yaw.txt","r")

a = File_object.readlines()
a = a[20:-20] #cut off first and last 20 lines

x = []
m1 = []
m2 = []
yaw = []
i = 0
for row in a:
    num_list = row.split()

    for c, num in enumerate(num_list):
        num = float(num.strip())
        num_list[c] = num
    print(num_list)
    try:
        m1.append(num_list[0])
        m2.append(num_list[1])
        yaw.append(num_list[2])
        x.append(i)
        i += 1
    except:
        break

plt.title("P Yaw Control (0.5)") #was moving towards 0.05 and it seemed to help with overshoot 
plt.plot(x, m1, label = "motor 1")
plt.plot(x, m2, label = "motor 2")
plt.plot(x, yaw, label = "yaw rate")
plt.plot(x, np.zeros(len(x)), label = "zero line", color = 'black')

plt.savefig("raw_contZrol")
plt.show()

