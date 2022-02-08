import matplotlib.pyplot as plt
import numpy as np

File_object = open(r"roll_control.txt","r")

a = File_object.readlines()
a = a[20:-20] #cut off first and last 20 lines

x = []
pitch = []
pitch_desired = []
i = 0
for row in a:
    num_list = row.split()

    for c, num in enumerate(num_list):
        num = float(num.strip())
        num_list[c] = num
    print(num_list)
    try:
        pitch_desired.append(num_list[0])
        pitch.append(num_list[1])
        x.append(i)
        i += 1
    except:
        break

plt.title("PID Roll Control (17 2 0.05)") #was moving towards 0.05 and it seemed to help with overshoot 
plt.plot(x, pitch, label = "line 1")
plt.plot(x, pitch_desired, label = "line 2")
plt.plot(x, np.zeros(len(x)), label = "zero line", color = 'black')

plt.savefig("roll_control")
plt.show()

