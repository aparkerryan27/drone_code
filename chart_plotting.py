import matplotlib.pyplot as plt
import numpy as np

File_object = open(r"pid_out.txt","r")

a = File_object.readlines()



x = []
gyro_accel = []
gyro_filter = []
pwm1 = []
pwm2 = []
i = 0
for row in a:
    num_list = row.split()

    for c, num in enumerate(num_list):
        num = float(num.strip())
        num_list[c] = num
    print(num_list)
    try:
        gyro_accel.append(num_list[0]*100)
        gyro_filter.append(num_list[1]*100)
        pwm1.append(num_list[2])
        pwm2.append(num_list[3])
        x.append(i)
        i += 1
    except:
        break

plt.title("PID Control")
plt.plot(x, gyro_filter, label = "line 2")
plt.plot(x, gyro_accel, label = "line 1")

plt.plot(x, pwm2, label = "line 4")
plt.plot(x, pwm1, label = "line 3")
plt.plot(x, np.zeros(len(x)), label = "zero line", color = 'black')

plt.savefig("pid_control")
plt.show()

