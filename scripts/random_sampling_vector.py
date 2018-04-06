import random
import math


size = 10
v = math.pow(random.random(), 1.0 / size)
u = [v for i in range(size)]

print(u)
for k in range(size - 1, 0, -1):
    v = math.pow(random.random(), 1.0 / k)
    u[k-1] = u[k] * v

print(u)