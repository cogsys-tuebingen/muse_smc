import numpy
import math

s = 0.0
w = 0.0
# -------------------------------
for i in range(0, 10):
    s += i * 0.5
    w += 0.5
print(s / w)

for i in range(0, 10):
    s += i
    w += 1
print(s / w)

for i in range(0, 10):
    s += 2
    w += 1
print(s / w)
# -------------------------------
for i in range(0, 10):
    s += i * 0.5
    w += 0.5
print(s / w)

for i in range(0, 10):
    s += i
    w += 1
print(s / w)

for i in range(0, 10):
    s += 2
    w += 1
print(s / w)



for i in range(0, 10):
    s += 8 * 0.75
    w += 0.75
print(s / w)