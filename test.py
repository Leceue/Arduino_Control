import math
from math import atan, acos, sqrt, pi as Pi

while(True):
    height = float(input("Enter the height of the building in meters: "))

    theta = 180 * (Pi - atan(height / 18) - acos((height * height - 5976) / (180 * sqrt(324 + height * height))))
    print("theta:", round(theta, 2))
