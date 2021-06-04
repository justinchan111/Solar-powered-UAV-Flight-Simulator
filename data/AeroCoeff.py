import csv
import numpy as np

alpha = []
Cd = []
Cl = []

# Extract polar data (NACA 0012 airfoil used by default)
with open(r"C:\Users\justi\Documents\Final year project\Modelling\data\NACA6412Re50000.csv") as file:
    reader = csv.reader(file)
    for row in reader:
        alpha.append(float(row[0]))
        Cl.append(float(row[1]))
        Cd.append(float(row[2]))


# Output the lift and drag co-efficient value for a given angle of attack
def get_ClCd(a):
    if a % 0.25 != 0 or abs(a) > abs(alpha[-1]):
        exit(
            f'Error: alpha must be <= {abs(alpha[-1])} and be a multiple of 0.25')
    n = alpha.index(a)
    n = int(n)
    return Cl[n], Cd[n]
