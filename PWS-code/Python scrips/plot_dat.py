import matplotlib.pyplot as plt

# Constants (modify these based on your setup)
air_density = 1.225  # kg/m^3 (at sea level)
reference_area = 80*100 /1000000  # m^2 (example value, adjust to your model)

data = """ """

angles = []  # Angle of attack (pitch)
cl_values = []  # Lift coefficient
cd_values = []  # Drag coefficient

for line in data.strip().split("\n"):
    # Skip the header or non-data rows
    if "Data Flag" in line or "Waiting for PID" in line:
        continue
    
    # Split the line into columns
    try:
        columns = line.split(",")
        airspeed = float(colums[2])
        pitch = float(columns[5])  # Angle of attack in degrees
        lift = float(columns[3])  # Lift force in N
        drag = float(columns[4])  # Drag force in N
        
        lift = lift * 9.81 *0.0001
        drag = drag *9.81 *0.0001
        # Compute coefficients
        cl = 2 * lift / (air_density * airspeed**2 * reference_area)
        cd = 2 * drag / (air_density * airspeed**2 * reference_area)
        
        # Store values
        angles.append(pitch)
        cl_values.append(cl)
        cd_values.append(cd)
    except (ValueError, IndexError):
        # Skip lines with missing or invalid data
        continue

# Plotting
# 1. C_L vs Angle of Attack
plt.figure()
plt.plot(angles, cl_values, label='C_L', marker='o')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Lift Coefficient (C_L)')
plt.title('Lift Coefficient vs Angle of Attack')
plt.grid(True)
plt.legend()

# 2. C_D vs Angle of Attack
plt.figure()
plt.plot(angles, cd_values, label='C_D', marker='s', color='orange')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Drag Coefficient (C_D)')
plt.title('Drag Coefficient vs Angle of Attack')
plt.grid(True)
plt.legend()

# 3. Polar Plot (C_L vs C_D)
plt.figure()
plt.plot(cd_values, cl_values, label='C_L vs C_D', marker='x', color='green')
plt.xlabel('Drag Coefficient (C_D)')
plt.ylabel('Lift Coefficient (C_L)')
plt.title('Aerodynamic Polar: C_L vs C_D')
plt.grid(True)
plt.legend()

plt.show()
