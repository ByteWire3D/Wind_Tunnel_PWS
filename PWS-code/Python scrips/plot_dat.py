import math
import matplotlib.pyplot as plt

number_corrected = 0
threshold = 0.2  # 20% deviation allowed
air_density = 1.225  # kg/m^3 (default at sea level)
reference_area = 80*100 /1000000  # m^2 (example value, adjust to your model)
data = """ """


name = "Bergey BW-3"
skip_speed1 = 8.80
skip_speed2 = 0
angles = {speed: [] for speed in [2.84, 4.83, 6.82, 8.80, 10.79, 12.78]}  # Angle of attack split by airspeed
cl_values = {speed: [] for speed in [2.84, 4.83, 6.82, 8.80, 10.79, 12.78]}
cd_values = {speed: [] for speed in [2.84, 4.83, 6.82, 8.80, 10.79, 12.78]}
clcd_values = {speed: [] for speed in [2.84, 4.83, 6.82, 8.80, 10.79, 12.78]}

drag_data = []
lift_data = []

# Correct abnormal values
def correct_abnormal_value(current_value, previous_value, next_value):
    if previous_value > 0 and next_value > 0:
         if current_value == 0 or (abs(current_value - previous_value) / previous_value > threshold and abs(current_value - next_value) / next_value > threshold):
            return (previous_value + next_value) / 2
    return current_value

# Process raw data
for i, line in enumerate(data.strip().split("\n")):
    if "Data Flag" in line or "Waiting for PID" in line:
        continue
    try:
        columns = line.split(",")
        if len(columns) < 5:
            print(f"continued for Index: {i}.")
            continue
        drag = float(columns[4]) * 9.81 / 1000
        lift = float(columns[3]) * 9.81 / 1000
        #print(i, end=",\t")
        #print(columns[2])
        drag_data.append(drag)
        lift_data.append(lift)
    except ValueError:
        print(f"got a value error for Index: {i}.")
        continue

# Apply offsets
offset_lift = -min(lift_data) if min(lift_data) < 0 else 0
offset_drag = -min(drag_data) if min(drag_data) < 0 else 0

# Correct and smooth data
corrected_drag_data = [drag_data[0]]
corrected_lift_data = [lift_data[0]]
print("drag data len: ", end="\t")
print(len(drag_data))
for i in range(1, len(drag_data) - 1):
    corrected_drag_data.append(correct_abnormal_value(drag_data[i], drag_data[i - 1], drag_data[i + 1]))
    corrected_lift_data.append(correct_abnormal_value(lift_data[i], lift_data[i - 1], lift_data[i + 1]))
corrected_drag_data.append(drag_data[-1])
corrected_lift_data.append(lift_data[-1])

print("corrected data len: ", end="\t")
print(len(corrected_drag_data))
# Calculate coefficients
for i, line in enumerate(data.strip().split("\n")):
    if "Data Flag" in line or "Waiting for PID" in line:
        continue
    try:
        columns = line.split(",")
        pitch = float(columns[5]) * math.pi / 180
        airspeed = float(columns[2])
        lift = corrected_lift_data[i] + offset_lift
        drag = corrected_drag_data[i] + offset_drag +0.01

        cl = 2 * lift / (air_density * airspeed**2 * reference_area * math.cos(pitch))
        cd = 2 * drag / (air_density * airspeed**2 * reference_area * math.cos(pitch))
        if cl > 0 and cd > 0:
            clcd = cl / cd
            cl_values[airspeed].append(cl)
            cd_values[airspeed].append(cd)
            clcd_values[airspeed].append(clcd)
            angles[airspeed].append(pitch * 180 / math.pi)
    except (ValueError, IndexError):
        #print(f"Index {i} is out of range for corrected_lift_data.")
        continue

def moving_average(data, window_size):
    return [sum(data[i:i+window_size]) / window_size for i in range(len(data) - window_size + 1)]

def truncate_list(data, size):
    return data[:size]

size =10
for speed in cl_values:
    cl_values[speed] = moving_average(cl_values[speed], size)

for speed in cd_values:
    cd_values[speed] = moving_average(cd_values[speed], size)

for speed in clcd_values:
    clcd_values[speed] = moving_average(clcd_values[speed], size)

for speed in angles:
    angles[speed] = truncate_list(angles[speed], len(cl_values[speed]))
# Lift Coefficient (Cl) vs Angle of Attack
plt.figure()
for speed, cl in cl_values.items():
    if cl:
        plt.plot(angles[speed], cl, label=f"Cl ({speed} m/s)", marker=",")
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Lift Coefficient (Cl)')
plt.title(f'Lift Coefficient vs Angle of Attack for {name}')
plt.legend()
plt.grid(True)

# Drag Coefficient (Cd) vs Angle of Attack
plt.figure()
for speed, cd in cd_values.items():
    if cd:
     plt.plot(angles[speed], cd, label=f"Cd ({speed} m/s)", marker=",")
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Drag Coefficient (Cd)')
plt.title(f'Drag Coefficient vs Angle of Attack for {name}')
plt.legend()
plt.grid(True)

# Lift-to-Drag Ratio (Cl/Cd) vs Angle of Attack
plt.figure()
for speed, clcd in clcd_values.items():
    if clcd:
     plt.plot(angles[speed], clcd, label=f"Cl/Cd ({speed} m/s)", marker=",")
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Lift-to-Drag Ratio (Cl/Cd)')
plt.title(f'Lift-to-Drag Ratio vs Angle of Attack for {name}')
plt.legend()
plt.grid(True)

# Lift Coefficient (Cl) vs Drag Coefficient (Cd)
plt.figure()
for speed in cl_values.keys():
    if speed != skip_speed1 and speed != skip_speed2:
        plt.plot(cd_values[speed], cl_values[speed], label=f"{speed} m/s", marker=",")
plt.xlabel('Drag Coefficient (Cd)')
plt.ylabel('Lift Coefficient (Cl)')
plt.title(f'Lift Coefficient vs Drag Coefficient for {name}')
plt.legend()
plt.grid(True)

plt.show()
