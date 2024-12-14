import matplotlib.pyplot as plt

# Constants (modify these based on your setup)
air_density = 1.225  # kg/m^3 (at sea level)
reference_area = 80*100 /1000000  # m^2 (example value, adjust to your model)

data = """ 

"""
number_corrected = 0

angles = []  # Angle of attack (pitch)

cl_values_284 = []  # Lift coefficient
cd_values_284 = []  # Drag coefficient
clcd_values_284 = [] #lift co / drag co 

cl_values_483 = []  # Lift coefficient
cd_values_483 = []  # Drag coefficient
clcd_values_483 = [] #lift co / drag co 

cl_values_682 = []  # Lift coefficient
cd_values_682 = []  # Drag coefficient
clcd_values_682 = [] #lift co / drag co 

cl_values_880 = []  # Lift coefficient
cd_values_880 = []  # Drag coefficient
clcd_values_880 = [] #lift co / drag co 

cl_values_1079 = []  # Lift coefficient
cd_values_1079 = []  # Drag coefficient
clcd_values_1079 = [] #lift co / drag co 

cl_values_1278 = []  # Lift coefficient
cd_values_1278 = []  # Drag coefficient
clcd_values_1278 = [] #lift co / drag co 

drag_data = []
lift_data = []
for line in data.strip().split("\n"):
    # Skip lines that contain specific flags
    if "Data Flag" in line or "Waiting for PID" in line:
        continue
    
    try: 
        # Split the line by commas and extract the drag value
        columns = line.split(",")
        drag = float(columns[4]) * 9.81 * 0.0001  # Drag force in N (converted)
        lift = float(columns[3]) * 9.81 * 0.0001
        # Store the drag value in the list
        drag_data.append(drag)
        lift_data.append(lift)
    
    except ValueError:
        # Handle any parsing errors (e.g., missing or corrupted data)
        continue

# Step 1: Find the smallest value in the drag data
min_val = min(drag_data)

# Step 2: Calculate the offset (inverse of the smallest value)
offset = -min_val + 0.01

# Step 3: Apply the offset to the entire drag data list
#corrected_drag_data = [drag + offset for drag in drag_data]



drag_dat = []
lift_dat = []

# Loop door de regels van de data
for line in data.strip().split("\n"):
    # Sla regels over die specifieke vlaggen bevatten
    if "Data Flag" in line or "Waiting for PID" in line:
        continue
    
    try:
        # Splits de regel en haal de lift- en dragwaarden op
        columns = line.split(",")
        drag = float(columns[4]) * 9.81 * 0.0001 + offset # Drag force in N (omgezet)
        lift = float(columns[3]) * 9.81 * 0.0001  # Lift force in N (omgezet)
        
        # Voeg de waarden toe aan de respectieve lijsten
        drag_dat.append(drag)
        lift_dat.append(lift)
    
    except ValueError:
        # Verwerk eventuele parsingfouten (bijvoorbeeld ontbrekende of corrupte data)
        continue

# Stap 1: Vind de minimale en maximale waarden voor drag en lift
min_drag = min(drag_dat)
max_drag = max(drag_dat)
min_lift = min(lift_dat)
max_lift = max(lift_dat)

# Stap 2: Controleer of de afwijkingen te groot zijn (bijvoorbeeld > 20% van het gemiddelde)
threshold = 1  # 20% afwijking

# Functie om te controleren of de afwijking te groot is en de waarde te corrigeren
def correct_abnormal_value(current_value, previous_value, next_value):
    # Bereken de gemiddelde verandering tussen de vorige en volgende meting
    if previous_value >= 0 and next_value >= 0:
        avg_growth = (next_value - previous_value) / 2
        corrected_value =  avg_growth
        return corrected_value
    return current_value

# Stap 3: Corrigeer de waarden als de afwijking te groot is
corrected_drag_data = []
corrected_lift_data = []

for i in range(1, len(drag_dat) - 1):
    drag_value = drag_dat[i]
    lift_value = lift_dat[i] 
    
    # Vergelijk met vorige en volgende waarden
    if abs(drag_value - drag_dat[i-1]) > threshold * abs(drag_dat[i-1]) and abs(drag_value - drag_dat[i+1]) > threshold * abs(drag_dat[i+1]):
        drag_value = correct_abnormal_value(drag_value, drag_dat[i-1], drag_dat[i+1])
        number_corrected += 1
    
    if abs(lift_value - lift_dat[i-1]) > threshold * abs(lift_dat[i-1]) and abs(lift_value - lift_dat[i+1]) > threshold * abs(lift_dat[i+1]):
        lift_value = correct_abnormal_value(lift_value, lift_dat[i-1], lift_dat[i+1])
        number_corrected += 1
    
    corrected_drag_data.append(drag_value)
    corrected_lift_data.append(lift_value)

# Voeg de eerste en laatste waarden zonder wijziging toe (omdat ze geen beide buren hebben)
corrected_drag_data = [drag_dat[0]] + corrected_drag_data + [drag_dat[-1]]
corrected_lift_data = [lift_dat[0]] + corrected_lift_data + [lift_dat[-1]]

# Initialize a counter to iterate through corrected data
corrected_index = 0

for line in data.strip().split("\n"):
    if "Data Flag" in line or "Waiting for PID" in line:
        continue

    try:

        columns = line.split(",")
        airspeed = float(columns[2])
        pitch = float(columns[5])  # Angle of attack in degrees
        
        # Use corrected lift and drag values
        lift = corrected_lift_data[corrected_index] 
        drag = corrected_drag_data[corrected_index] 

        corrected_index += 1  # Increment the index to match corrected data

        # Compute coefficients
        cl = 2 * lift / (air_density * airspeed**2 * reference_area)
        cd = 2 * drag / (air_density * airspeed**2 * reference_area)

        # Handle fallback for `cl` and `cd` values
        target_list = cl_values_284 if airspeed == 2.84 else \
                      cl_values_483 if airspeed == 4.83 else \
                      cl_values_682 if airspeed == 6.82 else \
                      cl_values_880 if airspeed == 8.80 else \
                      cl_values_1079 if airspeed == 10.79 else \
                      cl_values_1278 if airspeed == 12.78 else []
        if cl == 0 or cl <= 0.1:
       # if abs(cl - target_list[corrected_index - 1]) > threshold * abs(target_list[corrected_index - 1]) and abs(cl - target_list[corrected_index -2]) > threshold * abs(target_list[corrected_index -2]) or cl == 0:
            if len(target_list) >= 2:
                cl = target_list[-1] + (target_list[-2] - target_list[-1])
        
        target_list = cd_values_284 if airspeed == 2.84 else \
                      cd_values_483 if airspeed == 4.83 else \
                      cd_values_682 if airspeed == 6.82 else \
                      cd_values_880 if airspeed == 8.80 else \
                      cd_values_1079 if airspeed == 10.79 else \
                      cd_values_1278 if airspeed == 12.78 else []
        if cd == 0 or cd <= 0.1:
        #if abs(cd - target_list[corrected_index - 1]) > threshold * abs(target_list[corrected_index - 1]) and abs(cd - target_list[corrected_index - 2]) > threshold * abs(target_list[corrected_index -2]) or cd == 0:
            if len(target_list) >= 2:
                cd = target_list[-1] + (target_list[-2] - target_list[-1])

        if cl != 0 and cd != 0:
            cl_cd = cl / cd

        # Append data
        if airspeed == 2.84:
            angles.append(pitch)
            cl_values_284.append(cl)
            cd_values_284.append(cd)
            clcd_values_284.append(cl_cd)
        elif airspeed == 4.83:
            angles.append(pitch)
            cl_values_483.append(cl)
            cd_values_483.append(cd)
            clcd_values_483.append(cl_cd)
        elif airspeed == 6.82:
            angles.append(pitch)
            cl_values_682.append(cl)
            cd_values_682.append(cd)
            clcd_values_682.append(cl_cd)
        elif airspeed == 8.80:
            angles.append(pitch)
            cl_values_880.append(cl)
            cd_values_880.append(cd)
            clcd_values_880.append(cl_cd)
        elif airspeed == 10.79:
            angles.append(pitch)
            cl_values_1079.append(cl)
            cd_values_1079.append(cd)
            clcd_values_1079.append(cl_cd)
        elif airspeed == 12.78:
            angles.append(pitch)
            cl_values_1278.append(cl)
            cd_values_1278.append(cd)
            clcd_values_1278.append(cl_cd)
    except (ValueError, IndexError):
        continue




# Plotting
# Plot Cl vs Angle of Attack (alpha) for different airspeeds
plt.figure()
plt.plot(angles[:len(cl_values_284)], cl_values_284, label='Cl (2.84 m/s)', marker=',')
plt.plot(angles[:len(cl_values_483)], cl_values_483, label='Cl (4.83 m/s)', marker=',')
plt.plot(angles[:len(cl_values_682)], cl_values_682, label='Cl (6.82 m/s)', marker=',')
plt.plot(angles[:len(cl_values_880)], cl_values_880, label='Cl (8.80 m/s)', marker=',')
plt.plot(angles[:len(cl_values_1079)], cl_values_1079, label='Cl (10.79 m/s)', marker=',')
plt.plot(angles[:len(cl_values_1278)], cl_values_1278, label='Cl (12.78 m/s)', marker=',')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Lift Coefficient (Cl)')
plt.title('Lift Coefficient vs Angle of Attack for Different Airspeeds')
plt.grid(True)
plt.legend()

# Plot C_D vs Angle of Attack (alpha) for different airspeeds
plt.figure()
plt.plot(angles[:len(cd_values_284)], cd_values_284, label='Cd (2.84 m/s)', marker=',')
plt.plot(angles[:len(cd_values_483)], cd_values_483, label='Cd (4.83 m/s)', marker=',')
plt.plot(angles[:len(cd_values_682)], cd_values_682, label='Cd (6.82 m/s)', marker=',')
plt.plot(angles[:len(cd_values_880)], cd_values_880, label='Cd (8.80 m/s)', marker=',')
plt.plot(angles[:len(cd_values_1079)], cd_values_1079, label='Cd (10.79 m/s)', marker=',')
plt.plot(angles[:len(cd_values_1278)], cd_values_1278, label='Cd (12.78 m/s)', marker=',')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Drag Coefficient (Cd)')
plt.title('Drag Coefficient vs Angle of Attack for Different Airspeeds')
plt.grid(True)
plt.legend()


# Plot Cl/Cd vs Angle of Attack (alpha) for different airspeeds
plt.figure()
plt.plot(angles[:len(clcd_values_284)], clcd_values_284, label='Cl/Cd (2.84 m/s)', marker=',')
plt.plot(angles[:len(clcd_values_483)], clcd_values_483, label='Cl/Cd (4.83 m/s)', marker=',')
plt.plot(angles[:len(clcd_values_682)], clcd_values_682, label='Cl/Cd (6.82 m/s)', marker=',')
plt.plot(angles[:len(clcd_values_880)], clcd_values_880, label='Cl/Cd (8.80 m/s)', marker=',')
plt.plot(angles[:len(clcd_values_1079)], clcd_values_1079, label='Cl/Cd (10.79 m/s)', marker=',')
plt.plot(angles[:len(clcd_values_1278)], clcd_values_1278, label='Cl/Cd (12.78 m/s)', marker=',')
plt.xlabel('Angle of Attack (degrees)')
plt.ylabel('Lift-to-Drag Ratio (Cl/Cd)')
plt.title('Cl/Cd vs Angle of Attack for Different Airspeeds')
plt.grid(True)
plt.legend()


# Plot Cl vs Cd for different airspeeds
plt.figure()
plt.plot(cd_values_284, cl_values_284, label='Cl vs Cd (2.84 m/s)', marker='.')
plt.plot(cd_values_483, cl_values_483, label='Cl vs Cd (4.83 m/s)', marker='.')
plt.plot(cd_values_682, cl_values_682, label='Cl vs Cd (6.82 m/s)', marker='.')
plt.plot(cd_values_880, cl_values_880, label='Cl vs Cd (8.80 m/s)', marker='.')
plt.plot(cd_values_1079, cl_values_1079, label='Cl vs Cd (10.79 m/s)', marker='.')
plt.plot(cd_values_1278, cl_values_1278, label='Cl vs Cd (12.78 m/s)', marker='.')
plt.xlabel('Drag Coefficient (Cd)')
plt.ylabel('Lift Coefficient (Cl)')
plt.title('Cl vs Cd for Different Airspeeds')
plt.grid(True)
plt.legend()

print(number_corrected)
plt.show()
