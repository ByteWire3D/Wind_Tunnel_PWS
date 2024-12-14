import matplotlib.pyplot as yusu

data = """ 

"""


count = 0
prev_pitch = 0

angels_re_2e4 = []
cl_re_2e4 = []
cd_re_2e4 = []
clcd_re_2e4 = []

angels_re_34e3 = []
cl_re_34e3 = []
cd_re_34e3 = []
clcd_re_34e3 = []

angels_re_48e3 = []
cl_re_48e3 = []
cd_re_48e3 = []
clcd_re_48e3 = []

angels_re_62e3 = []
cl_re_62e3 = []
cd_re_62e3 = []
clcd_re_62e3 = []

angels_re_76e3 = []
cl_re_76e3 = []
cd_re_76e3 = []
clcd_re_76e3 = []

angels_re_9e4 = []
cl_re_9e4 = []
cd_re_9e4 = []
clcd_re_9e4 = []

# Loop door de regels van de data
for line in data.strip().split("\n"):
    # Sla regels over die specifieke vlaggen bevatten
    if "----" in line or "alpha" in line  or "mach" in line or "xtrf" in line or "Reynolds" in line or "Calculated" in line or "xflr5" in line:
        continue
    
    try:
        # Splits de regel en haal de lift- en dragwaarden op
        columns = line.split("|")
        if len(columns) < 4 or not columns[1]:
            print(f"Skipping invalid line: {line}")
            continue
        pitch = float(columns[1])  # Angle of attack in degrees
        cl = float(columns[2])
        cd = float(columns[3])

        clcd = cl/cd
        if ((abs(pitch - prev_pitch)) > 10):
           count += 1

        prev_pitch = pitch
        
        if count == 0:
            angels_re_2e4.append(pitch)
            cl_re_2e4.append(cl)
            cd_re_2e4.append(cd)
            clcd_re_2e4.append(clcd)
        elif count == 1:
            angels_re_34e3.append(pitch)
            cl_re_34e3.append(cl)
            cd_re_34e3.append(cd)
            clcd_re_34e3.append(clcd)
        elif count == 2:
            angels_re_48e3.append(pitch)
            cl_re_48e3.append(cl)
            cd_re_48e3.append(cd)
            clcd_re_48e3.append(clcd)
        elif count == 3:
            angels_re_62e3.append(pitch)
            cl_re_62e3.append(cl)
            cd_re_62e3.append(cd)
            clcd_re_62e3.append(clcd)
        elif count == 4:
            angels_re_76e3.append(pitch)
            cl_re_76e3.append(cl)
            cd_re_76e3.append(cd)
            clcd_re_76e3.append(clcd)
        elif count == 5:
            angels_re_9e4.append(pitch)
            cl_re_9e4.append(cl)
            cd_re_9e4.append(cd)
            clcd_re_9e4.append(clcd)
 

    except (ValueError, IndexError):
        # Verwerk eventuele parsingfouten (bijvoorbeeld ontbrekende of corrupte data)
        continue

def moving_average(data, window_size=5):
    return [sum(data[i:i+window_size]) / window_size for i in range(len(data) - window_size + 1)]

#cl_smoothed = moving_average(cl_values, window_size=25)
#cd_smoothed = moving_average(cd_values, window_size=25)

def truncate_list(data, size):
    return data[:size]

# Calculate moving averages
size = 3
cl_re_2e4 = moving_average(cl_re_2e4, window_size=size)
cd_re_2e4 = moving_average(cd_re_2e4, window_size=size)
clcd_re_2e4 = moving_average(clcd_re_2e4, window_size=size)

# Truncate angels_re_2e4 to match the length of smoothed data
angels_re_2e4 = truncate_list(angels_re_2e4, len(cl_re_2e4))

# Repeat for other Reynolds numbers
cl_re_34e3 = moving_average(cl_re_34e3, window_size=size)
cd_re_34e3 = moving_average(cd_re_34e3, window_size=size)
clcd_re_34e3 = moving_average(clcd_re_34e3, window_size=size)
angels_re_34e3 = truncate_list(angels_re_34e3, len(cl_re_34e3))

cl_re_48e3 = moving_average(cl_re_48e3, window_size=size)
cd_re_48e3 = moving_average(cd_re_48e3, window_size=size)
clcd_re_48e3 = moving_average(clcd_re_48e3, window_size=size)
angels_re_48e3 = truncate_list(angels_re_48e3, len(cl_re_48e3))

cl_re_62e3 = moving_average(cl_re_62e3, window_size=size)
cd_re_62e3 = moving_average(cd_re_62e3, window_size=size)
clcd_re_62e3 = moving_average(clcd_re_62e3, window_size=size)
angels_re_62e3 = truncate_list(angels_re_62e3, len(cl_re_62e3))

cl_re_76e3 = moving_average(cl_re_76e3, window_size=size)
cd_re_76e3 = moving_average(cd_re_76e3, window_size=size)
clcd_re_76e3 = moving_average(clcd_re_76e3, window_size=size)
angels_re_76e3 = truncate_list(angels_re_76e3, len(cl_re_76e3))

cl_re_9e4 = moving_average(cl_re_9e4, window_size=size)
cd_re_9e4 = moving_average(cd_re_9e4, window_size=size)
clcd_re_9e4 = moving_average(clcd_re_9e4, window_size=size)
angels_re_9e4 = truncate_list(angels_re_9e4, len(cl_re_9e4))

#print("2e4")
#for i in angels_re_2e4:
#    print(i)
#
#print("34e3")
#for i in angels_re_34e3:
#    print(i)
#
#print("48e3")
#for i in angels_re_48e3:
#    print(i)
#
#print("62e3")
#for i in angels_re_62e3:
#    print(i)
#
#print("76e3")
#for i in angels_re_76e3:
#    print(i)
#
#print("9e4")
#for i in angels_re_9e4:
#    print(i)

# Plot Cl vs Angle of Attack (alpha) for (airfiol)
yusu.figure()
yusu.plot(angels_re_2e4, cl_re_2e4, label='Cl (Re 2e4)', marker=',')
yusu.plot(angels_re_34e3, cl_re_34e3, label='Cl (Re 34e3)', marker=',')
yusu.plot(angels_re_48e3, cl_re_48e3, label='Cl (Re 48e3)', marker=',')
yusu.plot(angels_re_62e3, cl_re_62e3, label='Cl (Re 62e3)', marker=',')
yusu.plot(angels_re_76e3, cl_re_76e3, label='Cl (Re 76e3)', marker=',')
yusu.plot(angels_re_9e4, cl_re_9e4, label='Cl (Re 9e4)', marker=',')
yusu.xlabel('Angle of Attack (degrees)')
yusu.ylabel('Lift Coefficient (Cl)')
yusu.title('Lift Coefficient vs Angle of Attack for (airfiol)')
yusu.grid(True)
yusu.legend()

# Plot C_D vs Angle of Attack (alpha) for (airfiol)
yusu.figure()
yusu.plot(angels_re_2e4, cd_re_2e4, label='Cd (Re 2e4)', marker=',')
yusu.plot(angels_re_34e3, cd_re_34e3, label='Cd (Re 34e3)', marker=',')
yusu.plot(angels_re_48e3, cd_re_48e3, label='Cd (Re 48e3)', marker=',')
yusu.plot(angels_re_62e3, cd_re_62e3, label='Cd (Re 62e3)', marker=',')
yusu.plot(angels_re_76e3, cd_re_76e3, label='Cd (Re 76e3)', marker=',')
yusu.plot(angels_re_9e4, cd_re_9e4, label='Cd (Re 9e4)', marker=',')
yusu.xlabel('Angle of Attack (degrees)')
yusu.ylabel('Drag Coefficient (Cd)')
yusu.title('Drag Coefficient vs Angle of Attack for (airfiol)')
yusu.grid(True)
yusu.legend()


# Plot Cl/Cd vs Angle of Attack (alpha) for (airfiol)
yusu.figure()
yusu.plot(angels_re_2e4, clcd_re_2e4, label='Cl/Cd (Re 2e4)', marker=',')
yusu.plot(angels_re_34e3, clcd_re_34e3, label='Cl/Cd (Re 34e3)', marker=',')
yusu.plot(angels_re_48e3, clcd_re_48e3, label='Cl/Cd (Re 48e3)', marker=',')
yusu.plot(angels_re_62e3, clcd_re_62e3, label='Cl/Cd (Re 62e3)', marker=',')
yusu.plot(angels_re_76e3, clcd_re_76e3, label='Cl/Cd (Re 76e3)', marker=',')
yusu.plot(angels_re_9e4, clcd_re_9e4, label='Cl/Cd (Re 9e4)', marker=',')
yusu.xlabel('Angle of Attack (degrees)')
yusu.ylabel('Lift-to-Drag Ratio (Cl/Cd)')
yusu.title('Cl/Cd vs Angle of Attack for (airfiol)')
yusu.grid(True)
yusu.legend()


# Plot Cl vs Cd for (airfiol)
yusu.figure()
yusu.plot(cd_re_2e4, cl_re_2e4, label='Cl vs Cd (Re 2e4)', marker=',')
yusu.plot( cd_re_34e3, cl_re_34e3, label='Cl vs Cd (Re 34e3)', marker=',')
yusu.plot( cd_re_48e3, cl_re_48e3, label='Cl vs Cd (Re 48e3)', marker=',')
yusu.plot( cd_re_62e3, cl_re_62e3, label='Cl vs Cd (Re 62e3)', marker=',')
yusu.plot( cd_re_76e3, cl_re_76e3, label='Cl vs Cd (Re 76e3)', marker=',')
yusu.plot(cd_re_9e4, cl_re_9e4, label='Cl vs Cd (Re 9e4)', marker=',')
yusu.xlabel('Drag Coefficient (Cd)')
yusu.ylabel('Lift Coefficient (Cl)')
yusu.title('Cl vs Cd for (airfiol)')
yusu.grid(True)
yusu.legend()

yusu.show()

