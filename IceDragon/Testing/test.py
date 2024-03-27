import IceDragon.windData as windData
import numpy as np
import re

def clean_data1():
    with open("IceDragon/AntSoundingData.txt", 'r') as file:
        # Read lines from the input file
        lines = file.readlines()
        # Filter lines containing only numerical data
        numerical_lines = [line.strip() for line in lines if all(char.isdigit() or char in {'-', '.', ' '} for char in line.strip())]
    with open("IceDragon/filtered_sounding.txt", 'w') as file:
        # Write the filtered numerical data back to the file
        file.write('\n'.join(numerical_lines))


def clean_data2():
    with open("IceDragon/filtered_sounding.txt", 'r') as file:
        # Read lines from the input file
        lines = file.readlines()
    with open("IceDragon/filtered_sounding.txt", 'w') as file:
        for line in lines:
            if '-------------------------------------------------------------------------------------------' not in line:
                file.write(line)

def cut_data():
    with open("IceDragon/filtered_sounding.txt", 'r') as file:
        lines = file.readlines()
    pattern = '1000'
    found_pattern = False
    for i, line in enumerate(lines):
        if i == 0:
            continue
        if line.startswith(pattern):
            found_pattern = True
            break

    if found_pattern:
        lines = lines[:i]
        with open("IceDragon/filtered_sounding.txt", 'w') as file:
            file.writelines(lines)

            
clean_data1()
clean_data2()
#cut_data()


def get_sounding_data(file_path, current_alt):
    '''
    returns sounding data at current altitude

    :param alt: current altitude of vehicle
    '''
    columns = [[] for _ in range(13)] # columns of data in sounding file
    rows = []

    with open(file_path, 'r') as file:
        for line in file:
            data = re.split(r'\s+', line.strip())
            for i in range(len(data)):
                columns[i].append(data[i])

        pressure = np.array(columns[0], dtype=float)
        height = np.array(columns[1], dtype=float)
        temp = np.array(columns[2], dtype=float)
        direction = np.array(columns[8], dtype=float)
        speed = np.array(columns[9], dtype=float) 

        for i in range(len(pressure)):
            rows[i] = windData(pressure[i], height[i], direction[i], speed[i], temp[i])

    return rows

def get_SoundingAtAlt(alt):
    '''
    returns sounding data at current altitude

    :param alt: current altitude of vehicle
    '''
    # Initialize Lists
    pressure = []
    height = []
    direction = []
    speed = []
    temp = []
    columns = [[] for _ in range(13)]

    with open("IceDragon/filtered_sounding.txt", 'r') as file:
        for line in file:
            data = re.split(r'\s+', line.strip())
            for i in range(len(data)):
                columns[i].append(data[i])

        pressure = np.array(columns[0], dtype=float)
        height = np.array(columns[1], dtype=float)
        temp = np.array(columns[2], dtype=float)
        direction = np.array(columns[8], dtype=float)
        speed = np.array(columns[9], dtype=float)
            

    # Get closest data to current altitude
    #print(pressure)
    closest = min(height, key=lambda x: abs(x - alt))
    #print(closest)
    idx = np.where(height == closest)
    idx_sh = idx + 1
    #print(idx)
    #data = windData(pressure[idx], height[idx], direction[idx], speed[idx], temp[idx])

    return pressure[idx], height[idx], direction[idx_sh], speed[idx_sh], temp[idx_sh]

def get_sounding_data(alt):
    '''
    returns sounding data at current altitude

    :param alt: current altitude of vehicle
    '''
    # Initialize Lists
    pressure = []
    height = []
    direction = []
    speed = []
    temp = []

    # Read Sounding File
    with open ("IceDragon/filtered_sounding.txt", "r") as f:
        
        # WILL NEED TO CHECK FORMAT OF NEW SOUNDING FILE!!
        next(f)
        winds_aloft = f.read().split('\n')
        for i in winds_aloft:
            array = i.split("\t")
            #Handling NaN value in dataset
            if len(array) < 5:
                continue
            pressure.append(float(array[0]))
            height.append(float(array[1]))
            direction.append(float(array[2]))
            speed.append(float(array[3]))
            temp.append(float(array[4]))

    pressure = np.array(pressure)
    height = np.array(height)
    direction = np.array(direction)
    speed = np.array(speed)
    temp = np.array(temp)

    # Get closest data to current altitude
    closest = min(height, key=lambda x: abs(x - alt))
    idx = height.index(closest)
    print(pressure[idx])
    data = windData(pressure[idx], height[idx], direction[idx], speed[idx], temp[idx])

    return data

#pres, alt, direc, speed, temp = get_SoundingAtAlt(1262)
#print(pres, alt, direc, speed, temp)
#sound_data = get_SoundingAtAlt(1260)
#rows = get_sounding_data("IceDragon/filtered_sounding.txt", 1260)
#for i, column in enumerate(columns, start=1):
    #print(f"Column {i}:", column)
