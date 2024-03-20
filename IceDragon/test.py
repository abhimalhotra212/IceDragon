import windDataObject as windData
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
    with open("IceDragon/usable_sounding.txt", 'w') as file:
        for line in lines:
            if '-------------------------------------------------------------------------------------------' not in line:
                file.write(line)
            
clean_data1()
clean_data2()


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

    return columns, rows


columns, rows = get_sounding_data("IceDragon/usable_sounding.txt", 1260)
#for i, column in enumerate(columns, start=1):
    #print(f"Column {i}:", column)
