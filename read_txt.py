import numpy as np

#file1 = open('data.txt', 'r')

def format_lidar_data(file_place):
    file1 = open(file_place, 'r')
    Lines = file1.readlines()

    x_values = []
    y_values = []
    z_values = []

    list_xyz = []  
    # Strips the newline character
    for line in Lines:
        t = line.split(':')
        tlist_without_spaces = [s.strip() for s in t]
        if 'x' in tlist_without_spaces:
            x_values.append(float(t[1]))
        if 'y' in tlist_without_spaces:
            y_values.append( float(t[1]))
        if 'z' in tlist_without_spaces:
            z_values.append( float(t[1]))
    assert len(x_values) == len(y_values) == len(z_values), "length of lists of x,y,z is not equal"
    for i in range(len(x_values)):
        list_xyz.append([ x_values[i],y_values[i],z_values[i]] )
    assert len(list_xyz) == 35, "you must select 35 points in the lidar, considering the vertices of chessboard are 35 "
        
    return np.array(list_xyz)
#print(format_lidar_data())