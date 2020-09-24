#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import pyplot
from math import atan2, pi


def read_pgm(pgm_file):
    """Return a raster of integers from a PGM as a list of lists."""
    pgmf = open(pgm_file)
    
    assert pgmf.readline() == 'P5\n'
    
    while True:
        line = pgmf.readline()
        if line[0] != '#':
            break

    (width, height) = [int(i) for i in line.split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    return raster

data = read_pgm('/home/ivan/catkin_ws/src/f1tenth_gym_ros/map.pgm')


def find_nearest_track_edges(start):
    visited = np.zeros([600, 600])
    edge1, visited = find_nearest_track_edge(start, visited)
    edge2, visited = find_nearest_track_edge(start, visited)
    return edge1, edge2

def find_nearest_track_edge(start, visited):
    queue = []
    visited[start[0]][start[1]] = 1
    queue.append(start)
    
    while queue:
        s = queue.pop(0)

        if mapa[s[0]][s[1]] == 0: # occupied
            visited = mark_edge_as_visited(s)
            return s, visited
        
        if visited[ s[0]+1 ][ s[1] ] == 0:
            queue.append([s[0]+1, s[1]])
            visited[s[0]+1][s[1]] = 1
        if visited[ s[0] ][ s[1]+1 ] == 0:
            queue.append([s[0], s[1]+1])
            visited[s[0]+1][s[1]+1] = 1
        if visited[ s[0]-1 ][ s[1] ] == 0:
            queue.append([s[0]-1, s[1]])
            visited[s[0]-1][s[1]] = 1
        if visited[ s[0] ][ s[1]-1 ] == 0:
            queue.append([s[0], s[1]-1])
            visited[s[0]][s[1]-1] = 1

    return None, visited

def mark_edge_as_visited(start):
    visited = np.zeros([600, 600])
    queue = [start]
    visited[start[0]][start[1]] = 1

    while queue:
        s = queue.pop(0)

        if mapa[s[0]+1][s[1]] == 254 or mapa[s[0]][s[1]+1] == 254 or \
           mapa[s[0]-1][s[1]] == 254 or mapa[s[0]][s[1]-1] == 254:  #free
           visited[s[0]][s[1]] = 1
        else:
            continue

        if mapa[s[0]+1][s[1]] == 0 and visited[s[0]+1][s[1]] == 0: # occupied
            queue.append([s[0]+1, s[1]])
            visited[s[0]+1][s[1]] = 1
        if mapa[s[0]][s[1]+1] == 0 and visited[s[0]][s[1]+1] == 0: # occupied
            queue.append([s[0], s[1]+1])
            visited[s[0]][s[1]+1] = 1
        if mapa[s[0]-1][s[1]] == 0 and visited[s[0]-1][s[1]] == 0: # occupied
            queue.append([s[0]-1, s[1]])
            visited[s[0]-1][s[1]] = 1
        if mapa[s[0]][s[1]-1] == 0 and visited[s[0]][s[1]-1] == 0: # occupied
            queue.append([s[0], s[1]-1])
            visited[s[0]][s[1]-1] = 1

        if mapa[s[0]+1][s[1]+1] == 0 and visited[s[0]+1][s[1]+1] == 0: # occupied
            queue.append([s[0]+1, s[1]+1])
            visited[s[0]+1][s[1]+1] = 1
        if mapa[s[0]+1][s[1]-1] == 0 and visited[s[0]+1][s[1]-1] == 0: # occupied
            queue.append([s[0]+1, s[1]-1])
            visited[s[0]+1][s[1]-1] = 1
        if mapa[s[0]-1][s[1]+1] == 0 and visited[s[0]-1][s[1]+1] == 0: # occupied
            queue.append([s[0]-1, s[1]+1])
            visited[s[0]-1][s[1]+1] = 1
        if mapa[s[0]-1][s[1]-1] == 0 and visited[s[0]-1][s[1]-1] == 0: # occupied
            queue.append([s[0]-1, s[1]-1])
            visited[s[0]-1][s[1]-1] = 1

    return visited

def build_wall(edge1, edge2, mapa):
    if edge1 == edge2:
        return mapa

    mapa[edge1[0]][edge1[1]] = 0 # occupied
    direction = atan2(edge2[1] - edge1[1], edge2[0] - edge1[0])
    direction = direction * 180/pi
    if direction >= -45 and direction < 45:
        return build_wall([edge1[0]+1, edge1[1]], edge2, mapa)
    if direction >= 45 and direction < 115:
        return build_wall([edge1[0], edge1[1]+1], edge2, mapa)
    if direction >= 115 or direction < -115:
        return build_wall([edge1[0]-1, edge1[1]], edge2, mapa)
    if direction >= -115 and direction < -45:
        return build_wall([edge1[0], edge1[1]-1], edge2, mapa)

        
mapa = np.array(data)
#mapa[80][200] = 0  pocetna tocka

edge1, edge2 = find_nearest_track_edges([80, 200])
mapa = build_wall(edge1, edge2, mapa)

plt.imshow(mapa, pyplot.cm.gray)
plt.show()