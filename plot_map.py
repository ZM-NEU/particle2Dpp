import re
import numpy as np
import Image

def get_map_values(map_lines):
    search = re.compile('[a-zA-Z]')
    counter = 0
    map_array = np.empty((800, 800))
    for (num_line, line) in enumerate(map_lines):
        if not search.findall(line) and line.strip():
            map_array[counter] = np.array(map(float, line.split()))
            counter += 1
    return map_array.clip(min=0)

def get_particle_locations(particle_lines):
    return np.loadtxt(open(particle_lines,"rb"),delimiter=",",skiprows=1)
    #return np.genfromtxt(particle_lines, delimiter=',', names=True)

if __name__ == '__main__':
    # Read in map
    _map = open('data/map/wean.dat', 'r')
    map_lines = _map.readlines()
    map_array = get_map_values(map_lines)
    _map.close()
    
    # Read in particle get_particle_locations
    _part = "particles.csv"
    part_array = get_particle_locations(_part)
    numP = part_array[1,:].size/2;

    # Plot map
    threshold = 0.99
    map_threshold = threshold*np.ones([map_array[:,1].size,map_array[1,:].size])
    map_array = 1*np.greater_equal(map_array,map_threshold)
    
    img = Image.fromarray(np.uint8(map_array*255.999))
    img.show()
    img.save("map_gen.png")


