"""Parsing the epfl/mobility dataset
http://crawdad.org/epfl/mobility/20090224/

 and convert it to fit CSPrivacy application
https://github.com/ashxz47/CSPrivacy/.

Author: Andrey Shorov, ashxz47@gmail.com
License: MIT
"""

import iowork

import polyline
import os
import time


def get_data(filename, directory='data'):
    """get data from disk"""

    with open(directory + '/'+ filename + '.txt', 'r') as f:
        data_file = f.readline()
        f.close
    print("Data loaded from", filename + ".txt in working directory")
    
    return data_file


def sort_rule(elem):
    return elem['time']


def sort_file(filename):
    """sort file data by the time"""

    data = []
    with open(filename, 'r') as file_object:
        line = file_object.readline()
        while line:
            line_data = [name.strip() for name in line.split(' ')]
            row = {
                'lat': line_data[0],
                'lon': line_data[1],
                'busy': line_data[2],
                'time': line_data[3]
            }
            data.append(row)
            line = file_object.readline()
        data.sort(key=sort_rule)
    
    data[0]['filename'] = os.path.basename(filename).rsplit('.', 1)[0] # get only filename
    iowork.save_temp_data(data, 'sorted_data_' + data[0]['filename'])
    print("Data sorted by the time and saved")
        
    return data


def get_coor_between(data, time_interval):
    """get coordinates from file return only coordinates in X seconds time interval"""

    if not data:
        return
    path = []
    directions = []
    time_gap = 1
    orig_poly = {}
    for direct in data:
        if len(path) > 1:
            list_arg = {'path':path, 'original_polyline':{'points': orig_poly['points']}}
            directions.append(list_arg)
        path = []
        last_time = '0'
        for line in direct['path']:   
            if last_time == '0':
                last_time = line['time']
                path.append(line)
                orig_poly = {'points': direct['overview_polyline']['points']}
                time_gap = 1
            if (int(line['time']) - int(last_time) > time_interval*time_gap):  # if time of the line is more than time_interval
                if (int(line['time']) - int(last_time) < time_interval*(time_gap+1)): # but still not exceeds time_interval+gap
                    if time_gap >= 61:
                        time_gap = 1
                        last_time = line['time']
                        list_arg = {'path':path, 'original_polyline':{'points': orig_poly['points']}}
                        directions.append(list_arg)
                        path = []
                        path.append(line)
                        continue
                    path.append(line)
                    time_gap += 1
                elif (int(line['time']) - int(last_time) > time_interval*(time_gap+1)):
                    time_gap = 1
                    last_time = line['time']
                    list_arg = {'path':path, 'original_polyline':{'points': orig_poly['points']}}
                    directions.append(list_arg) 
                    path = []
                    path.append(line)
    directions = encode_dataset_polyline(directions) # encode overview polyline for each direction
    try:
        directions[0]['path'][0]['filename'] = data[0]['path'][0]['filename']
    except:
        directions[0]['path'][0]['filename'] = "unknown_filename"
    iowork.save_temp_data(directions, 'coor_between_' + directions[0]['path'][0]['filename'])
    print("Coordinates of time intervals obtained")

    return directions


def get_busy_directions(data):
    """get directions only for busy times and convert them to ACSPrivacy format"""

    if not data:
        return
    path = []
    directions = []
    last_busy = '0'

    for line in data:
        if line['busy'] == '1':
            if last_busy == '0':
                last_busy = '1'
            path.append(line)
            continue
        if line['busy'] == '0':
            if last_busy == '1':
                directions.append({'path':path})
                path = []
                last_busy = '0'
    
    if len(directions) <= 1 and len(directions[0]['path']) <= 1:
        return
    directions[0]['path'][0]['filename'] = data[0]['filename']
    directions = encode_dataset_polyline(directions) # get polyline for original path
    iowork.save_temp_data(directions, 'coor_busy_' + data[0]['filename'])
    print("Coordinates of directions for busy time received")
    
    return directions


def get_all_directions(data, split_by=0):
    """get directions for busy and not busy times and convert them to CSPrivacy format
        split_by: 0 - only busy or only free times paths, 1...n - get path contains number of lines"""

    if not data:
        return
    path = []
    directions = []
    
    if split_by == 0:
        last_busy = '0'
        for line in data:
            if line['busy'] == '1':
                if last_busy == '0':
                    directions.append({'path':path})
                    path = []
                    last_busy = '1'
                path.append(line)
                continue
            if line['busy'] == '0':
                if last_busy == '1':
                    directions.append({'path':path})
                    path = []
                    last_busy = '0'
                path.append(line)
    elif split_by > 0: 
        for i, line in enumerate(data):
            path.append(line)
            if i == split_by:
                directions.append({'path':path})
                path = []
                split_by += split_by
    else:
        print("split_by parameter is negative. Exiting...")
        exit(1)


    directions[0]['path'][0]['filename'] = data[0]['filename']
    directions = encode_dataset_polyline(directions) # get polyline for original path
    iowork.save_temp_data(directions, 'all_direct_' + data[0]['filename'])
    print("Coordinates of directions for busy and free times received")
    
    return directions


def encode_dataset_polyline(data):
    """get coordinates and encode polyline"""
    
    coordinates = ()
    directions = []
    for directions in data:
        direct_poly = []
        for line in directions['path']:
            coordinates = (float(line['lat']), float(line['lon']))
            direct_poly.append(coordinates)
        # add polyline in format of 'main.decodePolylines(directions)'
        directions.update({'overview_polyline': {'points': polyline.encode(direct_poly, 5)}}) 
   
    try:
        data[0]['path'][0]['filename']
    except:
        try:
            data[0]['path'][0]['filename'] = 'unknown_filename'
        except:
            print("No paths were found. Exiting...")
            exit(1)
    iowork.save_temp_data(data,'ds_polyline_' + data[0]['path'][0]['filename'])
    print("Polyline of dataset directions is encoded and saved")
    
    return data


def cut_directions(directions, direct_num, minutes_interval, tracking_interval):
    """cut number of directions while testing"""
    
    tracking_interval /= 60 # convert to minutes
    tracking_interval /= minutes_interval # select number of lines related to tracking_interval depends on minutes interval of input data
    cutted_directions = []
    if direct_num > len(directions):
        direct_num = len(directions)
    num_added = 0
    for i, line in enumerate(directions):
        if len(directions[i]['path']) > tracking_interval: # get only direction that continuos more than tracking_interval 
            cutted_directions.append(line)
            num_added += 1
            if num_added == direct_num:
                break
    if not cutted_directions:
        return False
    else:
        try:
            directions[0]['path'][0]['filename']
        except:
            directions[0]['path'][0]['filename'] = "unknown_filename"
        iowork.save_temp_data(cutted_directions, '{}_cutted_{:0.0f}_interval'.format(directions[0]['path'][0]['filename'], tracking_interval),directory='real_data')
        
        return cutted_directions


def get_for_json(directions):
    """get polyline to visualize them with help of map visualization script"""

    direct_poly = []
    for direct in directions:
        new_line = []
        for line in direct:
            new_line.append(line['overview_polyline']['points'])
        new_line.append(line['real_path']['original_polyline']['points']) # last polyline is original data set path
        direct_poly.append(new_line)
    
    return direct_poly


def add_to_json(json_file, var_name):
    """add 'var var_name=;' to json file to read it in map visualization script"""
    
    new_json = "var " + var_name + "=" + json_file + ";"
    iowork.save_as_text(new_json, var_name, directory='output', extension='.json')
    
    return new_json