"""Read/write functions for CSPrivacy application
https://github.com/ashxz47/CSPrivacy/.

Author: Andrey Shorov, ashxz47@gmail.com
License: MIT
"""

import os
import pickle
import json


def read_all_files(directory='data/cabspottingdata'):
    """read files in dataset for further processing"""

    file_list = []
    for root, _dirs, files in os.walk(directory):
        for file in files:
            if 'new_' and '.txt' in file:
                file_list.append(os.path.join(root, file))
            elif 'new_' and '.temp' in file:
                file_list.append(os.path.join(root, file))
    
    return file_list # return epfl/mobility files found in root directory


def save_temp_data(data, filename, directory='temp'):
    """save temp data to disk"""

    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(directory + '/'+ filename + '.temp', 'wb') as f:
        pickle.dump(data, f)
        f.close()
    print("Data saved to", filename + ".temp in working directory")


def get_temp_data(filename, directory='temp', extension='.temp'):
    """get temp data from disk"""
    
    if '.temp' not in filename:
        filename += extension
    try:
        with open(directory + '/'+ filename, 'rb') as f:
            data_file = pickle.load(f)
            f.close()
        print("Data loaded from {} in working directory".format(filename))
        return data_file
    except FileNotFoundError as err:
        print("\nNo file was found at following path: '{}'. Exiting...\n".format(err.filename))
        exit(1)


def save_as_json(data, filename, directory='output'):
    """save JSON data to disk"""

    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(directory + '/'+ filename + '.json', 'w') as json_file:  
        json.dump(data, json_file)
        json_file.close()
    print("Data saved to", filename +".json in "+directory+" directory")


def save_as_text(data, filename, directory='output', extension='.txt'):
    """save printable data to text file"""

    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(directory + '/'+ filename + extension, "w", encoding="utf-8") as text_file:
        print(data, file=text_file)
        text_file.close()
    print("Data saved to", filename + extension, "in working directory")


def get_text_data(filename, directory='output', extension='.txt'):
    """get text data from disk"""

    with open(directory + '/'+ filename + extension, 'r') as f:
        text_file = f.read()
        f.close()
    print("Data loaded from", filename + extension, "in working directory")
    return text_file


def print_data(directions):
    """Extract data to print"""

    direction_out = ''
    for i in range(len(directions)):
        direction_out += "Direction {}\n".format(i)
        direction_out += "Distance: {} ({})\n".format(directions[i]['legs'][0]['distance']['value'], 
                                                        directions[i]['legs'][0]['distance']['text'])
        direction_out += "Navigation's duration: {} ({})\n".format(directions[i]['legs'][0]['duration']['value'], 
                                                                directions[i]['legs'][0]['duration']['text'])
        direction_out += "Min duration: {}\n".format(directions[i]['min_duration'])
        direction_out += "Max free time: {}\n".format(directions[i]['overview_free_time'])
        try:
            direction_out += "Downloaded POI types: {}\n".format(directions[0]['place_type'])
        except:
            direction_out += "Downloaded POI types: Not specified\n"
        try:
            direction_out += "Selected POI types: {}\n\n".format(directions[0]['filtered_poi'])
        except:
            direction_out += "Selected POI types: Not specified\n"
        direction_out += "No stop probability is: {}\n".format(round(directions[i]['entropy_data']['no_stop_prob'], 2))
        direction_out += "Weighed stop probability is: {}\n".format(round(directions[i]['entropy_data']['weighed_no_stop'], 2))
        try:
            direction_out += "Amount of potential POIs: {}\n".format(len(directions[i]['all_destinations']))
        except KeyError:
            direction_out += "No of potential POIs are presented\n"
        for prob in range(len(directions[i]['entropy_data']['normal_prob'])):
            directions[i]['entropy_data']['normal_prob'][prob] = round(directions[i]['entropy_data']['normal_prob'][prob],3)
        direction_out += "Normalized probabilities of visit POI: {}\n".format(directions[i]['entropy_data']['normal_prob'])
        direction_out += "Ellipse area: {}\n".format(directions[i]['ellipse_area'][i])
        direction_out += "Entropy: {:.2f}\n\n".format(directions[i]['entropy_data']['direction_entropy'])

        direction_out += "In detail:\n"
        try:
            for j in range(len(directions[i]['all_destinations'])):
                direction_out += "POI[{}], probability of visit: {:.1f}%\n".format(j, directions[i]['normal_prob'][j] * 100)
        except KeyError:
            direction_out +="No of potential POIs are presented\n"
        direction_out += "\n\n"
    tracking = directions[0]['tracking_interval']
    start_addr = directions[0]['legs'][0]['start_address']
    end_addr = directions[0]['legs'][0]['end_address']
    output = "Route has {} directions\nStart address: {}\nEnd address: {}\nTracking interval: {:.0f}\n\n{}".format(len(directions),         
                                                                                                start_addr, end_addr, tracking, direction_out)

    save_as_text(output, filename="output_{}_{:.0f}".format(directions[0]['filename'], directions[0]['tracking_interval']))
    print("Output file is created")
    
    return True