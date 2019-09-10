"""Find probability of visit POI/ uncertainty of a user's driving direction.
See detail decription in the end of file.
https://github.com/ashxz47/CSPrivacy/

Author: Andrey Shorov, ashxz47@gmail.com
Licence: MIT
"""
import iowork
import dsparse


import polyline
import time
import functools
import populartimes


import numpy as np
import scipy as sp
from scipy import stats
import matplotlib.pyplot as plt


def run_time(inner_func):
    """Show the runtime of the decorated function"""

    @functools.wraps(inner_func)
    def wrapper_timer(*args, **kwargs):
        start_time = time.time()    
        value = inner_func(*args, **kwargs)
        end_time = time.time()      
        run_time = end_time - start_time    
        print(f"Runtime of {inner_func.__name__!r} function = {run_time:.4f} seconds\n")
        return value

    return wrapper_timer


@run_time
def get_directions(origin_addr, destination_addr, waypoints_list=None, filename='', first_run=False):
    """!!!Potential function!!! 
    to get direction from any navigation provider"""

    origin_addr = str(origin_addr)
    destination_addr = str(destination_addr)
    
    if waypoints_list is not None:
        directions = direction(origin_addr, destination_addr, waypoints=waypoints_list)
    else:
        directions = direction(origin_addr, destination_addr)
    
    print("Direction received")
    if first_run:
        if not filename:
            directions[0]['filename'] = str(round(get_time)) # creating filename extension of temporary data
            filename = directions[0]['filename']
        else:
            directions[0]['filename'] = filename
        print("Filename extension of temporary data is:", filename)
        iowork.save_temp_data(directions, 'directions_' + filename)
    
    return directions


@run_time
def in_time_directions(directions, tracking_interval):
    """!!!Potential function!!!
    to get only directions that are fit to the tracking interval. 
    Also find a free time that could be used to visit to POI"""

    available_directions = []
    tracking_interval = round(tracking_interval)
    for i in range(len(directions)):
        duration = 0
        min_duration = 0
        if len(directions[i]['legs']) == 1:     # do if there is no waypoints
                directions[i]['duration'] = directions[i]['legs'][0]['duration']
                min_duration = in_time_direction_probablity(directions[i]['legs'][0]['duration']) # min duration returned by lognorm dist
                directions[i]['min_duration'] = min_duration # save min duration
                if min_duration < tracking_interval:
                    directions[i]['overview_free_time'] = tracking_interval - min_duration 
                    available_directions.append(directions[i])
        else:   # do if there are waypoints
            for j in range(len(directions[i]['legs'])):
                duration += directions[i]['legs'][j]['duration'] # sum of min durations of legs
            directions[i]['duration'] = duration                                       
            for j in range(len(directions[i]['legs'])):
                min_duration += in_time_direction_probablity(directions[i]['legs'][j]['duration']) # sum of min durations of legs
            directions[i]['min_duration'] = min_duration   
            if min_duration < tracking_interval:
                directions[i]['overview_free_time'] = tracking_interval - min_duration
                available_directions.append(directions[i])
        
    if len(available_directions) > 0:
        print("The number of intime directions:", len(available_directions))
    else:
        print("No in time directions were found")
    
    return available_directions


def in_time_direction_probablity(duration):
    """return minimal time needed a vehicle to run a path by any navigation provider 
    with help of lognorm distribution"""

    mean_ln = np.log(duration)
    std_ln = 0.41 # regarding the paper std_nostop is ln
    mln_3std = mean_ln-3*std_ln
    min_duration = np.exp(mln_3std)
    
    return int(min_duration)


@run_time
def get_directions_for_ds(exist_directions, tracking_interval):
    """get from any navigation provider to compare the directions with data set"""

    directions = []
    time_interval = round(tracking_interval / 600) # get time interval to get 'path' coordinates related to the time
    if time_interval <= 0:
        print("WARNING! Time interval at get_directions_for_ds() <= 0! No tracking interval was defined.")
        exit(1)
    
    for line in exist_directions:
        start_addr = ''
        end_addr = ''
        coor_num = 0
        whole_path = [] 
        # get only coordinates when the tracking point (tracking interval) 
        # for coor_num in range(0, len(line['path']), time_interval):
        while coor_num <= len(line['path']):
            # after run, get start address
            """if coor_num == 0:
                start_addr = reverse_geocode(latitude=[line['path'][coor_num]['lat'], longitude=line['path'][coor_num]['lon']])
            else:       
                end_addr = reverse_geocode(latitude=[line['path'][coor_num]['lat'], longitude=line['path'][coor_num]['lon']])"""

            # if we have start and end addresses get directions
            if start_addr and end_addr:
                temp_directions = get_directions(start_addr, end_addr)
                temp_directions = in_time_directions(temp_directions, tracking_interval)
                # save real path coordinates to each direction
                new_real_path = {}
                for direct in temp_directions:
                    if not new_real_path:
                        if len(line['polyline_coordinates']) > coor_num: # delete exist real path and encode new polyline related to direction by any navigation provider 
                            new_real_path['real_coordinates'] = line['polyline_coordinates'][:coor_num+1]
                            new_real_path.update({'overview_polyline': polyline.encode(new_real_path['real_coordinates'],5)})
                            new_real_path.update({'original_polyline': line['original_polyline']['points']}) 
                    direct['real_path'] = new_real_path      # add the real path to direction for comparison 
                    direct['direction_time'] = coor_num      # save the tracking interval for direction
                start_addr = end_addr # for the next direction, the start address is equal to the end address of the previous direction
                temp_directions = decode_polylines(temp_directions)
                whole_path.extend(temp_directions)

            if len(line['path'])-1-coor_num >= time_interval: # check if interval is not exceed the path
                coor_num += time_interval
            elif len(line['path'])-1-coor_num >= 1: # if time interval in excess of the path get the last path's point
                coor_num += len(line['path'])-1-coor_num
            else:
                break # exit if zero
        if whole_path:
            directions.append(whole_path)
    try:
        directions[0][0]['filename'] = exist_directions[0]['path'][0]['filename']
    except:
        try:
            directions[0][0]['filename'] = exist_directions[0]['path'][0]['time']
        except:
            try:
                directions[0][0]['filename'] = 'unknown_filename'
            except:
                print("No directions were found. Exiting...")
                exit(1)

    iowork.save_temp_data(directions, "direct_" + directions[0][0]['filename'])
    
    return directions


@run_time
def decode_polylines(directions, path_num=None):
    """!!!Potential function!!!
    get coordinates from polylines"""

    if path_num: # decode and return only direction specified in 'path_num' parameter
        directions[path_num]['polyline_coordinates'] = polyline.decode(directions[path_num]['overview_polyline']) 
        new_list = []
        new_list.append(directions[path_num])
        directions = new_list
        print("Polyline of direction[{}] is decoded successfully".format(path_num)) 
    elif not path_num:
        for direction in directions:
            direction['polyline_coordinates'] = polyline.decode(direction['overview_polyline']) 
        print("Polylines decoded successfully")
    
    return directions


def pop_times(place_id):
    """Get rating, time spend with help of any navigation provider"""

    place_id = str(place_id)
    pop_times = populartimes(place_id)
    
    return pop_times


@run_time
def add_popular_times(places):
    """!!!Potential function!!!
    add popular times and time spend to polyline_coor_POI list"""
    
    print("Adding popular times and time spend to polyline_coor_POI list")
    for i in range(len(places['results'])): 
        pop_times_res = pop_times(places['results'][i]['place_id'])
        pop_times_fields = dict()
        d_items = list(['rating', 'rating_n', 'time_spent'])
        for j in d_items:
            pop_times_fields[j] = pop_times_res.get(j, -1)
            
        if pop_times_fields['time_spent'] != -1:
            pop_times_fields['time_spent'][0] = pop_times_fields['time_spent'][0] * 60
            pop_times_fields['time_spent'][1] = pop_times_fields['time_spent'][1] * 60
        
        places['results'][i].update(pop_times_fields)
 
    print("Popular times were added to the places")
    
    return places


def get_near_poi(location, max_radius, place_type=None, add_popular=True):
    """!!!Potential function!!!
    get near POIs from coordinates"""
    
    print("Get near POIs from coordinates")
    
    places = near_places(location = location, radius=max_radius, type=place_type)
    all_results = []
    all_results.extend(places['results'])                                               
    places['results'] = all_results 
    if add_popular:
        places = add_popular_times(places)   # request popular times for all places
    print(len(places['results']), "POIs are available")

    if len(places['results']) > 0: 
        print(len(places['results']), "POIs are available")
        if add_popular:
            places = add_popular_times(places)   # request popular times for all places
        else:
            print("No POIs are available") 
    
    return places


@run_time
def get_near_poi_polylines(directions, max_radius, filename='', place_type=[], add_popular=True, distance_between=0.020):
    """!!!Potential function!!!
    get POIs for polyline coordinates (polyline points)"""

    print("Get POI for polyline coordinates")
    for i in range(len(directions)): 
        directions[i]['polyline_coor_POI'] = []
        next_j = 0
        for j in range(len(directions[i]['polyline_coordinates'])):
            location = str(directions[i]['polyline_coordinates'][j][0]) + ',' + str(directions[i]['polyline_coordinates'][j][1])
            coordinates = directions[i]['polyline_coordinates'][j]
            if j == 0:
                places = []
                if place_type:
                    for place in place_type:
                        result = get_near_poi(location, max_radius, place, add_popular=add_popular)
                        places.extend(result['results'])
                elif not place_type:
                    result = get_near_poi(location, max_radius, add_popular=add_popular)
                    places.extend(result['results'])
                directions[i]['polyline_coor_POI'].append([tuple(coordinates)]+[places])

            distance_lat = directions[i]['polyline_coordinates'][next_j][0] - directions[i]['polyline_coordinates'][j][0]
            distance_lon = directions[i]['polyline_coordinates'][next_j][1] - directions[i]['polyline_coordinates'][j][1]
            
            distance_lat = abs(distance_lat)
            distance_lat = round(distance_lat, 6)

            distance_lon = abs(distance_lon)
            distance_lon = round(distance_lon, 6)
            
            if distance_lat > distance_between or distance_lon > distance_between: # distance between POIs by default 0.020
                next_j = j
                places = []
                for place in place_type:
                    result = get_near_poi(location, max_radius, place, add_popular=add_popular)
                    places.extend(result['results'])
                if not place_type:
                    result = get_near_poi(location, max_radius, add_popular=add_popular)
                    places.extend(result['results'])
                directions[i]['polyline_coor_POI'].append([tuple(coordinates)]+[places])
    if place_type:
        directions[0]['place_type'] = place_type # add downloaded place types for report
    if not filename:
        try:
            filename = directions[0]['filename']
        except:
            directions[0]['filename'] = 'unknown_filename'
    else:
        directions[0]['filename'] = filename
        print("WARNING! Filename extension was changed to:", filename)
    iowork.save_temp_data(directions, 'nearbyPOIs_' + filename)
    print("Nearby POIs are downloaded")

    return directions


def get_poi_by_type(poi_list, poi_type):
    """Find POI type"""

    if not poi_type:
        return True
    else:
        for i in range(len(poi_list)):
            for j in range(len(poi_type)):
                if poi_list[i] == poi_type[j]:
                    return True
        return False


@run_time
def get_waypoints_for_poi(directions, poi_type=None, filename=''):
    """!!!Potential function!!!
    select (filter) potential waypoints from obtained list of POIs with help of get_poi_by_type(), 
    remove all unnecessary data. Add only POIs that have "time_spent" info and are in free time intertval"""
    
    try:
        directions[0]['overview_free_time']
    except:
        print("WARNING! No key \'overview_free_time\' is presented! Run the \'in_time_directions\' function before!\nExiting...")
        exit(1)

    for i in range(len(directions)):
        waypoint_list = []
        origin_addr = directions[i]['legs'][0]['start_address'] # update to directions without waypoints
        if len(directions[i]['legs']) == 1:
            destination_addr = directions[i]['legs'][0]['end_address'] # for directions without waypoints
        else:
            destination_addr = directions[i]['legs'][2]['end_address'] # for directions with waypoints NEED TEST
        
        for j in range(len(directions[i]['polyline_coor_POI'])):
            for k in range(len(directions[i]['polyline_coor_POI'][j][1])):
                if get_poi_by_type(directions[i]['polyline_coor_POI'][j][1][k]['types'], poi_type): # filter the existing POIs by elements in poi_type, if type is None return True
                    try:
                        if directions[i]['polyline_coor_POI'][j][1][k]['time_spent'] != -1: # add only POIs have "time_spent" information
                            if directions[i]['polyline_coor_POI'][j][1][k]['time_spent'][0] < directions[i]['overview_free_time']:
                                waypoint = directions[i]['polyline_coor_POI'][j][1][k]['place_id']
                                name = directions[i]['polyline_coor_POI'][j][1][k]['name']
                                types = directions[i]['polyline_coor_POI'][j][1][k]['types']
                                time_spent = directions[i]['polyline_coor_POI'][j][1][k]['time_spent']
                                populartimes = directions[i]['polyline_coor_POI'][j][1][k]['populartimes']
                                rating_n = directions[i]['polyline_coor_POI'][j][1][k]['rating_n']
                                waypoint_list.append(["place_id:" + waypoint, name, types, time_spent, populartimes, rating_n])
                    except KeyError:
                        print("\'time_spent\' parameter is not available because no popular times were added")
        if waypoint_list:
            destination_wayp_list = [origin_addr, destination_addr, waypoint_list]
            directions[i].update({'dest_wayp_list': destination_wayp_list})
            destinations = get_destination_via_poi(destination_wayp_list)
            directions[i].update({'all_destinations': destinations})
            # directions_obtained = True
            print("Potential in time waypoints were obtained for direction[{}]".format(i))
    
    if poi_type:
        directions[0]['filtered_poi'] = poi_type # add filtered POI types for report
    if not filename:
        try:
            filename = directions[0]['filename']
        except:
            directions[0]['filename'] = 'unknown_filename'
    else:
        directions[0]['filename'] = filename
        print("WARNING! Filename extension was changed to:", filename)
    iowork.save_temp_data(directions, 'dest_wayp_list_' + filename)
   
    return directions


@run_time
def get_destination_via_poi (destination_list):
    """!!!Potential function!!!
    get all routes via POI for dest_wayp_list (get_waypoints_for_poi) list presentation"""

    destination = []
    for i in range(len(destination_list[2])):
        destination.extend(get_directions(destination_list[0], destination_list[1], destination_list[2][i][0]))
        destination[i]['name'] = destination_list[2][i][1]
        destination[i]['time_spent'] = destination_list[2][i][3]
        destination[i]['rating_n'] = destination_list[2][i][5]
    
    iowork.save_temp_data(destination,'potential_dest') 
    print("Potential directions via POI received")
    
    return destination


@run_time
def potential_visit_poi (directions, tracking_interval, filename='', add_no_stop=False):
    """calculate probability to visit POIs and overall entropy"""

    tracking_interval = tracking_interval/60
    for i in range(len(directions)): 
        all_probab = []
        duration = directions[i]['duration']/60
        free_time = directions[i]['overview_free_time']/60
        try:
            directions[i]['all_destinations'] = in_time_directions(directions[i]['all_destinations'], tracking_interval*60)
            for j in range(len(directions[i]['all_destinations'])):
                minim = abs(directions[i]['all_destinations'][j]['time_spent'][0]/60) # we guess that Goolge min time spent equal to -2*sigma
                maxim = abs(directions[i]['all_destinations'][j]['time_spent'][1]/60) # max time spent = 2*sigma
                if minim == maxim: # in case if there is no time interval, artificially create it
                    minim = minim/2
                    maxim = 3*maxim/2
                mean = np.mean([minim, maxim])
                std = (maxim-minim)/4

                # Z-score calc and finding of the potential probabl interval
                # three sigma rule: m+s=68%, m+2s=95%, m+3s=99,7%
                z = (free_time-mean)/std
                if -1<=z<=1:
                    probab = 0.341
                elif -2<=z<-1 or 2>=z>1:
                    probab = 0.136
                elif -3<=z<-2 or 3>=z>2:
                    probab =  0.021
                else:
                    probab = 0.0013
                directions[i]['all_destinations'][j]['dist_data'] = ({'mean': mean, 'std': std, 'zscore': z, 'probab': probab})
                
                # rating addition: unweighted probabilities
                if directions[i]['all_destinations'][j]['rating_n'] > 0:
                    probab = probab * directions[i]['all_destinations'][j]['rating_n'] #CHECK IF ZERO
                    all_probab.append(probab)

            # add nostop_prob     
            nostop_probab = no_stop_lognormal(duration, tracking_interval)
            if add_no_stop:
                all_probab.append(nostop_probab)
                sum_all = sum(all_probab)
                n_prob = []
                for prob in all_probab[:-1]:
                    n_prob.append(prob/sum_all)
                    print("Weighed probability of visit a POI is: {}%".format(round(prob/sum_all * 100, 2)))
                print("Weighed no stop probability is: {}%".format(round(nostop_probab/sum_all * 100, 2)))

            elif not add_no_stop:
                """find probability when no_stop doesn't participate in propotion""" 
                stop_prob_weight = 1-nostop_probab
                # find proportion to calc entropy * stop_prob_weight
                sum_all = sum(all_probab)
                n_prob = []
                for prob in all_probab:
                    n_prob.append(prob/sum_all * stop_prob_weight)
                    print("Weighed probability of visit a POI is: {}%".format(round(prob/sum_all * 100, 2)))
            
            # check of input data correctness for entropy
            test = sum(n_prob)
            print("\nCheck. The sum of probabilities is: {}\n".format(test))

            entropy = sp.stats.entropy(n_prob, base=2)
            entropy_data = {'probabilities': all_probab, 'normal_prob': n_prob, 'direction_entropy': entropy, 
                            'tracking_interval': tracking_interval, 'no_stop_prob': nostop_probab} # create dict to save entropy data
            directions[i].update(entropy_data)
            print("Direction[{}] entropy is: {}".format(i, entropy))
        except:
            nostop_probab = no_stop_lognormal(duration, tracking_interval)
            all_probab.append(nostop_probab)
            sum_all = sum(all_probab)
            n_prob = []
            for prob in all_probab[:-1]:
                n_prob.append(prob/sum_all)
                print("Weighed probability of visit a POI is: {}%".format(round(prob/sum_all * 100, 2)))
            print("Weighed no stop probability is: {}%".format(round(nostop_probab/sum_all * 100, 2)))
            entropy = sp.stats.entropy(n_prob, base=2)
            entropy_data = {'probabilities': all_probab, 'normal_prob': n_prob, 'direction_entropy': entropy, 
                            'tracking_interval': tracking_interval, 'no_stop_prob': nostop_probab} # create dict to save entropy data
            directions[i].update(entropy_data)
            print("Direction[{}] entropy is: {}".format(i, entropy))
            print("No potential destinations for direction[{}]".format(i))
    
    if not filename:
        filename = directions[0]['filename']
    else:
        directions[0]['filename'] = filename
        print("WARNING! Filename extension was changed to: ", filename)
    
    iowork.save_temp_data(directions, 'entropy_data_{}_{:.0f}'.format(filename, tracking_interval))
    
    return directions


def no_stop_lognormal(duration, tracking_interval):
    """ Return no stop probability based on the paper: Local Optimization Strategies in Urban Vehicular Mobility"""
    
    mean_ln = np.log(duration)
    std_ln = 0.41 # regarding the paper std_nostop is ln
    
    dist = sp.stats.lognorm(scale=np.exp(mean_ln), s=std_ln)
    nostop_probab = dist.sf(tracking_interval) # Survival function (also defined as 1 - cdf)
    
    # show_plot(mean_ln, std_ln, 'lognormal')
    """ Confidence interval of three sigma rule: m+s=68%, m+2s=95%, m+3s=99,7%
    logarithm rules: -ln_std=mean-std, ln_std=mean+std
    -2ln_std=mean-2std, 2ln_std=mean+2*std
     
    Z-score calc and finding of the potential probabl interval
    z_log = (np.log(tracking_interval)-mean_ln)/std_ln
    mln_std = mean_ln-std_ln
    mln_2std = mean_ln-2*std_ln
    mln_3std = mean_ln-3*std_ln
    ln_std = mean_ln+std_ln
    ln_2std = mean_ln+2*std_ln
    ln_3std = mean_ln+3*std_ln
    if mln_std<=z_log<=ln_std:
        nostop_probab = 0.341
    elif mln_2std<=z_log<mln_std or ln_2std>=z_log>ln_std:
        nostop_probab = 0.136
    elif mln_3std<=z_log<mln_2std or ln_3std>=z_log>ln_2std:
        nostop_probab =  0.021
    else:
        nostop_probab = 0.0013 """
   
    print("No stop probability is:", nostop_probab)
    
    return nostop_probab


def show_plot(mean, std, distribution):
    """ 'normal' for normal dist, 'lognormal' for lognormal"""

    if distribution == 'normal':
        s = np.random.normal(mean, std, 1000)
    elif distribution == 'lognormal':
        s = np.random.lognormal(mean, std, 1000) 
    else: # else run normal dist
        s = np.random.normal(mean, std, 1000) 

    _count, bins, _ignored = plt.hist(s, 100, density=True)
    plt.plot(bins, 1/(std * np.sqrt(2 * np.pi)) *
        np.exp( - (bins - mean)**2 / (2 * std**2) ),
        linewidth=2, color='r')

    plt.show()     
    plt.savefig('plots/{}_{:.2f}_{:.2f}.png'.format(distribution, mean, std))
    plt.close()
    

def convert_to(directions, direction_num=0):
    """convert multiply directions from dataset to a format that readble by other functions"""

    new_directions = directions[direction_num]

    return new_directions


def dsparse_run(tracking_interval):
    """epfl/mobility dataset processing"""

    files = iowork.read_all_files()         # get all files in data sets' directory
    for f in files:                         # for each file
        directions = dsparse.sort_file(f)   # sort by the time attribute
        
        directions = dsparse.get_busy_directions(directions)        # get coordinates in when taxi is busy
        if not directions:
            continue
        directions = dsparse.get_coor_between(directions, 600)      # get coordinates in X seconds time interval

        directions = dsparse.cut_directions(directions,10000,tracking_interval)
        
        directions = decode_polylines(iowork.get_temp_data('coor_between_new_abboip'),path_num=3)
        directions = get_directions_for_ds(directions, tracking_interval)
        iowork.save_temp_data(directions, 'direct_temp_{}'.format(tracking_interval))
    
        directions = dsparse.get_for_json(iowork.get_temp_data('direct_temp_{}'.format(tracking_interval)))
        iowork.save_as_json(directions, 'direct_{}'.format(tracking_interval))
        dsparse.add_to_json(iowork.get_text_data('direct_{}'.format(tracking_interval), extension='.json'), 
                                                                    'direct{}'.format(tracking_interval))
    return directions


"""Choose tracking interval in seconds"""
tracking_interval = 600 # tracking interval is seconds when vehicle position send to the vehicle owner

# Run epfl/mobility data set processing
# directions = dsparse_run(tracking_interval)

"""!!!Potential usage!!!"""
"""get Direction and save them to temp file (useful to reduce amount of requests)
    Uncomment functions between '######' to run a full cycle
    Replace start and finish addresses in the getDirection function below
    Now the addresses are: Kumpulan kampus, 00560 Helsinki", "Sello, Leppävaarankatu 3-9, 02600 Espoo

Potential POIs can be found with help of 'get_near_poi_polylines' using parameter in format: place_type=['poi_type1', 'poi_type2']). 
The function retrives potential POIs from any navigation provider. If list is empty (place_type=[]), 
the function should retrive all types of POIs providing by any navigation provider.
The POIs can be filtered during further processing with help of 'get_waypoints_for_poi' (format: ['poi_type1', 'poi_type2']).
"""
#############################################
# directions = get_directions("Kumpulan kampus, 00560 Helsinki", "Sello, Leppävaarankatu 3-9, 02600 Espoo", first_run=True)
# directions = in_time_directions(directions, tracking_interval)
# directions = decode_polylines(directions)
# directions = get_near_poi_polylines(directions, max_radius=1000, place_type=['shopping_mall'], add_popular=True)
# directions = get_waypoints_for_poi(directions)
# directions = potential_visit_poi(directions, tracking_interval, add_no_stop=True)
# iowork.save_as_json(directions, 'direct_entropy_data_'+ str(tracking_interval))
# iowork.save_temp_data(directions, 'direct_entropy_data_'+ str(tracking_interval))
# iowork.print_data(iowork.get_temp_data('direct_entropy_data_'+ str(tracking_interval)))
#############################################