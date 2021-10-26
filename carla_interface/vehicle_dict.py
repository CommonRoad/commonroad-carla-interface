"""
Helper functions for selecting most appropriate vehicle
x = length
y = width
z = height
"""

vehicle_dict = {
    'vehicle.audi.a2': {'x': 3.705369472503662, 'y': 1.7886788845062256, 'z': 1.5470870733261108}, 
    'vehicle.audi.tt': {'x': 4.181210041046143, 'y': 1.9941171407699585, 'z': 1.385296106338501}, 
    'vehicle.bmw.grandtourer': {'x': 4.611005783081055, 'y': 2.241713285446167, 'z': 1.6672759056091309}, 
    'vehicle.carlamotors.carlacola': {'x': 5.203838348388672, 'y': 2.614572286605835, 'z': 2.467444658279419}, 
    'vehicle.dodge_charger.police': {'x': 4.974244117736816, 'y': 2.0384013652801514, 'z': 1.5542958974838257}, 
    'vehicle.tesla.cybertruck': {'x': 6.273552894592285, 'y': 2.3895740509033203, 'z': 2.098191261291504}, 
    'vehicle.mercedesccc.mercedesccc': {'x': 4.673638820648193, 'y': 2.0022923946380615, 'z': 1.441947340965271}, 
    'vehicle.chargercop2020.chargercop2020': {'x': 5.237514495849609, 'y': 2.097083806991577, 'z': 1.638383150100708}, 
    'vehicle.chevrolet.impala': {'x': 5.357479572296143, 'y': 2.033203125, 'z': 1.4106584787368774}, 
    'vehicle.mustang.mustang': {'x': 4.717525005340576, 'y': 1.894826889038086, 'z': 1.300939917564392}, 
    'vehicle.volkswagen.t2': {'x': 4.4804368019104, 'y': 2.069315195083618, 'z': 2.0377910137176514}, 
    'vehicle.bmw.isetta': {'x': 2.2072951793670654, 'y': 1.4809197187423706, 'z': 1.3787471055984497}, 
    'vehicle.citroen.c3': {'x': 3.987684965133667, 'y': 1.8508483171463013, 'z': 1.6171096563339233}, 
    'vehicle.diamondback.century': {'x': 1.6428436040878296, 'y': 0.3725162446498871, 'z': 1.0239027738571167}, 
    'vehicle.charger2020.charger2020': {'x': 5.006059646606445, 'y': 2.097083806991577, 'z': 1.5347249507904053}, 
    'vehicle.audi.etron': {'x': 4.855708599090576, 'y': 2.0327565670013428, 'z': 1.6493593454360962}, 
    'vehicle.nissan.micra': {'x': 3.633375883102417, 'y': 1.845113754272461, 'z': 1.50146484375}, 
    'vehicle.gazelle.omafiets': {'x': 1.8354405164718628, 'y': 0.3289288878440857, 'z': 1.1256572008132935}, 
    'vehicle.lincoln.mkz2017': {'x': 4.901683330535889, 'y': 2.128324270248413, 'z': 1.5107464790344238}, 
    'vehicle.tesla.model3': {'x': 4.791779518127441, 'y': 2.163450002670288, 'z': 1.488319993019104}, 
    'vehicle.lincoln2020.mkz2020': {'x': 4.89238166809082, 'y': 2.230602979660034, 'z': 1.4801470041275024}, 
    'vehicle.seat.leon': {'x': 4.1928300857543945, 'y': 1.8161858320236206, 'z': 1.4738311767578125}, 
    'vehicle.bh.crossbike': {'x': 1.4872888326644897, 'y': 0.8592574596405029, 'z': 1.0795789957046509}, 
    'vehicle.yamaha.yzf': {'x': 2.2094459533691406, 'y': 0.8670341968536377, 'z': 1.2511454820632935}, 
    'vehicle.harley-davidson.low_rider': {'x': 2.3557403087615967, 'y': 0.7636788487434387, 'z': 1.2765706777572632}, 
    'vehicle.toyota.prius': {'x': 4.513522624969482, 'y': 2.006814479827881, 'z': 1.5248334407806396}, 
    'vehicle.kawasaki.ninja': {'x': 2.0333523750305176, 'y': 0.8025798797607422, 'z': 1.1454535722732544}, 
    'vehicle.nissan.patrol': {'x': 4.6045098304748535, 'y': 1.9315931797027588, 'z': 1.8548461198806763}, 
    'vehicle.mini.cooperst': {'x': 3.805800199508667, 'y': 1.97027587890625, 'z': 1.4750303030014038}, 
    'vehicle.mercedes-benz.coupe': {'x': 5.0267767906188965, 'y': 2.1515462398529053, 'z': 1.6355280876159668}, 
    'vehicle.jeep.wrangler_rubicon': {'x': 3.866220712661743, 'y': 1.9051965475082397, 'z': 1.8779358863830566}
}


def similar_by_length(length, width, height):
    """ Returns Carla BluePrint name of closest vehicle regarding length (second: width, third: height)
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_x': abs(length - current_best[1]['x']),
        'dif_y': abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for name in vehicle_dict:
        v = vehicle_dict[name]
        if abs(length - v['x']) < current_diff['dif_x']:
            current_best = (name, v)
            current_diff['dif_x'] = abs(length - current_best[1]['x'])
            current_diff['dif_y'] = abs(width - current_best[1]['y'])
            current_diff['dif_z'] = abs(height - current_best[1]['z'])
        if abs(length - v['x']) == current_diff['dif_x']:
            if abs(width - v['y']) < current_diff['dif_y']:
                current_best = (name, v)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
            if (abs(width - v['y']) == current_diff['dif_y']) &  (abs(height - v['z']) <= current_diff['dif_z']):
                current_best = (name, v)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
    return current_best


def similar_by_width(length, width, height):
    """ Returns Carla BluePrint name of closest vehicle regarding width (second: length, third: height)
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_x': abs(length - current_best[1]['x']),
        'dif_y': abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for name in vehicle_dict:
        v = vehicle_dict[name]
        if abs(length - v['y']) < current_diff['dif_y']:
            current_best = (name, v)
            current_diff['dif_x'] = abs(length - current_best[1]['x'])
            current_diff['dif_y'] = abs(width - current_best[1]['y'])
            current_diff['dif_z'] = abs(height - current_best[1]['z'])
        if abs(length - v['y']) == current_diff['dif_y']:
            if abs(width - v['x']) < current_diff['dif_x']:
                current_best = (name, v)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
            if (abs(width - v['x']) == current_diff['dif_x']) & (abs(height - v['z']) <= current_diff['dif_z']):
                current_best = (name, v)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
    return current_best


def similar_by_area(length, width, height):
    """ Returns Carla BluePrint name of closest vehicle regarding area (= length * width) (second: height)
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_area': abs(length - current_best[1]['x']) * abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for name in vehicle_dict:
        v = vehicle_dict[name]
        if abs(length * width - v['x'] * v['y']) < current_diff['dif_area']:
            current_best = v
            current_diff['dif_area'] = abs(length - current_best[1]['x']) * abs(length - current_best[1]['y']),
            current_diff['dif_z'] = abs(length - current_best[1]['z'])
        if (abs(length * width - v['x'] * v['y']) == current_diff['dif_area']) & (abs(height - v['z']) <= current_diff['dif_z']):
            current_best = v
            current_diff['dif_area'] = abs(length - current_best[1]['x']) * abs(length - current_best[1]['y']),
            current_diff['dif_z'] = abs(length - current_best[1]['z'])
    return current_best
