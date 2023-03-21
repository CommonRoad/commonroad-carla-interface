from typing import Dict, Union
from commonroad.scenario.obstacle import ObstacleType

# Helper functions for selecting most appropriate vehicle
# x = length
# y = width
# z = height
vehicle_dict: Dict[str, Dict[str, Union[float, ObstacleType]]] = {
    'vehicle.audi.a2':
        {'x': 3.705369472503662, 'y': 1.7886788845062256, 'z': 1.5470870733261108, 'type': ObstacleType.CAR},
    'vehicle.audi.tt':
        {'x': 4.181210041046143, 'y': 1.9941171407699585, 'z': 1.385296106338501, 'type': ObstacleType.CAR},
    'vehicle.bmw.grandtourer':
        {'x': 4.611005783081055, 'y': 2.241713285446167, 'z': 1.6672759056091309, 'type': ObstacleType.CAR},
    'vehicle.carlamotors.carlacola':
        {'x': 5.203838348388672, 'y': 2.614572286605835, 'z': 2.467444658279419, 'type': ObstacleType.CAR},
    'vehicle.dodge.charger_police':
        {'x': 4.974244117736816, 'y': 2.0384013652801514, 'z': 1.5542958974838257, 'type': ObstacleType.CAR},
    'vehicle.tesla.cybertruck':
        {'x': 6.273552894592285, 'y': 2.3895740509033203, 'z': 2.098191261291504, 'type': ObstacleType.TRUCK},
    'vehicle.chevrolet.impala':
        {'x': 5.357479572296143, 'y': 2.033203125, 'z': 1.4106584787368774, 'type': ObstacleType.CAR},
    'vehicle.ford.mustang':
        {'x': 4.717525005340576, 'y': 1.894826889038086, 'z': 1.300939917564392, 'type': ObstacleType.TRUCK},
    'vehicle.volkswagen.t2':
        {'x': 4.4804368019104, 'y': 2.069315195083618, 'z': 2.0377910137176514, 'type': ObstacleType.BUS},
    'vehicle.citroen.c3':
        {'x': 3.987684965133667, 'y': 1.8508483171463013, 'z': 1.6171096563339233, 'type': ObstacleType.CAR},
    'vehicle.diamondback.century':
        {'x': 1.6428436040878296, 'y': 0.3725162446498871, 'z': 1.0239027738571167, 'type': ObstacleType.BICYCLE},
    'vehicle.dodge.charger_2020':
        {'x': 5.006059646606445, 'y': 2.097083806991577, 'z': 1.5347249507904053, 'type': ObstacleType.CAR},
    'vehicle.audi.etron':
        {'x': 4.855708599090576, 'y': 2.0327565670013428, 'z': 1.6493593454360962, 'type': ObstacleType.CAR},
    'vehicle.nissan.micra':
        {'x': 3.633375883102417, 'y': 1.845113754272461, 'z': 1.50146484375, 'type': ObstacleType.CAR},
    'vehicle.gazelle.omafiets':
        {'x': 1.8354405164718628, 'y': 0.3289288878440857, 'z': 1.1256572008132935, 'type': ObstacleType.BICYCLE},
    'vehicle.lincoln.mkz_2017':
        {'x': 4.901683330535889, 'y': 2.128324270248413, 'z': 1.5107464790344238, 'type': ObstacleType.CAR},
    'vehicle.tesla.model3':
        {'x': 4.791779518127441, 'y': 2.163450002670288, 'z': 1.488319993019104, 'type': ObstacleType.CAR},
    'vehicle.lincoln.mkz_2020':
        {'x': 4.89238166809082, 'y': 2.230602979660034, 'z': 1.4801470041275024, 'type': ObstacleType.CAR},
    'vehicle.seat.leon':
        {'x': 4.1928300857543945, 'y': 1.8161858320236206, 'z': 1.4738311767578125, 'type': ObstacleType.CAR},
    'vehicle.bh.crossbike':
        {'x': 1.4872888326644897, 'y': 0.8592574596405029, 'z': 1.0795789957046509, 'type': ObstacleType.BICYCLE},
    'vehicle.yamaha.yzf':
        {'x': 2.2094459533691406, 'y': 0.8670341968536377, 'z': 1.2511454820632935, 'type': ObstacleType.MOTORCYCLE},
    'vehicle.harley-davidson.low_rider':
        {'x': 2.3557403087615967, 'y': 0.7636788487434387, 'z': 1.2765706777572632, 'type': ObstacleType.MOTORCYCLE},
    'vehicle.toyota.prius':
        {'x': 4.513522624969482, 'y': 2.006814479827881, 'z': 1.5248334407806396, 'type': ObstacleType.CAR},
    'vehicle.kawasaki.ninja':
        {'x': 2.0333523750305176, 'y': 0.8025798797607422, 'z': 1.1454535722732544, 'type': ObstacleType.MOTORCYCLE},
    'vehicle.nissan.patrol':
        {'x': 4.6045098304748535, 'y': 1.9315931797027588, 'z': 1.8548461198806763, 'type': ObstacleType.TRUCK},
    'vehicle.mini.cooper_s':
        {'x': 3.805800199508667, 'y': 1.97027587890625, 'z': 1.4750303030014038, 'type': ObstacleType.CAR},
    'vehicle.mercedes.coupe':
        {'x': 5.0267767906188965, 'y': 2.1515462398529053, 'z': 1.6355280876159668, 'type': ObstacleType.CAR},
    'vehicle.jeep.wrangler_rubicon':
        {'x': 3.866220712661743, 'y': 1.9051965475082397, 'z': 1.8779358863830566, 'type': ObstacleType.TRUCK}
}


def similar_by_length(length: float, width: float, height: float) -> str:
    """
    Returns Carla BluePrint name of the closest vehicle regarding length (second: width, third: height).

    :param length: Length of vehicle.
    :param width: Width of vehicle.
    :param height: Height of vehicle.
    :return: CARLA name of vehicle.
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_x': abs(length - current_best[1]['x']),
        'dif_y': abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for name, value in vehicle_dict.items():
        if abs(length - value['x']) < current_diff['dif_x']:
            current_best = (name, value)
            current_diff['dif_x'] = abs(length - current_best[1]['x'])
            current_diff['dif_y'] = abs(width - current_best[1]['y'])
            current_diff['dif_z'] = abs(height - current_best[1]['z'])
        if abs(length - value['x']) == current_diff['dif_x']:
            if abs(width - value['y']) < current_diff['dif_y']:
                current_best = (name, value)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
            if (abs(width - value['y']) == current_diff['dif_y']) & \
                    (abs(height - value['z']) <= current_diff['dif_z']):
                current_best = (name, value)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
    return current_best


def similar_by_width(length: float, width: float, height: float) -> str:
    """
    Returns Carla BluePrint name of the closest vehicle regarding width (second: length, third: height).

    :param length: Length of vehicle.
    :param width: Width of vehicle.
    :param height: Height of vehicle.
    :return: CARLA name of vehicle.
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_x': abs(length - current_best[1]['x']),
        'dif_y': abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for name, value in vehicle_dict.items():
        if abs(length - value['y']) < current_diff['dif_y']:
            current_best = (name, value)
            current_diff['dif_x'] = abs(length - current_best[1]['x'])
            current_diff['dif_y'] = abs(width - current_best[1]['y'])
            current_diff['dif_z'] = abs(height - current_best[1]['z'])
        if abs(length - value['y']) == current_diff['dif_y']:
            if abs(width - value['x']) < current_diff['dif_x']:
                current_best = (name, value)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
            if (abs(width - value['x']) == current_diff['dif_x']) & \
                    (abs(height - value['z']) <= current_diff['dif_z']):
                current_best = (name, value)
                current_diff['dif_x'] = abs(length - current_best[1]['x'])
                current_diff['dif_y'] = abs(width - current_best[1]['y'])
                current_diff['dif_z'] = abs(height - current_best[1]['z'])
    return current_best


def similar_by_area(length: float, width: float, height: float) -> str:
    """
    Returns Carla BluePrint name of the closest vehicle regarding area (length * width) (second: height).

    :param length: Length of vehicle.
    :param width: Width of vehicle.
    :param height: Height of vehicle.
    :return: CARLA name of vehicle.
    """
    current_best = list(vehicle_dict.items())[0]
    current_diff = {
        'dif_area': abs(length - current_best[1]['x']) * abs(length - current_best[1]['y']),
        'dif_z': abs(length - current_best[1]['z'])
    }
    for value in vehicle_dict.values():
        if abs(length * width - value['x'] * value['y']) < current_diff['dif_area']:
            current_best = value
            current_diff['dif_area'] = \
                abs(length - current_best[1]['x']) * abs(length - current_best[1]['y'])
            current_diff['dif_z'] = abs(length - current_best[1]['z'])
        if (abs(length * width - value['x'] * value['y']) == current_diff['dif_area']) \
                & (abs(height - value['z']) <= current_diff['dif_z']):
            current_best = value
            current_diff['dif_area'] = \
                abs(length - current_best[1]['x']) * abs(length - current_best[1]['y'])
            current_diff['dif_z'] = abs(length - current_best[1]['z'])
    return current_best
