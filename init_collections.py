from itertools import cycle
import time
import json
from myconfig.system_config import cube_setting
from battery_module.battery_station_controller import createChargingStation
from model.skycars import createSkycar
from model.binbuilder import init_bins
from model.cubebuilder import create_cube_map
from model.displayinfo import publish_info
from collections import deque
from communication_provider.webserver.controller import cube_setup, config
from myconfig.simulation_config import simulation
from termcolor import colored, cprint
from initialisation import stationSetup
import networkx as nx
from util.system_type import CycleStop
from execution import obstacleController

skycars = dict()
stations = dict()
battery_stations = dict()
bins = dict()
cube = dict()
obstacles = dict()
automateOrder = dict()
stationNeighbor = dict()
cycleStationNeighbor = dict()

optimalPutawayCoor = dict()
optimalPutawayCoor[1] = cycle([
    '34,8',
    '33,8',
    '32,8',
    '31,7',
    '28,9',
    '29,8',
    '34,3',
    '33,3',
    '32,3',
    '31,3',
    '30,8',
    '31,7',
    '32,8',
    '33,9',
])
optimalPutawayCoor[2] = cycle([
    '30,17',
    '31,18',
    '34,18',
    '33,19',
    '28,19',
    '29,18',
    '30,18',
    '31,18',
    '28,19',
    '29,18',
    '30,19',
    '33,19',
    '32,18',
    '33,20',
])
optimalPutawayCoor[3] = cycle([
    '33,28',
    '32,27',
    '32,28',
    '33,29',
    '28,29',
    '29,28',
    '30,27',
    '31,28',
    '28,30',
    '29,29',
    '30,30',
    '31,27',
    '32,29',
    '33,30',
])
optimalPutawayCoor[4] = cycle([
    '33,39',
    '32,37',
    '32,39',
    '33,38',
    '28,40',
    '29,38',
    '30,38',
    '31,38',
    '28,39',
    '29,37',
    '30,37',
    '31,37',
    '32,38',
    '33,37',
])
optimalPutawayCoor[5] = cycle([
    '30,48',
    '31,47',
    '32,50',
    '33,51',
    '28,52',
    '29,49',
    '30,47',
    '31,48',
    '28,49',
    '29,45',
    '30,52',
    '31,53',
    '32,47',
    '33,49',
])

setupReady = False

total_bin_to_station = dict()
total_orders_completed = dict()
operator_working_time = dict()
orders = dict()              


total_orders_completed.update({'Ricardo': [], 'Bob': [], 'Liang': []})
operator_working_time.update({'Ricardo': [], 'Bob': [], 'Liang': []})
maintenance_dock = dict()  # if not using, put as None

detectChanges = dict({"bins": {"changed": False, "bin_nos": []}, "adg": False, "skycar": False})

# pathfinding adg
adg_combined = nx.DiGraph()  # instantiate adg to be pass to pathfinding

# todo , resume job from DB

# recovery and error handling
hwXReconnectFlag = 0  


def warehouseSetup(mqtt_subscribe):
    ''' this function init all require object 
    1. from sm
    2. from faker '''

    global skycars, stations, bins, cube, battery_stations, setupReady, maintenance_dock

    if simulation["sm"] is True:

        battery_stations = createChargingStation()
        stations = stationSetup.createStation()
        cube, maintenance_dock = create_cube_map()
        skycars = createSkycar()

        bins = init_bins(False, cube, stations)

        # mockCoordinate = [(6, 5, 3)]
        # i = 0
        # for b in bins.values():
        #     b.coordinate.x = mockCoordinate[i][0]
        #     b.coordinate.y = mockCoordinate[i][1]
        #     b.coordinate.z = mockCoordinate[i][2]
        #     i += 1
        #     print(f"{b.coordinate} - {b.ID}")

        # bins["DRYRUN-BIN"] = bins['MY001-TB0001']
        # del bins['MY001-TB0001']
        # bins["DRYRUN-BIN"].ID = "DRYRUN-BIN"
        # bins["DRYRUN-BIN"].coordinate.x = 5
        # bins["DRYRUN-BIN"].coordinate.y = 5
        # bins["DRYRUN-BIN"].coordinate.z = 3

        obstacleController.initObstacle(skycars)
        print(skycars)


        # RESERVED COORDINATE
        # for x in range(9):
        #     obstacleController.seedObstacle(x,0)

        msg = f"Cube Setup Completed, Dimension x:{cube['x']} , y:{cube['y']} , z:{cube['z']} Bins :{len(bins)} Stations :{len(stations)} Charging Stations :{len(battery_stations)} MaintenanceDock :{len(maintenance_dock)}"
        cprint(msg, 'blue', attrs=["bold"])

        # mqtt_subscribe()

    else:
        x, y, z, totebins = cube_setup.warehouseSetup()

        bins = totebins
        # for b in bins.values():
        #     # print(b)

        battery_stations = createChargingStation()
        cube, maintenance_dock = create_cube_map(x, y, z)
        stations = stationSetup.createStation()
        skycars = createSkycar()

        obstacleController.initObstacle(skycars)

        msg = f"Cube Setup Completed, Dimension x:{x} , y:{y} , z:{z} Bins :{len(bins)} Stations :{len(stations)} Charging Stations :{len(battery_stations)} MaintenanceDock :{len(maintenance_dock)}"
        cprint(msg, 'blue', attrs=["bold"])

        # mqtt_subscribe()

    setupReady = True
