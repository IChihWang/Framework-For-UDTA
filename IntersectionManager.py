
import sys
import config as cfg
import traci
import threading
import random


from Cars import Car
from milp import Icacc, IcaccPlus, Fcfs, FixedSignal
from LaneAdviser import LaneAdviser


class IntersectionManager:
    def __init__(self, my_id):
        self.ID = my_id
        self.az_list = dict()
        self.pz_list = dict()
        self.ccz_list = dict()

        self.car_list = dict()   # Cars that needs to be handled
        self.cc_list = dict()    # Cars under Cruse Control in CCZ

        self.schedule_period_count = 0
        self.lane_advisor = LaneAdviser()
        self.scheduling_thread = None
        self.in_lanes = []
        self.out_lanes = []

        # For front car
        self.CC_last_cars_on_lanes = dict()
        for idx in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.CC_last_cars_on_lanes[idx] = None


        # Statistics
        self.total_delays = 0
        self.total_delays_by_sche = 0
        self.car_num = 0

        self.total_fuel_consumption = 0
        self.fuel_consumption_count = 0


        self.set_round_lane()


    def set_round_lane(self):
        for idx in range(1,5):
            for jdx in range(cfg.LANE_NUM_PER_DIRECTION):
                idx_str = self.ID + '_' + str(idx)+'_'+str(jdx)
                self.in_lanes.append(idx_str)

    def check_in_my_region(self, lane_id):
        if lane_id in self.in_lanes:
            return True
        else:
            return False


    def run(self, cars, index_of_target_car):

        # Cars: list of cars around the intersection
        # index_of_target_car: index of the target car in the list

        # The result is the candidate for turnings
        turning_results = dict()

        # Classify the cars for scheduler
        sched_car = []      # Scheduled
        n_sched_car = []    # Not scheduled

        for car_idx in range(len(cars)):
            car = cars[car_idx]
            if car.AT == None:
                car.OT = None       # Format the data for the scheduler
                car.D = None
                n_sched_car.append(car)

                # Cars given lane advice but not scheduled
                if car_idx != index_of_target_car:
                    advised_n_sched_car.append(car)
            else:
                car.OT = 0          # Format the data for the scheduler
                car.D = car.AT
                sched_car.append(car)


        # Update the table for the lane advising
        self.lane_advisor.updateTableFromCars(sched_car, advised_n_sched_car)

        for turning_str in ['R', 'S', 'L']:

            # Assign the turning to the car
            cars[index_of_target_car].turning = turning_str

            # Line advise
            for n_car in n_sched_car:
                advised_lane = self.lane_advisor.adviseLane(n_car)
                n_car.lane = advised_lane

            # Reset the delays
            for c_idx in range(len(n_sched_car)):
                n_sched_car[c_idx].D = None

            # Do the scheduling
            IcaccPlus(sched_car, n_sched_car)

            for car in cars:
                if car.AT == None and car not in n_sched_car:
                    n_sched_car.append(car)
                elif car.AT != None and car not in sched_car:
                    sched_car.append(car)
                    
            turning_results[turning_str] = [car.AT for car in cars]
            lane_results[turning_str] = [car.lane for car in n_sched_car]
