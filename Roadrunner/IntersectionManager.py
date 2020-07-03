
import sys
import config as cfg
import threading
import random


from basic_graph import Car
from milp import Roadrunner
from LaneAdviser import LaneAdviser
import global_val


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
        turning_delay = dict()
        lane_results = dict()

        # Classify the cars for scheduler
        sched_car = []      # Scheduled
        n_sched_car = []    # Not scheduled
        advised_n_sched_car = []

        delay_list = dict() # (car, delay)
        OT_list = dict() # (car, OT)

        target_car = cars[index_of_target_car]

        for car_idx in range(len(cars)):
            car = cars[car_idx]
            if car.position != None:
                delay_list[car] = None
                OT_list[car] = None
                n_sched_car.append(car)

                # Cars given lane advice but not scheduled
                if car_idx != index_of_target_car and car.position < target_car.position:
                    advised_n_sched_car.append(car)
            else:
                delay_list[car] = car.arriving_time
                OT_list[car] = 0
                sched_car.append(car)


        # Update the table for the lane advising
        self.lane_advisor.updateTableFromCars(sched_car, advised_n_sched_car)

        for turning in [global_val.RIGHT_TURN, global_val.STRAIGHT_TURN, global_val.LEFT_TURN]:
            # Assign the turning to the car
            target_car.turning = turning
            target_car.speed_in_intersection = self.get_speed_in_intersection(turning)

            # Line advise
            if len(sched_car) == 0 and len(n_sched_car) == 1:
                # Stay on the lane
                pass
            else:
                advised_lane = self.lane_advisor.adviseLane(target_car)
                target_car.lane = advised_lane

            # Reset the delays
            for car in n_sched_car:
                delay_list[car.id] = None

            # Do the scheduling
            delay_results = dict()
            if len(sched_car) == 0 and len(n_sched_car) == 1:
                if turning == global_val.STRAIGHT_TURN:
                    delay_results[target_car.id] = 0
                else:
                    delay_results[target_car.id] = 0.012458170110189348     # Decelerate for turning
            else:
                delay_results = Roadrunner(sched_car, n_sched_car, delay_list, OT_list)

            turning_delay[turning] = delay_results
            lane_results[turning] = target_car.lane

        return (turning_delay, lane_results)

    def get_speed_in_intersection(self, turn):
        turn_speed = 0
        if turn == global_val.STRAIGHT_TURN:
            turn_speed = global_val.MAX_SPEED
        else:
            turn_speed = global_val.TURN_SPEED

        return turn_speed
