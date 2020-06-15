import copy
import random
import config as cfg
from random import randrange
import json
import math
import numpy
import global_val
#import myGraphic

RESOLUTION = 2  # According to gen_advise.cpp

class LaneAdviser:
    def __init__(self):
            # Number of the boxes in one dimention
            self.num_di = RESOLUTION*(cfg.LANE_NUM_PER_DIRECTION*2+1)

            # 2D matrix of timing for advising
            self.timeMatrix = [[0]* self.num_di for i in range(self.num_di)]

            # Load the information of which boxes a trajectory will affect
            with open('./Roadrunner/advise_info/advise_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
                self.lane_dict = json.load(file)

            # Record the lane advice on each directions and initialize
            self.advised_lane = dict()
            for direction in range(4):
                self.advised_lane[(direction, 'L')] = direction*cfg.LANE_NUM_PER_DIRECTION
                self.advised_lane[(direction, 'R')] = direction*cfg.LANE_NUM_PER_DIRECTION + (cfg.LANE_NUM_PER_DIRECTION -1)
                self.advised_lane[(direction, 'S')] = (self.advised_lane[(direction, 'L')] + self.advised_lane[(direction, 'R')])//2

            self.count_lane_A_N_S_car_num = dict()
            for lane_id in range(4*cfg.LANE_NUM_PER_DIRECTION):
                self.count_lane_A_N_S_car_num[(lane_id, 'L')] = 0
                self.count_lane_A_N_S_car_num[(lane_id, 'R')] = 0
                self.count_lane_A_N_S_car_num[(lane_id, 'S')] = 0

    def copyMatrix(self):
        return copy.deepcopy(self.timeMatrix)

    # Update the table (final version for simulation usage)
    def updateTableFromCars(self, sched_car, advised_n_sched_car):
        self.resetTable()

        for car in sched_car:
            self.updateTable(car.lane, car.turning, car.arriving_time, self.timeMatrix)


        # Case 4: (case 1 + considering the halt) update with the latest car  "Using desired_lane!!!!!!!""
        for car in advised_n_sched_car:
            self.updateTable(car.lane, car.turning, car.position/cfg.MAX_SPEED, self.timeMatrix)


        # Count car number on each lane of advised but not scheduled
        for lane_id in range(4*cfg.LANE_NUM_PER_DIRECTION):
            self.count_lane_A_N_S_car_num[(lane_id, 'L')] = 0
            self.count_lane_A_N_S_car_num[(lane_id, 'R')] = 0
            self.count_lane_A_N_S_car_num[(lane_id, 'S')] = 0
        for car in advised_n_sched_car:
            self.count_lane_A_N_S_car_num[(car.lane, self.turn_to_str(car.turning))] += 1

        #myGraphic.gui.setTimeMatrix(self.timeMatrix)
    # Give lane advice to Cars
    def adviseLane(self, car):
        advise_lane = None
        # Sort out the LOTs and list the candidates
        start_lane = (car.lane//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION

        occup_time_list = [self.getMaxTime(start_lane+idx, car.turning, self.timeMatrix) for idx in range(cfg.LANE_NUM_PER_DIRECTION)]

        candidate_list = numpy.argsort(occup_time_list)

        # Get the shortest or the most ideal lane
        ideal_lane = None
        if car.turning == global_val.RIGHT_TURN:
            ideal_lane = start_lane+cfg.LANE_NUM_PER_DIRECTION-1
        elif car.turning == global_val.LEFT_TURN:
            ideal_lane = start_lane
        elif car.turning == global_val.STRAIGHT_TURN:
            # find one mid-lane with smallest LOTs
            if cfg.LANE_NUM_PER_DIRECTION > 2:
                ideal_lane = start_lane+candidate_list[0]
            else:
                for lane_idx in candidate_list:
                    if lane_idx != 0 and lane_idx != cfg.LANE_NUM_PER_DIRECTION:
                        ideal_lane = start_lane+lane_idx
                        break
        else:
            print(car.turning)
            print("Error: shouldn't happen")
            exit()

        advise_lane = ideal_lane

        # Scan through the candidates and see if we want to change our candidates
        # Get the cost of the ideal trajectory

        ideal_timeMatrix = self.copyMatrix()
        self.updateTableAfterAdvise(ideal_lane, car.turning, car.length, ideal_timeMatrix)
        ideal_others_LOT_list = dict()
        for key, lane_i in self.advised_lane.items():
            turn_i = key[1]
            ideal_others_LOT_list[key] = self.getMaxTime(lane_i, turn_i, ideal_timeMatrix)

        for lane_idx in candidate_list:
            candidate_lane = start_lane+lane_idx

            # The ideal lane has the smallest cost so far
            if start_lane+lane_idx == ideal_lane:
                break

            # see if the trajectory affects others
            candidate_timeMatrix = self.copyMatrix()
            self.updateTableAfterAdvise(candidate_lane, car.turning, car.length, candidate_timeMatrix)
            candidate_others_LOT_list = dict()
            for key, lane_i in self.advised_lane.items():
                turn_i = key[1]
                candidate_others_LOT_list[key] = self.getMaxTime(lane_i, turn_i, candidate_timeMatrix)

            # Compare the LOTs
            is_better = True
            for key, lane_i in self.advised_lane.items():
                turn_i = key[1]
                # Skip the candidate lanes, because obviously it will increase
                if lane_i == int(candidate_lane):
                    continue
                elif self.count_lane_A_N_S_car_num[(lane_i, turn_i)] > 0:
                    if candidate_others_LOT_list[key] > ideal_others_LOT_list[key]:
                        is_better = False
                        break

            if not is_better:
                continue
            else:
                advise_lane = candidate_lane
                break

        # The lane is chosen, update the matrix update giving the lane advice
        #self.updateTableAfterAdvise(advise_lane, car.turning, car.length, self.timeMatrix)
        # Record the given lane
        self.advised_lane[(car.lane//cfg.LANE_NUM_PER_DIRECTION, self.turn_to_str(car.turning))] = advise_lane
        # Change the exact index to the lane index of one direction
        advise_lane = advise_lane%cfg.LANE_NUM_PER_DIRECTION
        #myGraphic.gui.setTimeMatrix(self.timeMatrix)
        #myGraphic.gui.setAdviseMatrix(self.advised_lane)

        return cfg.LANE_NUM_PER_DIRECTION-advise_lane-1 # The index of SUMO is reversed
        #'''


    def getMaxTime(self, lane, turning, timeMatrix):
        # get the earliest time when a car can drive
        # through lane "lane" in direction "direction".

        # lane : int
        # direction : string
        direction = self.turn_to_str(turning)
        temp = []
        quotient = lane // cfg.LANE_NUM_PER_DIRECTION

        if quotient == 0:
            # no rotation
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e['X']][e['Y']])

        elif quotient == 1:
            # rotate 90 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[self.num_di-1 - e['Y']][e['X']])

        elif quotient == 2:
            # rotate 180 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[self.num_di-1 - e['X']][self.num_di-1 - e['Y']])

        else:  # quotient == 4
            # rotate 270 degree clockwise\
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                temp.append(timeMatrix[e['Y']][self.num_di-1 - e['X']])

        return max(temp)



    def updateTable(self, lane, turning, time, timeMatrix):
        # Update all the squares on the trajectory "lane + direction" to "time".
        quotient = (lane) // cfg.LANE_NUM_PER_DIRECTION
        direction = self.turn_to_str(turning)

        if quotient == 0:
            # no rotation
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e['X']][e['Y']]):
                    timeMatrix[e['X']][e['Y']] = time


        elif quotient == 1:
            # rotate 90 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[self.num_di-1 - e['Y']][e['X']]):
                    timeMatrix[self.num_di-1 - e['Y']][e['X']] = time


        elif quotient == 2:
            # rotate 180 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[self.num_di-1 - e['X']][self.num_di-1 - e['Y']]):
                    timeMatrix[self.num_di-1 - e['X']][self.num_di-1 - e['Y']] = time


        else:  # quotient == 3
            # rotate 270 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                if (time > timeMatrix[e['Y']][self.num_di-1 - e['X']]):
                    timeMatrix[e['Y']][self.num_di-1 - e['X']] = time


    def updateTableAfterAdvise(self, lane, turning, car_length, timeMatrix):

        # Update all the squares on the trajectory "lane + direction" to "diff_time".
        quotient = lane // cfg.LANE_NUM_PER_DIRECTION
        direction = self.turn_to_str(turning)
        speed = None

        if direction == 'S':
            speed = cfg.MAX_SPEED
        else:
            speed = cfg.TURN_SPEED

        if quotient == 0:
            # lane 0 or 1, no rotation
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*cfg.LANE_WIDTH/speed + car_length/speed
                timeMatrix[e['X']][e['Y']] += time

        elif quotient == 1:
            # lane 2 or 3, rotate 90 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*cfg.LANE_WIDTH/speed + car_length/speed
                timeMatrix[self.num_di-1 - e['Y']][e['X']] += time

        elif quotient == 2:
            # lane 4 or 5, rotate 180 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*cfg.LANE_WIDTH/speed + car_length/speed
                timeMatrix[self.num_di-1 - e['X']][self.num_di-1 - e['Y']] += time

        else:  # quotient == 3
            # lane 6 or 7, rotate 270 degree clockwise
            for e in self.lane_dict[str(lane % cfg.LANE_NUM_PER_DIRECTION) + direction]:
                time = e['distance']*cfg.LANE_WIDTH/speed + car_length/speed
                timeMatrix[e['Y']][self.num_di-1 - e['X']] += time


    # Reset the records
    def resetTable(self):
        self.timeMatrix = [[0] * self.num_di for i in range(self.num_di)]


    def turn_to_str(self, turn):
        turn_str = ""
        if turn == global_val.LEFT_TURN:
            turn_str = "L"
        elif turn == global_val.STRAIGHT_TURN:
            turn_str = "S"
        elif turn == global_val.RIGHT_TURN:
            turn_str = "R"
        else:
            turn_str = turn

        return turn_str
