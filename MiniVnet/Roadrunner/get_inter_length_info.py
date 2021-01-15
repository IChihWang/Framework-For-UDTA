import json
import copy

import config as cfg
import global_val


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])


class Data:
    def __init__(self):
        with open('./Roadrunner/inter_length_info/inter_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.length_dict = json.load(file)



    def getIntertime(self, in_lane, in_turn):

        test_str = str(in_lane) + self.turn_to_str(in_turn)
        length = self.length_dict[test_str]
        
        time = None
        if in_turn == global_val.STRAIGHT_TURN:
            time = length/global_val.MAX_SPEED
        else:
            time = length/global_val.TURN_SPEED

        return time

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
