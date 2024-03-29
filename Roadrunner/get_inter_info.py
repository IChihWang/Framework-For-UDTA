import json
import copy

import config as cfg
import global_val


#def main():
    #ans = getconflictregion(1, "s", 2, "r")
    #print (ans[0][1])


class Data:
    def __init__(self):
        with open('./Roadrunner/inter_info/lane_info'+str(cfg.LANE_NUM_PER_DIRECTION)+'.json', 'r') as file:
            self.tau = json.load(file)



    def getConflictRegion(self, car1, car2):
        reverse_flag = False


        if (car1.lane > car2.lane):
            car1, car2 = car2, car1
            reverse_flag = True

        l1 = car1.lane
        l2 = car2.lane

        # rotate to corresponding cases
        if l1 >= cfg.LANE_NUM_PER_DIRECTION:
            l2 = (l2 - (l1//cfg.LANE_NUM_PER_DIRECTION)*cfg.LANE_NUM_PER_DIRECTION) % (cfg.LANE_NUM_PER_DIRECTION*4)
            l1 = l1 % cfg.LANE_NUM_PER_DIRECTION


        # Test if there is conflict between l1 & l2
        # and give the conflict region.

        test_str = str(l1) + self.turn_to_str(car1.turning) + str(l2) + self.turn_to_str(car2.turning)

        if (test_str) in self.tau:
            val = self.tau[test_str].copy()

            # 0. Flip Xd, Yd if two car turn left from opposite directions
            if (l2 >= cfg.LANE_NUM_PER_DIRECTION*2 and l2 < cfg.LANE_NUM_PER_DIRECTION*3) and (self.turn_to_str(car1.turning) == 'L' and self.turn_to_str(car2.turning) == 'L'):
                val['Xm'], val['Xd'] = val['Xd'], val['Xm']


            # 1. Compute tau_S1_S2
            tau_S1_S2 = None
            # --- case 1: from Xm, Ym
            ans1 = (val['Xm']+car1.length+cfg.HEADWAY)/car1.speed_in_intersection - (val['Ym'])/car2.speed_in_intersection+cfg.DISTANCE*cfg.LANE_WIDTH/car1.speed_in_intersection

            # --- case 2: from Xd, Yd
            ans2 = (val['Xd']+car1.length+cfg.HEADWAY)/car1.speed_in_intersection - (val['Yd'])/car2.speed_in_intersection+cfg.DISTANCE*cfg.LANE_WIDTH/car1.speed_in_intersection

            tau_S1_S2 = max(ans1, ans2)




            # 2. Compute tau_S2_S1
            tau_S2_S1 = None
            # --- case 1: from Xm, Ym
            ans1 = (val['Ym']+car2.length+cfg.HEADWAY)/car2.speed_in_intersection - (val['Xm'])/car1.speed_in_intersection+cfg.DISTANCE*cfg.LANE_WIDTH/car2.speed_in_intersection

            # --- case 2: from Xd, Yd
            ans2 = (val['Yd']+car2.length+cfg.HEADWAY)/car2.speed_in_intersection - (val['Xd'])/car1.speed_in_intersection+cfg.DISTANCE*cfg.LANE_WIDTH/car2.speed_in_intersection

            tau_S2_S1 = max(ans1, ans2)

            ans = [tau_S1_S2, tau_S2_S1]



            if (reverse_flag):
                ans[0], ans[1] = ans[1], ans[0]

            # tau_S1_S2: car1 is in front of car2
            return ans
        else:
            return []

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
