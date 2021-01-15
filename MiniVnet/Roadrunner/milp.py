
from get_inter_info import Data
from ortools.linear_solver import pywraplp
import config as cfg
import copy
import global_val
import numpy


data = Data()

def Roadrunner(old_cars, new_cars, delay_list, OT_list):
    # part 1: calculate OT
    for c_idx in range(len(new_cars)):
        OT = new_cars[c_idx].position/cfg.MAX_SPEED
        OT_list[new_cars[c_idx]] = OT + cfg.SUMO_TIME_ERR

    # part 2: build the solver
    solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)






    # part 3: claim parameters
    for c_idx in range(len(new_cars)):
        if new_cars[c_idx].turning == global_val.STRAIGHT_TURN:
            delay_list[new_cars[c_idx]] = solver.NumVar(0, solver.infinity(), 'd'+str(c_idx))
        else:
            min_d = (2*cfg.CCZ_DEC2_LEN/(cfg.MAX_SPEED+cfg.TURN_SPEED)) - (cfg.CCZ_DEC2_LEN/cfg.MAX_SPEED)
            delay_list[new_cars[c_idx]] = solver.NumVar(min_d, solver.infinity(), 'd'+str(c_idx))

    # part 4: set constrain (10)
    all_cars = old_cars+new_cars
    for c_idx in range(len(all_cars)):
        for c_jdx in range(c_idx+1, len(all_cars)):
            if (all_cars[c_idx].lane == all_cars[c_jdx].lane):


                if (type(delay_list[all_cars[c_jdx]])==float or type(delay_list[all_cars[c_idx]])==float):
                    if (type(delay_list[all_cars[c_idx]])!=float and type(delay_list[all_cars[c_jdx]])==float):
                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection + (OT_list[all_cars[c_jdx]]+delay_list[all_cars[c_jdx]])
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == global_val.STRAIGHT_TURN and all_cars[c_jdx].turning != global_val.STRAIGHT_TURN:
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - OT_list[all_cars[c_idx]]
                        tmp_conts = solver.Constraint(bound,solver.infinity())
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_idx]], 1)
                        #print("eq1-1 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (type(delay_list[all_cars[c_idx]])==float and type(delay_list[all_cars[c_jdx]])!=float):
                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + (OT_list[all_cars[c_idx]]+delay_list[all_cars[c_idx]])
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == global_val.STRAIGHT_TURN and all_cars[c_idx].turning != global_val.STRAIGHT_TURN:
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        bound = bound - OT_list[all_cars[c_jdx]]
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_jdx]], 1)
                        #print("eq1-2 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])

                elif (type(delay_list[all_cars[c_jdx]])!=float or type(delay_list[all_cars[c_idx]])!=float):
                    if (OT_list[all_cars[c_idx]] > OT_list[all_cars[c_jdx]]):


                        bound = all_cars[c_jdx].length/all_cars[c_jdx].speed_in_intersection - OT_list[all_cars[c_idx]]+OT_list[all_cars[c_jdx]]
                        bound += cfg.HEADWAY/all_cars[c_jdx].speed_in_intersection
                        if all_cars[c_idx].turning == global_val.STRAIGHT_TURN and all_cars[c_jdx].turning != global_val.STRAIGHT_TURN:
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_idx]], 1)
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_jdx]], -1)
                        #print("eq1-3 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])
                    elif (OT_list[all_cars[c_idx]] < OT_list[all_cars[c_jdx]]):

                        bound = all_cars[c_idx].length/all_cars[c_idx].speed_in_intersection + OT_list[all_cars[c_idx]]-OT_list[all_cars[c_jdx]]
                        bound += cfg.HEADWAY/all_cars[c_idx].speed_in_intersection
                        if all_cars[c_jdx].turning == global_val.STRAIGHT_TURN and all_cars[c_idx].turning != global_val.STRAIGHT_TURN:
                            bound += (cfg.MAX_SPEED-cfg.TURN_SPEED)*(cfg.CCZ_DEC2_LEN)/(cfg.MAX_SPEED*(cfg.MAX_SPEED+cfg.TURN_SPEED))
                        tmp_conts = solver.Constraint(bound, solver.infinity())
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_idx]], -1)
                        tmp_conts.SetCoefficient(delay_list[all_cars[c_jdx]], 1)
                        #print("eq1-4 ", bound, all_cars[c_idx]['ID'], all_cars[c_jdx]['ID'])



    # part 5: set constrain (11)
    for c_idx in range(len(new_cars)):
        for c_jdx in range(c_idx+1, len(new_cars)):

            if (new_cars[c_idx].lane == new_cars[c_jdx].lane):
                continue


            #ans = data.getConflictRegion(new_cars[c_idx].lane, new_cars[c_idx].turning, new_cars[c_jdx].lane, new_cars[c_jdx].turning)
            ans = data.getConflictRegion(new_cars[c_idx], new_cars[c_jdx])


            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]





                flag = solver.IntVar(0, 1, 'flag'+str(c_idx)+'_'+str(c_jdx))

                bound = -OT_list[new_cars[c_jdx]] + OT_list[new_cars[c_idx]] + tau_S1_S2 - cfg.LARGE_NUM
                tmp_conts2 = solver.Constraint(bound, solver.infinity())
                tmp_conts2.SetCoefficient(delay_list[new_cars[c_idx]], -1)
                tmp_conts2.SetCoefficient(delay_list[new_cars[c_jdx]], 1)
                tmp_conts2.SetCoefficient(flag, -cfg.LARGE_NUM)
                #print("eq2-1 ", bound+ cfg.LARGE_NUM, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'], c_idx, c_jdx)

                bound = -OT_list[new_cars[c_idx]] + OT_list[new_cars[c_jdx]] + tau_S2_S1
                tmp_conts1 = solver.Constraint(bound, solver.infinity())
                tmp_conts1.SetCoefficient(delay_list[new_cars[c_idx]], 1)
                tmp_conts1.SetCoefficient(delay_list[new_cars[c_jdx]], -1)
                tmp_conts1.SetCoefficient(flag, cfg.LARGE_NUM)
                #print("eq2-2 ", bound, new_cars[c_idx]['ID'], new_cars[c_jdx]['ID'])

    #'''
    # part 6: set constrain (12)
    for nc_idx in range(len(new_cars)):
        for oc_idx in range(len(old_cars)):

            if (new_cars[nc_idx].lane == old_cars[oc_idx].lane):
                continue



            #ans = data.getConflictRegion(new_cars[nc_idx].lane, new_cars[nc_idx].turning, old_cars[oc_idx].lane, old_cars[oc_idx].turning)

            ans = data.getConflictRegion(new_cars[nc_idx], old_cars[oc_idx])





            if (len(ans) > 0):
                tau_S1_S2 = ans[0]
                tau_S2_S1 = ans[1]

                '''
                if (old_cars[oc_idx]['ID'] == 'L_27') and (new_cars[nc_idx]['ID'] == 'L_32'):
                    print(ans)
                    print(new_cars[nc_idx]['ID'], new_cars[nc_idx].lane, new_cars[nc_idx].turning, OT_list[new_cars[nc_idx]])
                    print(old_cars[oc_idx]['ID'], old_cars[oc_idx].lane, old_cars[oc_idx].turning, OT_list[old_cars[oc_idx]])
                '''

                flag = solver.IntVar(0, 1, 'flagg'+str(nc_idx)+"_"+str(oc_idx))

                bound = -delay_list[old_cars[oc_idx]]-OT_list[old_cars[oc_idx]] + OT_list[new_cars[nc_idx]] + tau_S1_S2 - cfg.LARGE_NUM
                tmp_conts3 = solver.Constraint(bound, solver.infinity())
                tmp_conts3.SetCoefficient(delay_list[new_cars[nc_idx]], -1)
                tmp_conts3.SetCoefficient(flag, -cfg.LARGE_NUM)

                bound = delay_list[old_cars[oc_idx]] + OT_list[old_cars[oc_idx]] - OT_list[new_cars[nc_idx]] + tau_S2_S1
                tmp_conts4 = solver.Constraint(bound, solver.infinity())
                tmp_conts4.SetCoefficient(delay_list[new_cars[nc_idx]], 1)
                tmp_conts4.SetCoefficient(flag, cfg.LARGE_NUM)
                #print("eq3-2 ", bound, new_cars[nc_idx]['ID'], old_cars[oc_idx]['ID'])
    #'''


    # part 7: set objective
    objective = solver.Objective()

    for c_idx in range(len(new_cars)):
        objective.SetCoefficient(delay_list[new_cars[c_idx]], 1)
    objective.SetMinimization()

    '''
    print('Car num', len(new_cars))
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    #'''

    # part 8: Solve the problem
    sol_status = solver.Solve()


    if (sol_status == 2):
        # Unfeasible
        #print ([car.position for car in old_cars])
        #print ([car.position for car in new_cars])
        print("Error: no fesible solution")
        exit()

    # Record the results
    delay_results = dict()
    for car in new_cars:
        delay_results[car.id] = delay_list[car].solution_value()

    return delay_results
