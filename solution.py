#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
import numpy as np
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

WALL = 10000
START_PT = 5000
#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    moves = 0
    for box in state.boxes.keys():
        set_list = []
        num = state.boxes[box]
        x1, y1 = box
        min_d = 3000
        if (state.restrictions != None) and (num < len(state.restrictions)):
            for rest in state.restrictions[num]:
                x, y = rest
                dis = abs(x-x1) + abs(y-y1)
                if dis < min_d:
                    min_d = dis
            moves += min_d
        else:
            for s in state.storage:
                x, y = s
                dis = abs(x-x1) + abs(y-y1)
                if dis < min_d:
                    min_d = dis
            moves += min_d
    return moves

def heur_alternate(state):

    # state.print_state()
	#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    cost = 0
    if state.parent == None:
        create_matrix(state)
        # print(matrix)
    for box in state.boxes:
        min_cost = 10000
        x, y = box
        restrict_list = []
        if state.restrictions != None:
            if state.restrictions[state.boxes[box]] != None:
                storage_cor = state.restrictions[state.boxes[box]]
                for i in storage_cor:
                    if i in state.storage.keys():
                        storage_num = mat_stor[i]
                        restrict_list.append(storage_cor)
                        cur_cost = matrix[storage_num][y][x]
                        if cur_cost == 0:
                            cur_cost = WALL
                        if cur_cost == START_PT:
                            cur_cost = 0
                        if min_cost > cur_cost:
                            min_cost = cur_cost
        else:
            for stor in state.storage:
                if not stor in restrict_list:
                    storage_num = mat_stor[stor]
                    cur_cost = matrix[storage_num][y][x]
                    if cur_cost == 0:
                        cur_cost = WALL
                    if cur_cost == START_PT:
                        cur_cost = 0
                    if min_cost > cur_cost:
                        min_cost = cur_cost
        # if min_cost == WALL:
        #     return WALL
        cost += min_cost
    return cost

def create_matrix(state):
    global matrix
    matrix = np.zeros((len(state.storage),state.height, state.width))
    storage_num = 0
    global mat_stor
    mat_stor = {}
    for item in state.storage:
        mat_stor[item] = storage_num
        for i in state.obstacles:
            x, y = i
            matrix[storage_num][y][x] = WALL
        expanded = []
        x_1, y_1 = item
        matrix[storage_num][y_1][x_1] = START_PT
        expanded.append(item)
        dis(matrix, storage_num, x_1, y_1, state, int(0), expanded)
        storage_num += 1

def dis(matrix, storage_num, x, y, state, cost, expanded):
    if x < (state.width - 2) and check_empty(storage_num, x+1, y) and check_empty(storage_num, x+2, y):
        if matrix[storage_num][y][x+1] == 0 or matrix[storage_num][y][x+1] > cost+1:
            expanded.append((x+1,y))
            matrix[storage_num][y][x+1] = cost+1
    if x > 1 and check_empty(storage_num, x-1, y) and check_empty(storage_num, x-2, y):
        if matrix[storage_num][y][x-1] == 0 or matrix[storage_num][y][x-1] > cost+1:
            expanded.append((x-1, y))
            matrix[storage_num][y][x-1] = cost+1
    if y < (state.height - 2) and check_empty(storage_num, x, y+1) and check_empty(storage_num, x, y+2):
        if matrix[storage_num][y+1][x] == 0 or matrix[storage_num][y+1][x] > cost+1:
            expanded.append((x,y+1))
            matrix[storage_num][y+1][x] = cost+1
    if y > 1 and check_empty(storage_num, x, y-1) and check_empty(storage_num, x, y-2):
        if matrix[storage_num][y-1][x] == 0 or matrix[storage_num][y-1][x] > cost+1:
            expanded.append((x,y-1))
            matrix[storage_num][y-1][x] = cost+1
    # print(matrix)
    expanded.pop(0)
    if (len(expanded) != 0):
        x, y = expanded[0]
        dis(matrix, storage_num, x, y, state, cost+1, expanded)
		
		
# def check_box_trapped(state):

		
		
# def trapped(state, x, y, check):
#     num = 0
#     sur = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)]
#     flag = [0, 0, 0, 0]
#     for i in sur:
#         if i in state.obstacles or i in check:
#             flag[num] = 1
#             if flag[num - 1] == 1:
#                 return True
#             elif num == 3 and flag[0] == 1:
#                 return True
#         num += 1
#     return False

def check_empty(storage_num, x, y):
    if matrix[storage_num][y][x] == WALL:
        return False
    return True

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    return sN.gval + weight*sN.hval

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    start_time = os.times()[0]
    se = SearchEngine("best_first", "full")
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    goal = se.search(timebound, None)
    cost = 10000
    final_goal = goal
    while ((os.times()[0] - start_time) < timebound):
        if goal == False: #didn't find a solution
            se.init_search(initial_state, sokoban_goal_state, heur_fn)
        else:
            se.init_search(initial_state, sokoban_goal_state, heur_fn)
        if cost >= goal.gval:
            cost = goal.gval
            final_goal = goal
    return final_goal

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    start_time = os.times()[0]
    start_weight = 20
    wrapped_fval_function = (lambda sN: fval_function(sN, start_weight))
    se = SearchEngine("astar", "full")
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    goal = se.search(timebound, None)
    cost = 10000
    final_goal = goal
    while ((os.times()[0] - start_time) < timebound):
        if goal == False: #didn't find a solution
            wrapped_fval_function = (lambda sN: fval_function(sN, start_weight * 2))
            se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        else:
            wrapped_fval_function = (lambda sN: fval_function(sN, start_weight / 4))
            se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
        if cost >= goal.gval:
            cost = goal.gval
            final_goal = goal
    return final_goal

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")
  print("Running A-star")

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit
  print("Running Anytime Weighted A-star")

  for i in range(0, 10):
    print("*************************************")
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)
    counter += 1

  if counter > 0:
    percent = (solved/counter)*100

  print("*************************************")
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))
  print("*************************************")
