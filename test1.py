# -*- coding: utf-8 -*-
"""
A star for any location goal.
"""
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '../..'))
from heapq import heappop, heappush
from itertools import count
from collections import namedtuple
from util.system_type import ObstacleType
from myconfig.system_config import cube_setting
import traceback
import init_collections as db


def a_star(start, goal, skycar, constraints=None):
    """The main A* search

    Requires the following classes to be defined:
    - State
    - A_Star_Queue

    Requires the following functions to be defined:
    - heuristic(state1, state2)
    - is_solution(state, goal)
    - get_neighbours(state)

    ****Flow*****
    Outer loop
            - openlist
    Inner loop (n2)
            - let neighbours = n
            - n1 = unfiltered 
            - n2 = filtered
    """
    # if not is_valid(goal):
    #     raise ValueError(f"{goal} is not a valid goal.")

    open_list = A_Star_Queue()
    closed = set()
    came_from = {}
    g_scores = {}
    f_scores = {}

    g = 0
    h = heuristic(start, goal)
    f = g + h
    open_list.add(start, f, g, h)
    came_from[start] = None

    g_scores[start] = g
    f_scores[start] = f

    iterations = 0
    while len(open_list) != 0:
        current_state, current_f, current_g, current_h = open_list.pop()
        closed.add(current_state)

        # print(f"{iterations} {current_state} f = {current_f} g = {current_g} h = {current_h}", flush = True)

        if is_solution(current_state, goal):
            # print("Done", iterations)
            cost = current_g
            path = []
            while came_from[current_state] is not None:
                path.append(current_state)
                current_state = came_from[current_state]
            path.append(current_state)
            path.reverse()
            # for p in path:
            # print(p)
            # print(iterations)
            return path, cost

        neighbours = get_neighbours(current_state, skycar)

        if constraints is not None:
            neighbours = check_constraints(came_from[current_state], current_state, neighbours, constraints, start, goal)

        for neighbour in neighbours:
            if neighbour in closed:
                continue

            g = g_scores[current_state] + 1
            # g = gscore(current_state, neighbour, g_scores, came_from)

            # if arriving at solution before goal time, waiting costs 0
            # NOTE: does this break the consistency of the heurisitic?
            if (is_solution_except_time(current_state, goal)
                    and neighbour.equal_except_time(current_state)):
                g = g_scores[current_state] + 1
                # g = gscore(current_state, neighbour, g_scores, came_from)

            if neighbour in g_scores:
                # print('Already explored', current_state, neighbour, g, g_scores[neighbour])
                if g < g_scores[neighbour]:
                    h = heuristic(neighbour, goal)
                    f = g + h
                    open_list.add(neighbour, f, g, h)

                    g_scores[neighbour] = g
                    f_scores[neighbour] = f
                    came_from[neighbour] = current_state

                else:
                    pass
            # print("Old g score was better", neighbour)
            else:
                h = heuristic(neighbour, goal)
                f = g + h
                open_list.add(neighbour, f, g, h)

                g_scores[neighbour] = g
                f_scores[neighbour] = f
                came_from[neighbour] = current_state

        iterations += 1

        # kent: break the A* pathfinding if too much iterations
        if len(open_list) > 80000:
            break
    # print(f"Not Found.")# i = {i} Start = {start} Goal = {goal}")#, constraints = {constraints}")
    # from pprint import pprint
    # pprint(constraints)
    return None, None


class A_Star_Queue:
    """ A queue for a_star()"""

    def __init__(self):
        self.heap = []
        self.counter = count()
        self.set = set()

    def add(self, state, f, g, h):
        if state in self.set:
            # print(f"{state} already in the queue.")
            # TODO: Replace the state with a better g value
            # See: https://stackoverflow.com/questions/10162679/python-delete-element-from-heap
            return
        self.set.add(state)
        heappush(self.heap, (f, h, g, next(self.counter), state))

    def pop(self):
        # print(self.heap)
        f, h, g, c, state = heappop(self.heap)
        self.set.remove(state)
        return state, f, g, h

    def __str__(self):
        states = []
        for f, g, h, c, state in self.heap:
            states.append(state)
        return str(states)

    def __len__(self):
        return len(self.heap)


class State(namedtuple("State", "x y z direction time")):
    """State object used for a_star()"""

    __slots__ = ()

    def equal_except_time(self, other):
        return (
            self.x == other.x
            and self.y == other.y
            and self.z == other.z
            and self.direction == other.direction
        )


def is_solution(state, goal):
    """Checks if the state is the goal position"""
    # Comparing x

    if goal.x is None:
        x = True
    else:
        x = state.x == goal.x

    # x = (state.x == goal.x) if goal.x is not None else True
    # x = True if goal.x is None else (state.x == goal.x)

    # Comparing y
    if goal.y is None:
        y = True
    else:
        y = state.y == goal.y

    # Comparing z
    if goal.z is None:
        z = True
    else:
        z = state.z == goal.z

    # Comparing direction
    if goal.direction is None:
        direction = True
    else:
        direction = state.direction == goal.direction

    # Comparing time
    if goal.time is None:
        time = True
    else:
        time = state.time >= goal.time

    return x and y and z and direction and time


def is_solution_except_time(state, goal):
    """Checks if the state is the goal position"""
    # Comparing x
    if goal.x is None:
        x = True
    else:
        x = state.x == goal.x

    x = (state.x == goal.x) if goal.x is not None else True
    x = True if goal.x is None else (state.x == goal.x)

    # Comparing y
    if goal.y is None:
        y = True
    else:
        y = state.y == goal.y

    # Comparing z
    if goal.z is None:
        z = True
    else:
        z = state.z == goal.z

    # Comparing direction
    if goal.direction is None:
        direction = True
    else:
        direction = state.direction == goal.direction

    return x and y and z and direction


def get_neighbours(state, skycar):
    """Gets neighbours of the input state

     Requires is_valid(state) to be defined"""

    x = state.x
    y = state.y
    z = state.z
    direction = state.direction
    time = state.time

    if z > 0:  # only when winch lowered
        neighbours = [
            State(x, y, z, direction, time + 1),
            State(x, y, z + 1, direction, time + 1),
            State(x, y, z - 1, direction, time + 1),
        ]

    elif direction == "x":
        neighbours = [
            State(x, y, z, direction, time + 1),
            State(x, y, z, "y", time + 1),
            State(x + 1, y, z, direction, time + 1),
            State(x - 1, y, z, direction, time + 1),
            State(x, y, z + 1, direction, time + 1),
        ]

    elif direction == "y":
        neighbours = [
            State(x, y, z, direction, time + 1),
            State(x, y, z, "x", time + 1),
            State(x, y + 1, z, direction, time + 1),
            State(x, y - 1, z, direction, time + 1),
            State(x, y, z + 1, direction, time + 1),
        ]
    else:
        print(state)
    valid_neighbours = [n for n in neighbours if is_valid(n, skycar)]

    #     constraints = {}#{State(10, 0 ,0, 'x', 10)}
    #     neighbours = [n for n in valid_neighbours if n not in constraints]
    return valid_neighbours


def check_constraints(prev_state, current_state, neighbours, constraints, Start, Goal):  # Kent: add in prev_state
    """Filter out all neighbours that are redundant or constrained. (Both vertex and edge constraints.)"""

    constrained = []
    for neighbour in neighbours:
        if (neighbour in constraints or (current_state, neighbour) in constraints or
                (prev_state is not None and neighbour[:4] == prev_state[:4] and current_state[:4] != prev_state[:4]) or  # Kent: prevent return back to the previously visited nodes
                (Goal.x != None and Goal.y != None and current_state.x != Goal.x and current_state.y != Goal.y and neighbour.z != 0) or # no need to explore bottom if SC haven't reach destination
                (Start.z == Goal.z == 0 and neighbour.z != 0)):  # Kent: if the pf is on roof level only, then no need to explore cell below roof level
            continue
        else:
            constrained.append(neighbour)
    return constrained


def is_valid(state, skycar):
    """Check if a state is within cube limits"""

    is_within_limits = (
        0 <= state.x <= cube_setting["grid_data"]["max_x_index"]
        and 0 <= state.y <= cube_setting["grid_data"]["max_y_index"]
        and 0 <= state.z <= cube_setting["grid_data"]["max_z_index"]
    )

    # # obstacle is updated by nick or ronn
    #

    is_not_blocked = False
    # obstacle is updated by nick or ronn

    obstaclesExcludeSelf = {key for (key, value) in db.obstacles.items() if value.job == None or value.job.skycar != skycar}
    # 1,0 : { obstacle }

    if (f"{state.x},{state.y}" not in obstaclesExcludeSelf):
        is_not_blocked = True

    return is_within_limits and is_not_blocked


def heuristic(state1, state2):
    """Calculates the heurisitic, h, from state1 to a state2

     h = dx + dy + dz + ddir

     Where:
          - dx is the difference in x values between state1 and state2
          - dy is the difference in y values between state1 and state2

          - dz is the *sum* of z values of state1 and state2, unless state1 and
            state2 are in the same x-y location, in which case it's the
            *difference* in z values between state1 and state2.
            This is to simulate the winching behaviour of the skycar-
            lowering the winch anywhere other than the correct x-y
            location will take you further from your location.

          - ddir is 0 when state2 can be reached by only moving in the
            current direction. If not it is 1.

     """
    # Note: goal.direction and goal.time are NOT used

    # # For wildcard goals
    # if None in state2:
    #     return 0  #this will make the search act like Dijkstra's algorithm
    # Computing dx
    if state2.x is None:
        dx = 0
    else:
        dx = abs(state2.x - state1.x)

    # Computing dy
    if state2.y is None:
        dy = 0
    else:
        dy = abs(state2.y - state1.y)

    # Computing dz
    if state2.z is None:
        dz = 0
    else:
        if state1.x == state2.x and state1.y == state2.y:
            dz = abs(state2.z - state1.z)
        else:
            dz = abs(state2.z) + abs(state1.z)

    # Computing ddir
    if state2.direction is None:
        if dx == 0 and state1.direction == "y" or dy == 0 and state1.direction == "x":
            ddir = 0
        else:
            ddir = 1
    else:
        if state1.direction == state2.direction:
            ddir = 0
        else:
            ddir = 1

    # Kent: edit the entire time calculation method
    # dt = state2.time - state1.time
    # if dt < 0:
    #     dt = 0

    ''' 
    Kent:
    If goal time is not specified (= 0), the heuristic just depends on dx + dy + dz + ddir only as the pf ends after SC reach the goal
    thus, no dt was assigned

    If goal time is given, there are two possibilities which need to taking care of dt:
    1) SC reach destination before the specified goal time, then SC needs to stay at destination until reaching goal time,
    thus dt = state1 to state 2 duration - travel duration
    2) SC reach destination later than the designated goal time, then no need to add any additionay staying time since the SC already late to the goal,
    thus dt = 0
    '''
    if state2.time == 0:
        dt = 0
    else:
        time_diff_btw_states = state2.time - state1.time
        if dx + dy + dz + ddir < time_diff_btw_states:
            dt = time_diff_btw_states - (dx + dy + dz + ddir)
        else:
            # dt = dx + dy + dz + ddir - time_diff_btw_states (possibly wrong)
            dt = 0
            
    return dx + dy + dz + ddir + dt


if __name__ == "__main__":
    start = State(x=5, y=5, z=0, direction="y", time=0)

    goal = State(x=None, y=None, z=0, direction=None, time=5)
    con = set([])
    for z in range(20):
        # for t in range(9,10):
        # t=2
        con.add(State(5, 5, z, "y", 2))
        con.add(State(5, 5, z, "x", 2))

        con.add(State(5, 6, z, "y", 2))
        con.add(State(5, 6, z, "x", 2))

        # t=3
        con.add(State(5, 4, z, "y", 3))
        con.add(State(5, 4, z, "x", 3))

        con.add(State(5, 5, z, "y", 3))
        con.add(State(5, 5, z, "x", 3))

        # t=4
        con.add(State(5, 3, z, "y", 4))
        con.add(State(5, 3, z, "x", 4))

        # con.add(State(5, 4, z, "y", 4))
        # con.add(State(5, 4, z, "x", 4))

        # # t=5
        # con.add(State(5, 2, z, "y", 5))
        # con.add(State(5, 2, z, "x", 5))

        # con.add(State(5, 3, z, "y", 5))
        # con.add(State(5, 3, z, "x", 5))

        # con.add(State(5, 1, z, "y", 5))
        # con.add(State(5, 1, z, "x", 5))

    path, cost = a_star(start, goal, 'skycar', constraints=con)
    print(*path, sep="\n")
