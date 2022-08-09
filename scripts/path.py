#!/usr/bin/env python3
import sys
from PIL import Image
import rospy 
from std_msgs.msg import Int64
from milestone3.srv import *

class Node():
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

class StackFrontier():
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node

class QueueFrontier(StackFrontier):

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node

class Map():
    def __init__(self, filename):
        # Read file and set height and width of maze
        with open(filename) as f:
            contents = f.read()

        # Validate start and goal
        if contents.count("A") != 1:
            print("Please Enter The Starting Point: (i,j) ")
            a=input()
            self.start=tuple(int(x) for x in a.split(","))
        if contents.count("B") != 1:
            print("Please Enter The Goal Point: (i,j) ")
            a=input()
            self.goal=tuple(int(x) for x in a.split(","))
 
        # Determine height and width of maze
        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        # Keep track of walls
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                try:
                    if contents[i][j] == "A":
                        self.start = (i, j)
                        row.append(False)
                    elif contents[i][j] == "B":
                        self.goal = (i, j)
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("down", (row + 1, col)),
                ("downright", (row+1, col + 1)),
                ("right", (row, col + 1)),
                ("upright", (row-1, col + 1)),
                ("up", (row - 1, col)),
                ("upleft", (row-1, col - 1)),
                ("left", (row, col - 1)),
                ("downleft", (row+1, col - 1))
        ]
        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

    def solve(self,c):
        """Finds a solution to maze, if one exists."""

        # Keep track of number of states explored
        self.num_explored = 0

        # Initialize frontier to just the starting position
        start = Node(state=self.start, parent=None, action=None)
        if c=="B" or c=='b':
            frontier = QueueFrontier()
        elif c=='D' or c=='d':
            frontier = StackFrontier()
        frontier.add(start)

        # Initialize an empty explored set
        self.explored = set()

        # Keep looping until solution found
        while True:

            # If nothing left in frontier, then no path
            if frontier.empty():
                raise Exception("No Path Found")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)

    def output_image(self, filename, show_solution=True, show_explored=False):

        from PIL import Image, ImageDraw
        cell_size = 50
        cell_border = 2

        # Create a blank canvas
        img = Image.new(
            "RGBA",
            (self.width * cell_size, self.height * cell_size),
            "black"
        )
        draw = ImageDraw.Draw(img)

        solution = self.solution[1] if self.solution is not None else None
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):

                # Walls
                if col:
                    fill = (40, 40, 40)

                # Start
                elif (i, j) == self.start:
                    fill = (0, 0, 153)

                # Goal
                elif (i, j) == self.goal:
                    fill = (0, 171, 28)

                # Solution
                elif solution is not None and show_solution and (i, j) in solution:
                    fill = (0, 204, 204)

                # Explored
                elif solution is not None and show_explored and (i, j) in self.explored:
                    fill = (204, 0, 102)

                # Empty cell
                else:
                    fill = (237, 240, 252)

                # Draw cell
                draw.rectangle(
                    ([(j * cell_size + cell_border, i * cell_size + cell_border),
                      ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)]),
                    fill=fill
                )

        img.save(filename)


def FindPath():
    global control 
    global m 
    print("press D for DFS OR B for BFS OR E to Exit : ")
    c=input()
    if c=="E":
        print("BYE")
        rospy.signal_shutdown("Mission Accomplished")
        quit()

    m = Map('map.txt')
    m.solve(c)
    m.output_image("map.png", show_explored=True)

    for cell in m.solution[1]:
        print(cell)
        if cell == m.solution[1][len(m.solution[1])-1]:
            Goal=1
            response = control(cell[0],cell[1],Goal)
            print(response.res)
        
        else:
            Goal=0
            response = control(cell[0],cell[1],Goal)
            print(response.res)
    

if __name__ == '__main__':   
    global control
    rospy.init_node('PathPlanner', anonymous=True)
    rospy.wait_for_service('go')
    control = rospy.ServiceProxy('go', go)
    while not rospy.is_shutdown():
        FindPath()
    """ im = Image.open("map.png")
    im.show() """



