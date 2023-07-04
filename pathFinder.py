import pygame
import math
from queue import PriorityQueue

# Making the window application for the path finder
width = 800
window = pygame.display.set_mode((width, width))
pygame.display.set_caption("A* Path Finding Algorithm")

# Defining the colours of RGB
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 255, 0)
yellow = (255, 255, 0)
white = (255, 255, 255)
black = (0, 0, 0)
purple = (128, 0, 128)
orange = (225, 165, 0)
grey = (128, 128, 128)
turquoise = (64, 224, 208)


class Cell:

    def __init__(self, row, col, width, totalRows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = white  # Default color for each square
        self.neighbors = []
        self.width = width
        self.totalRows = totalRows

    def get_position(self):
        """ returns the position of the cell in the 50x50 grid"""
        return self.row, self.col

    def closed_cell(self):
        """ checks if the cell is closed
        (if it is red, then it is closed since it's already been visited)"""
        return self.color == red

    def open_cell(self):
        """ checks if the cell is in the set
        (if it is green, then it is in the set of cells to visit)"""
        return self.color == green

    def barrier_cell(self):
        """ checks if the cell is an obstacle to overcome
        (if it is black, then it is an obstacle)"""
        return self.color == black

    def start_cell(self):
        """ checks if the cell is the starting cell
        (if it is blue, then it is the starting position)"""
        return self.color == orange

    def end_cell(self):
        """checks if the cell is the end goal
        (if it is turquoise, it is the end cell)"""
        return self.color == turquoise

    def reset_cell(self):
        """ sets the colour of the cell to white, indicating the cell has been reset"""
        self.color = white

    def create_cell_start(self):
        """ creates the starting cell"""
        self.color = orange

    def create_cell_closed(self):
        """ creates the closed cell"""
        self.color = red

    def create_cell_open(self):
        """ creates an open cell to visit"""
        self.color = green

    def create_barrier(self):
        """ creates obsticale(s) that the algorithmn should maenouver through to
        get to the end cell"""
        self.color = black

    def create_cell_end(self):
        """ creates the goal cell that the algorithm should finish at"""
        self.color = turquoise

    def create_path(self):
        """ creates the path from start cell to goal cell"""
        self.color = purple

    def draw(self, win):
        """ draws the cell in the grid"""
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_cell_neighbors(self, grid):
        """ Decide which cells can be identified as "visitable" cells and which cells cannot
         (e.g, barrier cells cannot be visited, so it will not be considered as an adjacent cell).
         The visitable cells will then be identified as neighbor cells."""
        self.neighbors = []

        # Check if you can go move DOWN a cell to visit
        if self.row < self.totalRows - 1 and not grid[self.row + 1][self.col].barrier_cell():
            # If this is valid, then append the next row
            self.neighbors.append(grid[self.row + 1][self.col])

        # Check if you can move UP a cell to visit
        if self.row > 0 and not grid[self.row - 1][self.col].barrier_cell():
            # If this is valid, then append the next row
            self.neighbors.append(grid[self.row - 1][self.col])

        # Check if you can move to the RIGHT of a cell to visit
        if self.col < self.totalRows - 1 and not grid[self.row][self.col + 1].barrier_cell():
            # If this is valid, then append the next row
            self.neighbors.append(grid[self.row][self.col + 1])

        # Check if you can move to the LEFT of a cell to visit
        if self.col > 0 and not grid[self.row][self.col - 1].barrier_cell():
            # If this is valid, then append the next row
            self.neighbors.append(grid[self.row][self.col - 1])

    # Comparison
    def __lt__(self, other):
        return False


def heuristic_function(point1: tuple[int, int], point2: tuple[int, int]):
    """ Heuristic function that takes the parameters point1 and point2 and returns
        the estimated distance between these two points via the manhattan distance.
        The manhattan distance is calculated by vertical and horizontal movements."""

    # Assigning x and y coordinated of point1 and point2
    x1, y1 = point1
    x2, y2 = point2

    # Taking the absolute value and returning it
    return abs(x1 - x2) + abs(y1 - y2)


def create_path(cameFrom: dict, current: Cell, draw: list[list[Cell]]):
    """ creating the path from the start cell to the end/goal cell once the algorithmn has
    computed the shortest path"""
    while current in cameFrom:
        current = cameFrom[current]
        current.create_path()
        draw()


def algorithm(draw: any, grid: list[list[Cell]], start: Cell, end: Cell):
    """ The main A* path finding algorithm that computes the lowest cost from getting to
    the start cell to the end cell. First, an open set is created as a priority queue."""
    count = 0
    openset = PriorityQueue()

    # Add to the priority queue, where <0> is the f score of the starting cell, counter keeps track of the number,
    # start is the startng cell
    openset.put((0, count, start))

    # Creating a dictionary to keep track of the path (e.g, cell #4 came rom cell #2, which came from cell #6, etc)
    cameFrom = {}
    g_score = {}
    for row in grid:
        for cell in row:
            # Keeping track of the shortest g score (the current shortest distance from start cell to adjacent cell)
            # since we are assuming that every cell (before the algo is run) has an f score of infinity
            g_score[cell] = float("inf")

    g_score[start] = 0  # Gscore for start node is zero

    f_score = {}
    for row in grid:
        for cell in row:
            # Keeping track of the shortest g score (the current shortest distance from start cell to adjacent cell)
            # since we are assuming that every cell (before the algo is run) has an f score of infinity
            f_score[cell] = float("inf")

    # The f score of the starting cell is the heuristic function since we want to estimate how far away the
    # start cell is from the goal cell (it has no g score)
    f_score[start] = heuristic_function(start.get_position(), end.get_position())

    # Keeping track of all the items in the priority queue (since we cannot check if something is in the open_set as it
    # is a prioroity queue, but here, we can check if anything is in it since it is a hash that stores all the same
    # things that the priority queue stores but we ignore the data structure implementation)
    openset_hash = {start}  # Helps us see if something is in the openset

    # The algo runs until the open set is empty
    while not openset.empty():
        for event in pygame.event.get():

            # Giving the user a way of exiting this loop
            if event.type == pygame.QUIT:
                pygame.quit()

        # Indexing at 2 since <open_set> stores the cell at index 2, it is the current cell we are looking at
        # and will start as the starting cell
        current = openset.get()[2]

        # Remove the current cell (the cell we are looking at) to avoid duplicates
        openset_hash.remove(current)

        # Check if the current cell is the end cell
        if current == end:
            # Calling <create_path> from the start cell to end cell
            create_path(cameFrom, end, draw)
            end.create_cell_end()
            start.create_cell_start()
            return True

        for neighbor in current.neighbors:

            # Assuming we visit the neighbor cell that we are looking at, the g score will then be 1: The distance
            # between the starting cell and the neighbor cell is 1, so we add 1
            temp_gscore = g_score[current] + 1

            # Check if gscore from the start cell to the neighbor cell is the shortest path possible
            if temp_gscore < g_score[neighbor]:
                cameFrom[neighbor] = current

                # Update g score and f score
                g_score[neighbor] = temp_gscore
                f_score[neighbor] = temp_gscore + heuristic_function(neighbor.get_position(), end.get_position())

                # Check if neighbor is in open set or not
                if neighbor not in openset_hash:
                    count += 1
                    openset.put((f_score[neighbor], count, neighbor))
                    openset_hash.add(neighbor)
                    neighbor.create_cell_open()

        draw()  # Calling the draw function to run (passed in through lambda (anonymous function))

        # Check if current is not the start cell
        if current != start:
            # Make it closed
            current.create_cell_closed()

    # If we do not find a path, return false
    return False


def drawGrid(rows: int, width: int):
    """ Creating the grid by taking in the parameters row and width and returning
    the grid based on the cell created."""
    grid = []

    # Calculating the widh of each cell
    widthOfCells = width // rows

    for row in range(rows):

        # Creating placeholders for the cells in that row
        grid.append([])
        for col in range(rows):
            # Creating a cell object that represents a single cell in the grid
            cell = Cell(row, col, widthOfCells, rows)

            # Append to the cell object to the corresponding row in the grid
            grid[row].append(cell)
    return grid


def drawGridLines(win: int, rows: int, width: int):
    """ Drawing the grid lines of a grid."""

    # Calculating the width of the cells
    widthOfCells = width // rows

    # Making the horizontal gridlines
    for row in range(rows):

        # Draw a horizontal line for every row in the grid
        pygame.draw.line(win, grey, (0, row * widthOfCells), (width, row * widthOfCells))
        for col in range(rows):
            # Draw a vertical line for every row in the grid
            pygame.draw.line(win, grey, (col * widthOfCells, 0), (col * widthOfCells, width))


def draw(win: int, grid: list[list[Cell]], rows: int, width: int):
    """ Drawing everything for the grid"""

    # Filling in the colour of the grid after every frame
    win.fill(white)
    for row in grid:
        for cell in row:
            cell.draw(win)

    # Creating grid lines on top of cells
    drawGridLines(win, rows, width)

    # Update display
    pygame.display.update()


def checkMousePosition(position: tuple[int, int], rows: int, width: int):
    """ Gets the mouse position on the grid and determines where the cell is according
    to the mouse position."""

    # Calculating the width of the cells
    widthOfCells = width // rows

    # Setting the position
    y, x = position

    # Getting the position of where the cell of interest is
    row = y // widthOfCells
    col = x // widthOfCells

    # The row and col that was clocked
    return row, col


def main(win: int, width: int):
    """ The main function that puts everything together; it
        will determine all of the checks that we are doing"""

    rows = 50

    # Creating the 2-D array of cells; the grid
    grid = drawGrid(rows, width)

    # Defining the start and the end goal position to be intially None
    start = None
    end = None

    # Checking if the main loop has run
    run = True

    # Checking if the algorithmn has started or not
    started = False

    while run:
        draw(win, grid, rows, width)

        for event in pygame.event.get():

            # Check if user has exited the game by pressing x button at top right
            if event.type == pygame.QUIT:
                run = False

            if started:
                continue

            # Check if the left mouse button is pressed (the common left click)
            if pygame.mouse.get_pressed()[0]:

                # Gets the position (x,y) of the mouse
                position = pygame.mouse.get_pos()
                row, col = checkMousePosition(position, rows, width)
                cell = grid[row][col]

                # Check if starting cell has been chosen
                if not start and cell != end:

                    # If not, create the start cell
                    start = cell
                    start.create_cell_start()

                # Check if the goal cell has been created
                elif not end and cell != start:

                    # If not, create the end cell
                    end = cell
                    end.create_cell_end()

                # Check if mouse is not clicking on the start/goal cell (in other words, the user is making barriers)
                elif cell != start and cell != end:

                    # Create barriers
                    cell.create_barrier()

            # Check if the right mouse button is pressed
            elif pygame.mouse.get_pressed()[2]:
                position = pygame.mouse.get_pos()
                row, col = checkMousePosition(position, rows, width)
                cell = grid[row][col]
                cell.reset_cell()

                # Check if start and end cell are pressed by right mouse button (right clicked)
                if cell == start:
                    # Reset start
                    start = None
                elif cell == end:
                    # Reset end
                    end = None

            # Check if a key has been pressed down from keyboard
            if event.type == pygame.KEYDOWN:

                # Check if the user presses the space button
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            # Updating all the neighbor cells for every cell in the row
                            spot.update_cell_neighbors(grid)

                    # Calling the algorithm function as lambda (an anonymous function that is used instead of a
                    # user defined function for the sake of simplicity
                    algorithm(lambda: draw(win, grid, rows, width), grid, start, end)

                # Check of the enter key is pressed to clear the grid
                if event.key == pygame.K_BACKSPACE:
                    start = None
                    end = None
                    grid = drawGrid(rows, width)
    pygame.quit()


# Running the program with everything defined above into account
main(window, width)
