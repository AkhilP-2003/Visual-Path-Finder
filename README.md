# Visual-Path-Finder
An A* Path Finder algorithm computes the shortest distance between a starting point and an ending point on a 50x50 grid.

# How to operate:

  1. Clone the repository and run python3 pathFinder.py in your terminal
  2. After running, a 50x50 grid should appear. Right-click once anywhere you would like to place the starting point (the orange block)
  3. Then, right-click again anywhere on the grid to place the endpoint (the point at which you want to finish, the blue block)
  4. If you want, you can hold right click on the grid to create obstacles for the path-finding algorithm to go through
  5. Lastly, to run the algorithm as the visualizer, press the spacebar key and the shortest path from start to end should appear
  6. If you want to clear the grid and go again, press the backspace key

## How does the algorithm work?
The A* algorithm works as an informed search, meaning it will take into account where the end "goal" is while it computes the shortest path from start to finish. The algorithm can be represented by the following function:

                                                    F(n) = g(n) + h(n)
where g(n) is the cost to reach from the starting cell on the grid, to cell n (where n is/are the adjacent blocks of the starting point). h(n) represents a heuristic function that estimates the cost of the shortest path from block n to the goal/finish block. This is computed in terms of Manhattan distance, where in simpler terms, it is the vertical and horizontal distance from the starting block to the ending block.

Essentially, this algorithm works by computing the distance from start -> n (this is where g(n) is applied), then from n -> end (this is where h(n) is applied). The sum of g(n) and h(n) is the total path/cost to reach from start -> end, denoted by F(n).
