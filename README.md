# PathFindingBelkan
Developed multiple path finding algorithms and reactive-like machine.
`./install.sh` and then we could `./Belkan

## Depth First-Search
Non-garantee of optimal solution, problems with convergence and halting, could cycle but pretty fast.
Computationally depends of the depth of the solution. Must avoid going in circles.

## Breadth First-Search 
Garantee of optimal solution, always halt and computationally depends of b^d temporally and spacially.
Where b is the branching factor of the tree.  And d is the depth of the solution 

## A Star algorithm
Basically improves the solution from Breadth-First Search utilizing heuristics. f(n) = g(n) + h(n).
We sort using f(n), where g(n) is the cost of the movement and h(n) is the euclidean distance to the objetive.

## Djikstra algorithm
First approach to multiple path finding problem for the challenge, level 4, nonetheless caome up with greater 
and faster solution using A star and some of my own heuristic such as avoid highly costly paths and enabling
for longer paths for safety.
