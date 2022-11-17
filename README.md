# Path-Planning

Motion Planning for robotics consists of moving a robot from a start state to a goal state while avoiding obstacles as well as obeying constraints such as joint and torque limits.

Path Planning is a subset of the larger field of Motion Planning. Path planning consists of finding the geometric path that connects a start state to a goal state, while avoiding obstacles. In path planning, however, we ignore robot dynamics and additional environmental and motion constraints.

In this repository, we explore two common path planning algorithms:
1. A* search algorithm
2. Rapidly Exploring Random Trees

## 1) A* Search Algorithm

A* is one of the most common and efficient graph search algorithms. It finds the optimal path from a start node to a goal node, connecting intermediate nodes via edges.

In this code, I have implemented the A* algorithm on the example Kevin Lynch presents in his video on Path Planning [here](https://youtu.be/ZI800-2jv38).

Below you can see the example graph, along with it's optimal path, shown in green.

<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/graph.png" width=25%>
<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/solution.png" width=25%>

The graph consists of 6 nodes. The start node is node #1 while the goal node is node #6. Some of the nodes are connected via edges such that there are multiple paths you can take to reach the goal node from the start node.

Each edge also contains an associated cost. This can be thought of as the effort required to take that path, or perhaps the distance between nodes, though then this graph is not shown to scale.

Here we can see that the optimal path is to start at node #1, travel to node #4, then through node #5, and finally to the goal node, nod #6. If we sum up the cost associated with each path, we get a ```cost = 30```. Therefore, the optimal path is ```1-4-5-6```.

But how to we find this optimal path? That's where the A* algorithm comes in handy. I've implemented the solution in the matlab script [Astar.m](https://github.com/jschultz299/Path-Planning/blob/main/A-Star/aStar.m).


## Acknowledgments
Much of the information here came from Kevin Lynch's book, [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) as well as his corresponding YouTube series, found [here](https://www.youtube.com/playlist?list=PLggLP4f-rq02vX0OQQ5vrCxbJrzamYDfx).


