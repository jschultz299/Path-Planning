# A* Search Algorithm

A* is one of the most common and efficient graph search algorithms. It finds the optimal path from a start node to a goal node, connecting intermediate nodes via edges.

In this code, I have implemented the A* algorithm on the example Kevin Lynch presents in his video on Path Planning [here](https://youtu.be/ZI800-2jv38).

Below you can see the example graph, along with it's optimal path, shown in green.

<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/graph.png" width=31%> <img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/solution.png" width=30%>

This is a weighted, undirectional graph. The graph consists of ```n = 6``` nodes. The start node is node #1 while the goal node is node #6. Some of the nodes are connected via edges such that there are multiple paths you can take to reach the goal node from the start node.

Each edge also contains an associated cost. This can be thought of as the effort required to take that path, or perhaps the distance between nodes, though then this graph is not shown to scale.

Here we can see that the optimal path is to start at node #1, travel to node #4, then through node #5, and finally to the goal node, nod #6. If we sum up the cost associated with each path, we get a ```cost = 30```. Therefore, the optimal path is ```1-4-5-6```.

But how to we find this optimal path? That's where the A* algorithm comes in handy. I've implemented the solution in the matlab script [Astar.m](https://github.com/jschultz299/Path-Planning/blob/main/A-Star/aStar.m).

The first step is to recreate the nodes and associated costs in a matrix. Because there are 6 total nodes, or matrix will be a ```6x6``` matrix, where each element in the matrix is the cost associated with travelling along the path between the nodes ```i``` and ```j```. In an undirectional graph, it does not matter which direction you travel between nodes, so there will be duplicate costs in our matrix. This is okay, the A* search algorithm can handle this.

Here's what the cost matrix looks like for this example:

```matlab
cost = [ 0  0 18 12 30  0 ;
         0  0 27  0  0 10 ;
        18 27  0  0  0 15 ;
        12  0  0  0  8 20 ;
        30  0  0  8  0 10 ;
         0 10 15 20 10  0 ];
```
Next, we initialize 4 ```1xn``` vectors, where the ```nth``` column corresponds to the node:
1. Past Cost - cost of previous best known path through each node
2. Optimistic Cost to Go - lower bound on the actual cost to go from a node to a goal node
3. Estimated Total Cost - sum of Past Cost and Optimistic Cost to Go
4. Parent Node - previous node on best known path for each node

For the ```past_cost```, the cost to travel from node #1 to itself is ```0```, whereas we initialize the rest to be ```inf```. 

For the ```optimist_ctg```, we want to choose values that are close to the actual cost to go, but serve as a lower bound. In this case, we will estimate the straight line distance from each node to the goal node. In a real problem, this may be measured. For this reason, we will estimate the ```optimist_ctg``` for node #1 to be ```20```, ```10``` for nodes #2-5, and ```0``` for node #6, because it is the goal node.

For the ```est_tot_cost```, we simply sum the ```past_cost``` and the ```optimist_ctg```.

Finally, for the ```parent_node```, we initialize all the nodes to ```0```.

Here is a summary table of our intial lists:

<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/initial_lists.png" width=50%>

We also need an ```OPEN``` list and a ```CLOSED``` list. These will tell the A* algorithm which node to search next, and when we can be finished searching a particular node. To start, we add the ```start_node``` to the ```OPEN``` list and leave the ```CLOSED``` empty.

To iteratively step through the A* search algorithm for this example, feel free to follow along with Kevin's video [here](https://youtu.be/ZI800-2jv38), or set breakpoints in the [Astar.m](https://github.com/jschultz299/Path-Planning/blob/main/A-Star/aStar.m) script.

Here is the section of the code that runs the actual A* algorithm:

```matlab
% Start A* Graph Search
while isempty(open_node) == false
    search_node = open_node(1);
    if search_node == goal_node
        disp('Search Done!')
        break
    end
    for i = 1:num_nodes
        if cost(search_node,i) ~= 0
            current_node = i;
            current_cost = cost(search_node,i) + past_cost(1,search_node);
            if current_cost < past_cost(1, current_node)
                parent_node(1,i) = search_node;
                past_cost(1,i) = cost(parent_node(1,i),i) + past_cost(1,parent_node(1,i));
                est_tot_cost = past_cost + optimist_ctg;
                % If search node is already in open_node list, replace it
                if ismember(current_node, open_node) == true
                    idx = find(open_node==current_node);
                    open_node(idx) = [];
                    open_cost(idx) = [];
                end
                open_node = [open_node, current_node];
                open_cost = [open_cost, est_tot_cost(1,current_node)];
                % Sort open list by ascending cost
                [open_cost, I] = sort(open_cost);
                open_node = open_node(I);
            end
        end
    end
    % Find and move search node from open to closed list
    idx = find(open_node==search_node);
    closed_node = [closed_node, open_node(idx)];
    open_node(idx) = [];
    open_cost(idx) = [];
end
```
While the ```OPEN``` list contains nodes, we loop through our cost matrix, comparing the costs between nodes to previously computed costs. If ```current_cost < past_cost```, then we update our information and continue the search, otherwise, it is not an optimal path and we skip it. The algorithm terminates if we search all possible paths, the first node in the ```OPEN``` list is the goal node, or the ```OPEN``` list is empty.

Once the A* algorithm finds an optimal path, we can reconstruct this path by stepping back through the ```parent_node``` list. We can also sum the costs as we go to compute the ```Optimal Cost```. Here is the final section of the code that achieves this:

```matlab
% After search is finished, reconstruct the optimal path
optimal_path = zeros(1,num_nodes);
optimal_path(1,end) = goal_node;
optimal_cost = 0;
tic
for i = num_nodes-1:-1:1
    optimal_path(1,i) = parent_node(optimal_path(1,i+1));
    optimal_cost = optimal_cost + cost(optimal_path(1,i), optimal_path(1,i+1));
    if optimal_path(1,i) == 1
        disp('Path Found!')
        optimal_path(optimal_path==0) = [];
        break
    end
end
```
Finally, let's display the ```Optimal Path``` and the ```Optimal Cost```.

```matlab
disp('Optimal Path: ')
disp(optimal_path)
disp('Optimal Cost: ')
disp(optimal_cost)
```
This should be the output if you've implemented the code correctly:

<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/output.png" width=35%>

If we compare this to what we computed previously, this is the optimal solution.

<img src="https://github.com/jschultz299/Path-Planning/blob/main/A-Star/img/solution.png" width=30%>

Optimal Path: ```1-4-5-6```,
Optimal Cost: ```30```

So congratulations! Our A* search algorithm found the optimal path in less than ```0.01``` seconds!

If you want to learn about another path planning approach, check out my project on [Rapidly Exploring Random Trees](https://github.com/jschultz299/Path-Planning/tree/main/Rapidly-Exploring-Random-Trees).

## Acknowledgments
Much of the information here came from Kevin Lynch's book, [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) as well as his corresponding YouTube series, found [here](https://www.youtube.com/playlist?list=PLggLP4f-rq02vX0OQQ5vrCxbJrzamYDfx).
