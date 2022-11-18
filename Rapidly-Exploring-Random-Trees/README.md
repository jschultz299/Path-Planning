# Rapidly Exploring Random Trees

In the previous project, we explored the [A* Graph Search algorithm](https://github.com/jschultz299/Path-Planning/tree/main/A-Star), where we used the algorithm to search a true representation of an environment to find the optimal path. It's often difficult or impossible to completely represent or model the environment in real applications, however.

For this reason, we need to use sampling path planning methods that allow us to build a representation of the environment as we explore.

One popular sampling path planning method is called a Rapidly Exploring Random Tree (RRT). 

In this code, I have implemented an RRT in a 2D environment in MATLAB. Much like in the [A* Search Algorithm](https://github.com/jschultz299/Path-Planning/tree/main/A-Star), we begin by creating an environment with a ```START``` node, a ```GOAL``` node, as well as ```OBSTACLES```.

Here is what our example environment will look like:

<img src="https://github.com/jschultz299/Path-Planning/blob/main/Rapidly-Exploring-Random-Trees/img/rrt-ex1-setup.png" width=40%>

And here is the section of the code that creates and plots this environment:

```matlab
OB = [0 0; 12 0; 12 12; 0 12; 0 0];
qs = [1 1];
qg = [8 7];

figure
plot(OB(:,1),OB(:,2))
hold on
plot(qs(1), qs(2), '.', 'MarkerSize', 20)
text(qs(1)-0.6,qs(2)-0.5,'START','Color','red','FontSize',14)
plot(qg(1), qg(2), '.', 'MarkerSize', 20)
text(qg(1)-0.6,qg(2)-0.5,'GOAL','Color','red','FontSize',14)
axis([-1 13 -1 13])

obstacle = [5 4; 7 4; 7 6; 5 6; 5 4];
obstacle2 = [4.5 6.5; 5.5 6.5; 5.5 7.5; 4.5 7.5; 4.5 6.5];
obstacle3 = [2 8; 4 8; 4 10; 2 10; 2 8];
obstacle4 = [9 1; 11 1; 11 3; 9 3; 9 1];
obstacle5 = [7 2; 8 2; 8 3; 7 3; 7 2];

fill(obstacle(:,1), obstacle(:,2),'r')
fill(obstacle2(:,1), obstacle2(:,2),'r')
fill(obstacle3(:,1), obstacle3(:,2),'r')
fill(obstacle4(:,1), obstacle4(:,2),'r')
fill(obstacle5(:,1), obstacle5(:,2),'r')
```
You can easily change the positions of the ```START``` and ```GOAL``` nodes as well as the number and positions of the ```OBSTACLES``` here.

In general, the RRT algorithm has two goals:
1. Find a path from the ```START``` node to the ```END``` node
2. Explore the space

Unlike the A* algorithm, the RRT does not guarantee the optimal path will be found. Instead, this is a sampling algorithm that will explore until a node is found within some radius of the goal node, or until a specified number of nodes have been searched.

In the code, we implement this with a ```while``` loop. Here, ```E``` is our search radius and in this case we've set it to ```0.2```. If we increase ```E```, it will be easier for the RRT algorithm to find a path planning solution, however it is likely the solution will not be as accurate. When a node is found within the specified radius to the ```GOAL``` node, the while loop will exit.

```matlab
while E > 0.2
    % Implement RRT algorithm here
end
```

As an added failsafe, we can add an additional exit criteria if the number of nodes exceeds a certain limit. We can do this by counting the number of iterations in the ```while``` loop and comparing it to the limit each time through the loop. In this case, we set our iteration limit to ```1000```. If this iteration limit is exceeded, we terminate the ```while``` loop.

```matlab
if i >= 1000
    warning('Too many iterations. Stopping here and plotting closest point.')
    plot(data.node(index_E,1), data.node(index_E,2), '*k', 'MarkerSize', 8)
    flag = 1;
    break
end
```


## Acknowledgments
Much of the information here came from Kevin Lynch's book, [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) as well as his corresponding YouTube series, found [here](https://www.youtube.com/playlist?list=PLggLP4f-rq02vX0OQQ5vrCxbJrzamYDfx).
