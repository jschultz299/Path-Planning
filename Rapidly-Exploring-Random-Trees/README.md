# Rapidly Exploring Random Trees

In the previous project, we explored the [A* Graph Search algorithm](https://github.com/jschultz299/Path-Planning/tree/main/A-Star), where we used the algorithm to search a true representation of an environment to find the optimal path. It's often difficult or impossible to completely represent or model the environment in real applications, however.

For this reason, we need to use sampling path planning methods that allow us to build a representation of the environment as we explore.

One popular sampling path planning method is called a Rapidly Exploring Random Tree (RRT). 

In this code, I have implemented an RRT in a 2D environment in MATLAB. Much like in the [A* Search Algorithm](https://github.com/jschultz299/Path-Planning/tree/main/A-Star), we begin by creating an environment with a ```START``` node, a ```GOAL``` node, as well as ```OBSTACLES```.

Here is what our example environment will look like:

<img src="https://github.com/jschultz299/Path-Planning/blob/main/Rapidly-Exploring-Random-Trees/img/rrt-ex1-setup.png" width=50%>


## Acknowledgments
Much of the information here came from Kevin Lynch's book, [Modern Robotics: Mechanics, Planning, and Control](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) as well as his corresponding YouTube series, found [here](https://www.youtube.com/playlist?list=PLggLP4f-rq02vX0OQQ5vrCxbJrzamYDfx).
