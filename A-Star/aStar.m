% This code works through the sample A* graph search algorithm
% presented in Kevin Lynch's Modern Robotics video series
% Video - Chapter 10.2.4: Graph Search

% Goal: find a path between start and end nodes which minimizes the cost

% Graph is weighted, undirected graph
% Correct solution:
%   - Optimal Path: 1-4-5-6
%   - Optimal Cost: 30

% Terms:
%   Past Cost (past_cost): 
%   Optimistic Cost to Go (optimist_ctg)
%       - lower bound on actual cost to go from a node to a goal node
%       - optimistic if actual cost to go can never be lower
%       - needs to be fast to evaluate and reasonably close to actual
%       - good function computes straight line distance from start to goal
%   Estimated Total Cost (est_tot_cost):
%   Parent Node (parent_node): previous node on best known path for node(i)

clear
clc
close all

% Initialize graph
num_nodes = 6;
nodes = 1:1:num_nodes;
start_node = nodes(1);
goal_node = nodes(end);
    
cost = [ 0  0 18 12 30  0;
         0  0 27  0  0 10;
        18 27  0  0  0 15;
        12  0  0  0  8 20;
        30  0  0  8  0 10;
         0 10 15 20 10  0];
         
past_cost = zeros(1, num_nodes);
% optimist_ctg = zeros(1, num_nodes);
% est_tot_cost = zeros(1, num_nodes);
parent_node = zeros(1, num_nodes); 

% Initialize A* table
for i = 1:1:num_nodes
    if i ~= start_node
        past_cost(i) = inf;
    end
end

% Start with fake optimistic cost to go.
% Should have a function that evaluates this
optimist_ctg = [20, 10, 10, 10, 10, 0];

est_tot_cost = past_cost + optimist_ctg;

% Initialize open and closed lists
open_node = start_node;
open_cost = optimist_ctg(start_node);
closed_node = [];

% Start A* Graph Search
tic
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
toc
disp(' ')

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
toc
disp(' ')

disp('Optimal Path: ')
disp(optimal_path)
disp('Optimal Cost: ')
disp(optimal_cost)