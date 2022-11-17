% MCE 793: Intelligent Controls
% Homework 6: Rapidly Exploring Random Tree (RRT)
% Created by Jack Schultz
% Due April 23rd, 2020

clear; clc; close all

OB = [0 0; 12 0; 12 12; 0 12; 0 0];
qs = [4.5 6.5];
qg = [10.5 .5];

c1 = [ 6.0, 5.0];
c2 = [ 8.5, 1.5];
c3 = [10.5, 2.0];

obstacle1 = [c1(1)-1 c1(2)-1; c1(1)+1 c1(2)-1; c1(1)+1 c1(2)+1; c1(1)-1 c1(2)+1; c1(1)-1 c1(2)-1]; 
obstacle2 = [c2(1)-1 c2(2)-1; c2(1)+1 c2(2)-1; c2(1)+1 c2(2)+1; c2(1)-1 c2(2)+1; c2(1)-1 c2(2)-1]; 
obstacle3 = [c3(1)-1 c3(2)-1; c3(1)+1 c3(2)-1; c3(1)+1 c3(2)+1; c3(1)-1 c3(2)+1; c3(1)-1 c3(2)-1]; 

figure
plot(OB(:,1),OB(:,2))
hold on
plot(qs(1), qs(2), 'g.', 'MarkerSize', 18)
plot(qg(1), qg(2), 'm.', 'MarkerSize', 18)
axis([-1 13 -1 13])

fill(obstacle1(:,1), obstacle1(:,2),'r')
fill(obstacle2(:,1), obstacle2(:,2),'r')
fill(obstacle3(:,1), obstacle3(:,2),'r')

%% Part a) Plot the tree

data.parent = 0;
data.node = qs;
i = 1;
E = inf;
flag = 0;
shortest_E = inf;

while E > .1
    i = i+1;
    % Every so often, set new node to equal the goal node
%     if mod(i,50) == 0
%         data.node(i,:) = qg;
%     else
%         data.node(i,:) = 12*rand(2,1);
%     end

    data.node(i,:) = 12*rand(1,2);
    
    % Find nearest parent node to new node
    shortest_d = inf;
    for ii = 1:1:size(data.node,1)
        d = sqrt((data.node(ii,1)-data.node(i,1))^2 + (data.node(ii,2)-data.node(i,2))^2);
        if d ~= 0 && d < shortest_d
            shortest_d = d;
            index_d = ii;
        end
    end
    
    data.parent(i,1) = index_d;
    
    % Collision Detection
    v = [linspace(data.node(data.parent(i),1), data.node(i,1), 50)', linspace(data.node(data.parent(i),2), data.node(i,2), 50)'];
    for r = 1:1:length(v)
        [in, on] = inpolygon(v(r,1), v(r,2), obstacle1(:,1),obstacle1(:,2));
        [in2, on2] = inpolygon(v(r,1), v(r,2), obstacle2(:,1),obstacle2(:,2));
        [in3, on3] = inpolygon(v(r,1), v(r,2), obstacle3(:,1),obstacle3(:,2));
        if      in == 1 ||  on == 1 ...
            || in2 == 1 || on2 == 1 ...
            || in3 == 1 || on3 == 1
        
            data.node(i,:) = v(r-1,:);
            break
        end
    end
    
    plot([data.node(index_d,1) data.node(i,1)], [data.node(index_d,2) data.node(i,2)], 'b')
    pause(.01)
    
    E = sqrt((qg(1)-data.node(i,1))^2 + (qg(2)-data.node(i,2))^2);
    if E < shortest_E
        shortest_E = E;
        index_E = i;
    end
    
    if i >= 2000
        warning('Too many iterations. Stopping here and plotting closest point.')
        plot(data.node(index_E,1), data.node(index_E,2), '*g', 'MarkerSize', 8)
        flag = 1;
        break
    end
end

if flag == 0
    disp('Node found!')
    plot(data.node(index_E,1), data.node(index_E,2), '*g', 'MarkerSize', 8)
end

% Plot the path to the node closest to the goal
j = index_E;
k = data.parent(index_E);

while k ~= 0
    plot([data.node(k,1) data.node(j,1)], [data.node(k,2) data.node(j,2)], 'g', 'LineWidth', 2)
    j = k;
    k = data.parent(j);
end


%% Part b) Report Vertices
display(['vertices = ', num2str(length(data.node))])


%% Part a) Report Edges
display(['edges = ', num2str(length(data.node)-1)])