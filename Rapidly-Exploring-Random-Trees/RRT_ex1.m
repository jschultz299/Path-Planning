% Rapidly Exploring Random Tree Implementation Example
% Author: Jack Schultz

clear; clc; close all

% Part a) Plot the world

OB = [0 0; 12 0; 12 12; 0 12; 0 0];
qs = [1 1];
qg = [8 7];
% qs = [0 0];
% qg = [12 12];

figure
plot(OB(:,1),OB(:,2))
hold on
plot(qs(1), qs(2), '.', 'MarkerSize', 20)
text(qs(1)-0.6,qs(2)-0.5,'START','Color','red','FontSize',14)
plot(qg(1), qg(2), '.', 'MarkerSize', 20)
text(qg(1)-0.6,qg(2)-0.5,'GOAL','Color','red','FontSize',14)
axis([-1 13 -1 13])
% axis('square')

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

data.parent = 0;
data.node = qs;
i = 1;
E = inf;
flag = 0;
shortest_E = inf;

while E > .2
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
        [in, on] = inpolygon(v(r,1), v(r,2), obstacle(:,1),obstacle(:,2));
        [in2, on2] = inpolygon(v(r,1), v(r,2), obstacle2(:,1),obstacle2(:,2));
        [in3, on3] = inpolygon(v(r,1), v(r,2), obstacle3(:,1),obstacle3(:,2));
        [in4, on4] = inpolygon(v(r,1), v(r,2), obstacle4(:,1),obstacle4(:,2));
        [in5, on5] = inpolygon(v(r,1), v(r,2), obstacle5(:,1),obstacle5(:,2));
        if      in == 1 ||  on == 1 ...
            || in2 == 1 || on2 == 1 ...
            || in3 == 1 || on3 == 1 ...
            || in4 == 1 || on4 == 1 ...
            || in5 == 1 || on5 == 1
        
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
    
    if i >= 1000
        warning('Too many iterations. Stopping here and plotting closest point.')
        plot(data.node(index_E,1), data.node(index_E,2), '*k', 'MarkerSize', 8)
        flag = 1;
        break
    end
end

if flag == 0
    disp('Goal node found!')
    title('Goal Node Found!','FontSize',18,'Color','r')
    plot(data.node(index_E,1), data.node(index_E,2), '*k', 'MarkerSize', 8)
end

% Plot the path to the node closest to the goal
j = index_E;
k = data.parent(index_E);

while k ~= 0
    plot([data.node(k,1) data.node(j,1)], [data.node(k,2) data.node(j,2)], 'k', 'LineWidth', 2)
    j = k;
    k = data.parent(j);
end

