% Valentine's Day RRT
clear; clc; close all;

% ----------------------------------------------------------------------
% ESTABLISH CONFIGURATION SPACE AND OBSTACLES 

% Create the configuration space (heart)
t = linspace(-pi,pi,360);
cx = (16*sin(t).^3);
cy = (13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
plot(cx,cy,'r','LineWidth',2)
set(gcf,'position',[600 50 800 700]);
hold on 

% Create an obstacle (smaller heart)
t = linspace(-pi,pi,360);
ox1 = 3 + (1/16)*(16*sin(t).^3);
oy1 = 3 + (1/16)*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
plot(ox1,oy1,'k'); fill(ox1,oy1,'k')
hold on

% Create an obstacle (smaller heart)
t = linspace(-pi,pi,360);
ox2 = -3 + (1/16)*(16*sin(t).^3);
oy2 = -5 + (1/16)*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
plot(ox2,oy2,'k'); fill(ox2,oy2,'k')
hold on

% Create an obstacle (smaller heart)
t = linspace(-pi,pi,360);
ox3 = -10 + (1/16)*(16*sin(t).^3);
oy3 = 7 + (1/16)*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
plot(ox3,oy3,'k'); fill(ox3,oy3,'k')
hold on

% Create a goal heart (center of heart)
t = linspace(-pi,pi,360);
gx = 2 + (1/10)*(16*sin(t).^3);
gy = -2 + (1/10)*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
plot(gx,gy,'r'); fill(gx,gy,'r')
hold on

% Set goal position (center of heart)
xd = 2; 
yd = -2;
plot(xd,yd,'bo')
hold on

% Set starting position 
xs = -10;
ys = 2;
plot(xs,ys,'go')

% --------------------------------------------------------------------
% BEGIN RRT WITH OBSTACLE DETECTION 

%RRT with obstacle detection

iter = 3000;
vertex(1,:) = [xs ys];
parent(1,:) = [xs ys];
cond = 2;
count = 0;
idx_parent(1,:) = 0;

while cond > 0.1
    count = count + 1;
    count

    rands_all = [cx*rand(1); cy*rand(1)];
    N = length(rands_all);
    n = randi([1,N],1);
    rands = [rands_all(1,n); rands_all(2,n)];
    
    if count > 1500
        rands = [xd yd];
    end
    
    % Check if random point is within CS
    [in,on] = inpolygon(rands(1),rands(2),cx,cy);
        tempx = rands(1); tempy = rands(2);
        if tempx(in) & tempy(in)
            rands = rands;
        elseif (tempx~=in) | (tempy~=in)
            rands_all = [cx*rand(1); cy*rand(1)];
            N = length(rands_all);
            n = randi([1,N],1);
            rands = [rands_all(1,n); rands_all(2,n)];
        end


    % Check if random point is in or on an obstacle
    [rin1,ron1] = inpolygon(rands(1),rands(2),ox1,oy1);
    [rin2,ron2] = inpolygon(rands(1),rands(2),ox2,oy2);
    [rin3,ron3] = inpolygon(rands(1),rands(2),ox3,oy3);
    
    while rin1 == 1 | ron1 == 1 | rin2 == 1 | ron2 == 1 | rin3 == 1  | ron3 == 1
        
        rands_all = [cx*rand(1); cy*rand(1)];
        N = length(rands_all);
        n = randi([1,N],1);
        rands = [rands_all(1,n); rands_all(2,n)];

        [rin1,ron1] = inpolygon(rands(1),rands(2),ox1,oy1);
        [rin2,ron2] = inpolygon(rands(1),rands(2),ox2,oy2);
        [rin3,ron3] = inpolygon(rands(1),rands(2),ox3,oy3);
        
    end
    p = plot(rands(1),rands(2),'r.','MarkerFaceColor','r');
    
    
    for j = 1:1:count
        dist(j,:) = sqrt((vertex(j,1) - rands(1))^2 + (vertex(j,2) - rands(2))^2);
    end
    [val,idx] = min(dist);
    near = vertex(idx,:);
    idx_parent(count+1,1) = idx;
    
    parent(count+1,:) = near;
    
    % Check for obstacle collision
    xq = linspace(rands(1),near(1));
    yq = linspace(rands(2),near(2));

    [IN,ON] = inpolygon(xq,yq,cx,cy);
    [IN1,ON1] = inpolygon(xq,yq,ox1,oy1);
    [IN2,ON2] = inpolygon(xq,yq,ox2,oy2);
    [IN3,ON3] = inpolygon(xq,yq,ox3,oy3);
    
    if IN(1,:) == 1 & IN1(1,:) == 0 & ON1(1,:) == 0 & IN2(1,:) == 0 & ON2(1,:) == 0 & IN3(1,:) == 0 & ON3(1,:) == 0;
        line([rands(1),near(1)],[rands(2),near(2)],'Color','m','LineWidth',0.5);
        drawnow
        hold on
        vertex(count+1,:) = rands;
        
    else IN(1,:) ~= 1 | IN1(1,:) ~= 0 | ON1(1,:) ~= 0 | IN2(1,:) ~= 0 | ON2(1,:) ~= 0 | IN3(1,:) ~= 0 | ON3(1,:) ~= 0;
        delete(p);
        if count == 1
            vertex(count+1,:) = vertex(count,:);
        elseif count > 1
            vertex(count+1,:) = vertex(count-1,:);
        end
    end
    
    
    
    cond = sqrt((vertex(count,1) - xd)^2 + (vertex(count,2) - yd)^2);
    
    % Check if within tolerance of goal position
    if cond <= 0.4
        disp('Solution Found');
        disp(['Number of vertices in tree =   ', num2str(numel(vertex(:,1)))]);
        disp(['Number of edges in tree =      ', num2str((numel(vertex(:,1)))-1)]);
        disp(['Number of iterations =         ', num2str(count)]);
        break
    end
    
    
end

% Trace the path from the final vertex to start position 
hold on
temp_vertex = vertex;
temp_vertex(end,:) = [];

temp_idxparent = idx_parent;
temp_idxparent(end,:) = [];

i(1) = temp_idxparent(end);
k(1) = length(temp_vertex);

while i ~=0
  line([temp_vertex(k,1) temp_vertex(i,1)],[temp_vertex(k,2) temp_vertex(i,2)],'Color','c','LineWidth',3);
  drawnow
  hold on
  k = i;
  i = temp_idxparent(k);
end

% Print title once solution is found 
title('Heart Found - Happy Valentines Day!','FontSize',18,'Color','r')
