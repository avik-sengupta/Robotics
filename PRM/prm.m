% You must run clc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% set up robot and initial joint configuration
    tic
	rob = createRobot();
	
    qStart =[-4.78 -4.78 0 0 -4.78 -4.78];
	xGoal = [0.5;0.0;-0.5];
    sphereCenter = [0.5;-0.5;-0.5];
    sphereCenter1 = [-0.5;0.5;-0.5];
    sphereCenter2 = [-0.5;-0.5;-0.5];
    sphereCenter3 = [0.5;0.5;0.5];
    sphereRadius = 0.2;
    
    % plot robot and sphere
    rob.plot(qStart);
    text(0.5,0.0,-0.5,"Destination")
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    drawSphere(sphereCenter1,sphereRadius);
    drawSphere(sphereCenter2,sphereRadius);
    drawSphere(sphereCenter3,sphereRadius);
    
    [cost,path,V] = milestones(rob,sphereCenter,sphereCenter1,sphereCenter2,sphereCenter3,sphereRadius,qStart,xGoal);
    
    qMilestones=[];
    p=1;
    
    while(p<=length(path))
        qMilestones=[V(path(p),:);qMilestones];
        p=p+1;
    end
   
    %interpolate and plot direct traj from start to goal
    qTraj = interpMilestones(qMilestones);
    rob.plot(qTraj);
    toc         
    

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end


function rob = createRobot()

   % L(1) = Link([0 0 0 1.571]);
   % L(2) = Link([0 0 0 -1.571]);
   % L(3) = Link([0 0.4318 0 -1.571]);
   % L(4) = Link([0 0 0.4318 1.571]);
%     L(5) = Link([0 0.4318 0 1.571]);
    
   % rob = SerialLink(L, 'name', 'robot');
   mdl_puma560
    rob = p560;
   

end

function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end


