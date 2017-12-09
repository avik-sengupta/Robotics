% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob,q1,q2,sphereCenter,r)

    Steps = 50;
    
    colums = size(q1,2); 
    
    q = zeros(colums, Steps);
    
    for i=1:colums
        % generates 30 points.The spacing between the points is (x2-x1)/29.
        q(i,:) = linspace(q1(i), q2(i), Steps);
    end
    q = q';
    
    rows = size(q,1);
    
    for i=1:rows
        collision = robotCollision(rob, q(i,:),sphereCenter,r);
        if(collision) 
            break; 
        end
    end
end