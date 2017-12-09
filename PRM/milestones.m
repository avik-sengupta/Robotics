% Calculate a path from qStart to xGoal
% input: qStart -> 1x6 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%		 sphereCenter2 -> 3x1 position of center of spherical obstacle
%		 sphereCenter3 -> 3x1 position of center of spherical obstacle
%		 sphereCenter4 -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 6xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.
function [cost,path,V] = milestones(rob,sphereCenter,sphereCenter2,sphereCenter3,sphereCenter4,sphereRadius,qStart,xGoal)
    
    qGoal = rob.ikine(transl(xGoal),zeros(1,6),[1,1,1,0,0,0]);
    V=(qStart);
    V=[V;qGoal];
    
    G=[];
    V=getVertices(V,rob,sphereCenter,sphereCenter2,sphereCenter3,sphereCenter4,sphereRadius,qStart);
     
    G=generateGraph(G,V,rob,sphereCenter,sphereCenter2,sphereCenter3,sphereCenter4,sphereRadius);
    
    adjacencyMatrix= generateAdjacencyMatrix(G,V);  
     
    [cost,path] = dijkstra(adjacencyMatrix,1,2);
end

function adjacencyMatrix = generateAdjacencyMatrix(G,V)
[vertextSize,~]=size(V);
     [graphSize,~]=size(G);
    
     adjacencyMatrix=zeros(vertextSize);
    
    
    index=1;
    %loop based on number of vertices
    while(index<=vertextSize)
        graphIndex=1;
        closestNodes=[];
        distance=[];
        adjacencyIndex=1;
        
        % get the nearest node of each joint angles
        while(graphIndex<=graphSize)
            if(isequal(V(index,:),G(graphIndex,1:4)))
                closestNodes=[closestNodes;G(graphIndex,5:8)];
                distance=[distance;G(graphIndex,9)];
            end
            graphIndex=graphIndex+1;
        end
        
        %number of rows in closestNodes
        [r,~]=size(closestNodes);
        
        % index based on number of closest nodes
        while(adjacencyIndex<=r)
            
            %index based on number of total vertices
            tempIndex=1;
           
            while(tempIndex<=vertextSize)
               if(isequal(closestNodes(adjacencyIndex,:),V(tempIndex,:)))
                  
                    adjacencyMatrix(index,tempIndex)=distance(adjacencyIndex);
                    
                    tempIndex=vertextSize+1;
                    
                end
                tempIndex=tempIndex+1;
            end
            
            adjacencyIndex=adjacencyIndex+1;
        end
        
        index=index+1;
    end
end


function graph = generateGraph(G,V,rob,sphereCenter,sphereCenter2,sphereCenter3,sphereCenter4,sphereRadius)

i=1;
[vertextSize,~]=size(V);
     while(i<=vertextSize)
         
         % get 30 closest neighbours
         [n,d] = knnsearch(V,V(i,:),'k',30);
         j=1;
         k=1;
         
         while(j<=30)
             
             
            %if distance is less than 0.09 
            if(d(j)<0.09)
                j=j+1;
                continue;
            end
            
            node=n(j);
            distance=d(j);
            collision = edgeCollision(rob, V(i,:), V(node,:), sphereCenter, sphereRadius);
           
            %if colliding continue
            if(collision)
                j=j+1;
                continue; 
            end
            
            collision = edgeCollision(rob, V(i,:), V(node,:), sphereCenter2, sphereRadius);
           
            %if colliding continue
            if(collision)
                j=j+1;
                continue; 
            end
            
            collision = edgeCollision(rob, V(i,:), V(node,:), sphereCenter3, sphereRadius);
           
            %if colliding continue
            if(collision)
                j=j+1;
                continue; 
            end
            
            collision = edgeCollision(rob, V(i,:), V(node,:), sphereCenter4, sphereRadius);
           
            %if colliding continue
            if(collision)
                j=j+1;
                continue; 
            end
            
            %add both joint angles and edge to graph
            G=[G;V(i,:) V(node,:) distance];
            j=j+1;
            k=k+1;
             
            %add only 10 closest neighbour
            if(k>10)
                break;
            end
            
         end
         
         i=i+1;
     end
     
     graph=G;
end

function vertices = getVertices(V,rob,sphereCenter,sphereCenter2,sphereCenter3,sphereCenter4,sphereRadius,qStart)

 i=1;
    
     while(i<=400)
        %Generating random points
        qRand = getJointConfiguration(qStart);
        
        collision = robotCollision(rob,qRand,sphereCenter,sphereRadius);
        
        %if colliding continue
        if(collision) 
           
            continue; 
        end
        
        collision = robotCollision(rob,qRand, sphereCenter2, sphereRadius);
        
        %if colliding continue
        if(collision) 
            
            continue; 
        end
        
        collision = robotCollision(rob,qRand, sphereCenter3, sphereRadius);
        
        %if colliding continue
        if(collision) 
          
            continue; 
        end
        
        collision = robotCollision(rob,qRand, sphereCenter4, sphereRadius);
        
        %if colliding continue
        if(collision) 
          
            continue; 
        end
        
        V = [V; qRand];
        i=i+1;
     end
     vertices=V;
end

function jointConfig = getJointConfiguration(qStart)
    
   jointConfig = (rand(1,size(qStart,2))*2*pi-pi);
   
end

function index = getNodeIndex(tree,q)
    row = size(tree,1);
    distance     = tree - repmat(q, row, 1);
    distance     = sqrt(sum(distance.^2,2));
    [~, index] = min(distance);
end