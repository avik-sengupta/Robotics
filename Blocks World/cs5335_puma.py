from openravepy import *
from numpy import *
#from velctl import *
import time

# Public functions

## release the held block on the Buffer Stack
def placeBlockOnBuffer(robot,ikmodel,basemanip,env,b,Tpick0):
	Tplace = Tpick0.copy()
    	Tplace[2,3] = Tplace[2,3] + b;
    	Tplace[0,3] = Tplace[0,3] 
    	solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
	# Drop object
    	robot.ReleaseAllGrabbed()
	print("place on Buffer");

## release the held block on the initial stack 
def placeBlockOnInit(robot,ikmodel,basemanip,env,i,Tpick):
	Tplace = Tpick.copy()
    	Tplace[2,3] = Tplace[2,3] + i;
    	Tplace[0,3] = Tplace[0,3] 
    	solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
	# Drop object
    	robot.ReleaseAllGrabbed()
	print("place on Init");

## release the held block on the Goal stack
def placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,l):
	# Move to place location
	Tplace = Tpick0.copy()
    	Tplace[2,3] = Tplace[2,3] + l
    	Tplace[0,3] = Tplace[0,3] - 0.4
    	solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
	# Drop object
    	robot.ReleaseAllGrabbed()
	print("place on Goal");

## pick the given block from Initial stack
def pickBlockFromInit(robot,ikmodel,basemanip,env,cylinder,x):
	cyl1 = RaveGetEnvironment(1).GetKinBody(cylinder)
   	Tcyl1 = cyl1.GetTransform()
    	Tpick = eye(4)
    	Tpick[0:2,3] = Tcyl1[0:2,3]
    	Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
	Tpick[2,3] = x
    	solutions = ikmodel.manip.FindIKSolutions(Tpick,True)
	# Move to pick location
 	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
    	# Grab cylinder
    	with env:
        	robot.Grab(env.GetKinBody(cylinder))
	print("Pick from Init");

## pick the given block from Buffer stack
def pickBlockFromBuffer(robot,ikmodel,basemanip,env,cylinder,x):
	cyl1 = RaveGetEnvironment(1).GetKinBody(cylinder)
   	Tcyl1 = cyl1.GetTransform()
    	Tpick = eye(4)
    	Tpick[0:2,3] = Tcyl1[0:2,3]
    	Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
	Tpick[2,3] = x
    	solutions = ikmodel.manip.FindIKSolutions(Tpick,True)
	# Move to pick location
 	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
    	# Grab cylinder
    	with env:
        	robot.Grab(env.GetKinBody(cylinder))
	print("Pick from Buffer");

## pick the given block from Table
def pickBlockFromTable(robot,ikmodel,basemanip,env,cylinder,x):
	cyl1 = RaveGetEnvironment(1).GetKinBody(cylinder)
   	Tcyl1 = cyl1.GetTransform()
    	Tpick = eye(4)
    	Tpick[0:2,3] = Tcyl1[0:2,3]
    	Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
	Tpick[2,3] = x
    	solutions = ikmodel.manip.FindIKSolutions(Tpick,True)
	# Move to pick location
 	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)
    	# Grab cylinder
    	with env:
        	robot.Grab(env.GetKinBody(cylinder))
	print("Pick from Table");

## place the held block on Table
def placeBlockOnTable(robot,ikmodel,basemanip,env,y,Tpick0):
	# Move to place location
	Tplace = Tpick0.copy()
    	Tplace[2,3] = Tplace[2,3] + 0.01
    	Tplace[0,3] = Tplace[0,3] - 0.2
    	solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    	traj = basemanip.MoveActiveJoints(goal=solutions[0])
    	robot.WaitForController(0)

	# Drop object
    	robot.ReleaseAllGrabbed()
	print("place on Table");

## Motion planning for blocks world problem
def algo(Input,Init,Buffer,Table,robot,ikmodel,basemanip,env,Tpick0,Tpick):
    Goal=[];
    y=0.01;
    i=0.01;
    b=0.01;
    bufferPosition=RaveGetEnvironment(1).GetKinBody(Buffer[0]);
    initPosition=RaveGetEnvironment(1).GetKinBody(Init[0]);
    while(len(Input)>0):
	if len(Input)>0 and len(Init)>0 and Input[0]==Init[len(Init)-1]:
		if len(Init)==1:x=0.1;
		if len(Init)==2:x=0.16;
		if len(Init)==3:x=0.22;
		pickBlockFromInit(robot,ikmodel,basemanip,env,Init[len(Init)-1],x);
		
		placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,y);
		y=y+0.07;
		Goal.append(Init[len(Init)-1]);
		Input.remove(Input[0]);
		Init.pop();

	if len(Input)>0 and len(Buffer)>0 and Input[0]==Buffer[len(Buffer)-1]:
		if len(Buffer)==1:x1=0.1;
		if len(Buffer)==2:x1=0.16;
		if len(Buffer)==3:x1=0.23;
		pickBlockFromBuffer(robot,ikmodel,basemanip,env,Buffer[len(Buffer)-1],x1);
		
		placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,y);
		y=y+0.07;
		Goal.append(Buffer[len(Buffer)-1]);
		Input.remove(Input[0]);
		Buffer.pop();

	if len(Input)>0 and Input[0] in Init and Init[len(Init)-1]!=Input[0]:
		while(Init[len(Init)-1]!=Input[0]):
			if len(Init)==1:x=0.1;
			if len(Init)==2:x=0.16;
			if len(Init)==3:x=0.22;
			pickBlockFromInit(robot,ikmodel,basemanip,env,Init[len(Init)-1],x);
			if len(Buffer)==0:b=0.001;
			if len(Buffer)>0:b=0.001+len(Buffer)*0.065;
			if len(Buffer)<3:
				placeBlockOnBuffer(robot,ikmodel,basemanip,env,b,Tpick0);		
				Buffer.append(Init[len(Init)-1]);
				Init.pop();
			else:
				placeBlockOnTable(robot,ikmodel,basemanip,env,y,Tpick0);
				Table.append(Init[len(Init)-1]);
				Init.pop();
		if len(Init)==1:x=0.1;
		if len(Init)==2:x=0.16;
		if len(Init)==3:x=0.22;		
		pickBlockFromInit(robot,ikmodel,basemanip,env,Init[len(Init)-1],x);
		
		placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,y);
		y=y+0.07;
		Goal.append(Init[len(Init)-1])
		Init.pop();
		Input.remove(Input[0]);

## if the next block is in Buffer Stack
	if len(Input)>0 and Input[0] in Buffer and Buffer[len(Buffer)-1]!=Input[0]:
## loop until block is on the top of Buffer
		while(Buffer[len(Buffer)-1]!=Input[0]):
			if len(Buffer)==1:x1=0.1;
			if len(Buffer)==2:x1=0.16;
			if len(Buffer)==3:x1=0.24;
			pickBlockFromBuffer(robot,ikmodel,basemanip,env,Buffer[len(Buffer)-1],x1);
			if len(Init)==0:i=0.001;
			if len(Init)>0:i=0.001+len(Init)*0.065;
			if len(Init)<3:
				placeBlockOnInit		 	 (robot,ikmodel,basemanip,env,i,Tpick);
				Init.append(Buffer[len(Buffer)-1]);
				Buffer.pop();
			else:
				placeBlockOnTable(robot,ikmodel,basemanip,env,y,Tpick0);
				Table.append(Buffer[len(Buffer)-1]);
				Buffer.pop();
		if len(Buffer)==1:x1=0.1;
		if len(Buffer)==2:x1=0.16;
		if len(Buffer)==3:x1=0.23;

		pickBlockFromBuffer(robot,ikmodel,basemanip,env,Buffer[len(Buffer)-1],x1);
		placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,y);
		y=y+0.07;
		Goal.append(Buffer[len(Buffer)-1]);
		Buffer.pop();
		Input.remove(Input[0]);

## if the next block is on the table
	if len(Input)>0 and Input[0] in Table:
		pickBlockFromTable(robot,ikmodel,basemanip,env,Input[0],0.1);
		
		placeBlockOnGoal(robot,ikmodel,basemanip,env,Tpick0,y);
		y=y+0.07;
		Goal.append(Input[0]);
		Table.remove(Input[0]);
		Input.remove(Input[0]);
	
    print(Goal)

## Run Method        
def run():
# Set up environment
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('cs5335.env.xml')
   
    #~ env.Load('../../src/data/puma_rob.env.xml')

	# Set up robot
    robot = env.GetRobots()[0] # get the first robot
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, 	iktype=IkParameterization.Type.Transform6D)
    basemanip=interfaces.BaseManipulation(robot)
    robot.SetActiveDOFs([0, 1, 2, 3, 4, 5])
    print("The color of the cylinders are represented by alphabets as follows:\n");
    a='cylinder_green_3'#blue
    b='cylinder_green_2'#green
    c='cylinder_green_1'#red
    d='cylinder_green_4'#yellow
    e='cylinder_green_5'#pink
    print("a:Blue\nb:Green\nc:Red\nd:Yellow\ne:Pink\n");
    Input=input("Please enter the order of stack using '[' , ']' and ','\n eg: [a,b,c,d,e]:");

    #Input=[b,c,d,e,a]
    Init=[c,b];
    Buffer=[a];
    Table=[d,e];#represents scattered single cylinders on the table
    x=0.1
    y=0.00

## Goal Position Tpick0
    cyl0 = RaveGetEnvironment(1).GetKinBody('cylinder_green_3')
    Tcyl0 = cyl0.GetTransform()
    Tpick0 = eye(4)
    Tpick0[0:2,3] = Tcyl0[0:2,3]
    Tpick0[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
    Tpick0[2,3] = x

## Buffer Position Tpick1
    cyl1 = RaveGetEnvironment(1).GetKinBody('cylinder_green_1')
    Tcyl1 = cyl1.GetTransform()
    Tpick = eye(4)
    Tpick[0:2,3] = Tcyl1[0:2,3]
    Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
    Tpick[2,3] = x
    algo(Input,Init,Buffer,Table,robot,ikmodel,basemanip,env,Tpick0,Tpick);


# Lift arm
    Tplace = Tpick.copy()
    Tplace[2,3] = Tplace[2,3] + 0.2
    solutions = ikmodel.manip.FindIKSolutions(Tplace,True)
    traj = basemanip.MoveActiveJoints(goal=solutions[0])
    robot.WaitForController(0)
    raw_input("Press Enter to continue...")

if __name__ == "__main__":
    run()


