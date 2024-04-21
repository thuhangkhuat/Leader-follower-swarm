%% a test script for simulator class
clear all;
close all;

addpath(genpath('E:\STUDY_UET_MS\Dêcntralize system\LeaderFollower'))

% generate map for the simulation
size = 15;
resolution = 10;
numObstacles = 1;
space = 5;
%p = zeros(size*resolution);
map_gen = MapGenerate(size,size,space,resolution);
[p,map_gen] = map_gen.addBounds(2);
for i = 1:numObstacles
    [p,map_gen] = map_gen.addRectangle(20,20, 15, 15);
     [p,map_gen] = map_gen.addRectangle(40,40, 15, 15);
    [p,map_gen] = map_gen.addRectangle(50,70, 20, 20);
    [p,map_gen] = map_gen.addRectangle(120,100, 15, 15);
     [p,map_gen] = map_gen.addRectangle(15,80, 15, 15);
     [p,map_gen] = map_gen.addRectangle(120,40, 15, 15);
     [p,map_gen] = map_gen.addRectangle(80,20, 20, 20);
      [p,map_gen] = map_gen.addRectangle(70,110, 20, 20);
     [p,map_gen] = map_gen.addRectangle(85,75, 15, 15);
end
map = binaryOccupancyMap(p,resolution);

%% specify some parameters
form = VShapeFormation();
numRobots = form.numRobots;
numSensors = 5;
sensorRange = 2.5;
showTraj = false;
initial_poses = 8*(rand(3,numRobots).*[0.5;0.5;0]) + [0.5;0.5;0];
robotInfos = cell(1,numRobots);
for i = 1:numRobots
    t = "DiffDrive"; % differential drive dynamics
    R = 0.1; 
    L = 0.5;
    s = numSensors;
    r = sensorRange;
    show = showTraj;
    robotInfo = RobotInfo(t,R,L,s,r);
    robotInfos{i} = robotInfo;
end
swarmInfo = SwarmInfo(numRobots,robotInfos,initial_poses,false);
%% leader-follower simulation
sim = LeaderFollowerSimulation(map,swarmInfo,form);
for i = 1:700
    sim = sim.step();
    axis([0 size 0 size])
    pause(0.02);
end
