classdef LeaderFollowerSimulation < simulation
    %LEADERFOLLOWERSIMULATION 
    % A Leader-Follower formation control simulation
    
    properties
        form
        numFollowers
    end
    
    methods
        function obj = LeaderFollowerSimulation(map,swarmInfo,form)
            %LEADERFOLLOWERSIMULATION 
            obj.sampleTime = 0.05;
            obj.numRobots = swarmInfo.numRobots;
            obj.numFollowers = obj.numRobots - 1;
            obj.world = world(swarmInfo);
            obj.controllers = cell(1,obj.numRobots);
            % assign controller for the leader(PRM waypoints)
            leader_goal = [12 12];
            leader_pose = swarmInfo.poses(:,1);
            leader_wps = planPRM(map,leader_pose,leader_goal);
            obj.controllers{1} = DiffDrivePursueWayPoints(leader_wps);
            % assign controller for the followers
            obj.form = form;%LineFormation();
            for i = 2:obj.numRobots
                type = obj.form.getType(i);
                params = obj.form.getParam(i);
                obj.controllers{i} = DiffDriveFollower(type,params);
            end
            % assign actuators
            for i = 1:obj.numRobots
                robotInfo = swarmInfo.infos{i};
                R = robotInfo.wheel_radius;
                L = robotInfo.body_width;
                if (robotInfo.type == "DiffDrive")
                    obj.actuators{i} = actuatorDiffDrive(R,L);
                end
            end
        end
        
        function obj = step(obj)
            readings = obj.sensor_phase();
            controls = obj.control_phase(readings);
            poses = obj.actuate_phase_(controls);
            obj = obj.physics_phase(poses);
            obj.visualize_();
        end
        
        function controls = control_phase(obj,readings)
            % get current poses of all robots
            poses = obj.world.get_poses();
            controls = cell(1,obj.numRobots);
            % control the leader
            ctl = obj.controllers{1};
            pose = poses(:,1);
            controls{1} = ctl.compute_control(pose,readings);
            % control the followers
            for i = 2:obj.numRobots
                ctl = obj.controllers{i};
                type = obj.form.getType(i);
                leadIdx = obj.form.getIdx(i);
                pose = poses(:,i);
                if (strcmp(type,"dphi"))
                    lead = poses(:,leadIdx);
                    controls{i} = ctl.compute_control(pose,lead,[0;0;0]);
                elseif (strcmp(type,"dd"))
                    lead1 = poses(:,leadIdx(1));
                    lead2 = poses(:,leadIdx(2));
                    controls{i} = ctl.compute_control(pose,lead1,lead2);
                end
            end
        end
    end
end

