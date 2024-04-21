classdef (Abstract)LFformation
    
    properties
        numRobots
        leaderIdx
        followInfo
    end
    
    
    methods
     
        function type = getType(obj,n)
            % return the type of a follower
            fi = obj.followInfo{n-1};
            type = fi.type;
        end
        
        function idx = getIdx(obj,n)
            % get the index of the robot
            % which number nth robot are following
            idx = obj.leaderIdx{n-1};
        end
        
        function params = getParam(obj,n)
            params = obj.followInfo{n-1};
        end
    end
end

