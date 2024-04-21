classdef MapGenerate
    %MAPGENERATE class generate random map with obstacle
    
    properties
        x_bnd
        y_bnd
        occupy
        resol
        space  
        obstacles % an cell array to store obetacles
    end
    
    methods
        function obj = MapGenerate(x_bnd,y_bnd,space,resol)
            %MAPGENERATE Construct a map generator
            obj.x_bnd = x_bnd;
            obj.y_bnd = y_bnd;
            obj.space = space;
            obj.occupy = zeros(x_bnd*resol,y_bnd*resol); 
            obj.resol = resol; % resolution of the map
        end
        
        function [occupy,obj] = addBounds(obj,s)
            % add boundary to the occupancy grid
            x_max = size(obj.occupy,1);
            y_max = size(obj.occupy,2);
            for i = 1:x_max
                for j = 1:y_max
                    if ((i<=s) || (i>=x_max-s) || (j<=s) || (j>=y_max-s))
                        obj.occupy(i,j) = 1;
                    end
                end
            end
            occupy = obj.occupy;
        end
        
        function [occupy,obj] = addRectangle(obj,x_start, y_start, width, height)
            x_max = size(obj.occupy,1);
            y_max = size(obj.occupy,2);
            x_end = x_start + height - 1;
            y_end = y_start + width - 1;
            for i = max(1, x_start):min(x_end, x_max)
                for j = max(1, y_start):min(y_end, y_max)
                    obj.occupy(i, j) = 1;
                end
            end
            occupy = obj.occupy;
        end
        
        function [obj,occupy,success] = addObstacle(obj,xy,size)
            % add obstacle to occupy matrix based on
            % type of obstacle: circle
            % position: (x,y)
            % size: radius of the obstacle
            success = true;
            numObstacles = length(obj.obstacles);
            new_obst = Circle(xy,size);
            bound = new_obst.bound_box;
            l = bound(1); t = bound(2); r = bound(3); b = bound(4);
            A = [l t]; B = [r t]; C = [r b]; D = [l b];
            apex = [A;B;C;D];
            for i = 1:4
                p = apex(i,:);
                for j = 1:numObstacles
                    obst = obj.obstacles{j};
                    if (obst.InBoundBox(p)==true)
                        success = false;
                        break;
                    end
                end
            end
            if (success==true)
                obj = obj.addPoints(new_obst);
                obj.obstacles{numObstacles+1} = new_obst;
            end
            occupy = obj.occupy;
        end
        
        function occupied = testOccupied(obj,xy)
            x = xy(1); y = xy(2);
            x_coord = round(x*obj.resol);
            y_coord = round(y*obj.resol);
            if (obj.occupy(x_coord,y_coord) == 1)
                occupied = true;
            else
                occupied = false;
            end
        end
        
        function obj = addPoints(obj,obstacle)
            x_size = size(obj.occupy,1);
            y_size = size(obj.occupy,2);
            step = 1/obj.resol;
            for i = 1:x_size
                for j = 1:y_size
                    x = i*step; y = j*step;
                    if (obstacle.InObstacle([x y]))
                        obj.occupy(i,j) = 1;
                    else
                        continue;
                    end
                end
            end
        end
    end
end

