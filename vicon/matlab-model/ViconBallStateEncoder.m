classdef ViconBallStateEncoder < LCMCoder
    properties
        defaultCoordinateNames
        PreviousTime
        PreviousState
    end
    methods
        function obj = ViconBallStateEncoder(coordinateNames)
            obj.defaultCoordinateNames = coordinateNames;
            obj.PreviousState = [0,0,0,0,0,0,0,0,0,0,0,0];
            obj.PreviousTime = 0;
        end
        function [x,t] = decode(obj,data)
            if ~isempty(data)
                msg = vicon.vicon_ballstate(data.data);
                xdot = 0;
                ydot = 0;
                zdot = 0;
                t = msg.timestamp;
                dt = t - obj.PreviousTime;
                if dt < 1 %Data Gap isn't too wide
                    xdot = (obj.PreviousState(1) - msg.x)/dt;
                    ydot = (obj.PreviousState(2) - msg.y)/dt;
                    zdot = (obj.PreviousState(3) - msg.z)/dt;
                end
                x = [msg.x,msg.y,msg.z, 0, 0, 0,xdot,ydot,zdot,0,0,0]
                obj.PreviousTime = t;
                obj.PreviousState = x;
            else
                x = obj.PreviousState
                t = obj.PreviousTime;
            end
        end
        function msg = encode(obj,t,x)
            msg = vicon.vicon_ballstate();
            msg.timestamp = t;
            msg.x = x(1);
            msg.y = x(2);
            msg.z = x(3);
        end
        function dimmatrix = dim(obj)
            dimmatrix = 6;
        end
        function name = timestampName(obj)
            name = 'timestamp';
        end
        function name = coordinateNames(obj)
            name = obj.defaultCoordinateNames;
        end
    end
end