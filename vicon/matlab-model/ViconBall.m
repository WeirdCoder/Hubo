classdef ViconBall < TimeSteppingRigidBodyManipulator
    %Please note that this tracks the largest unlabeled marker (a marker that isn't fit into a tracked object.)
    methods
        function obj = ViconBall(urdf,time,options)
            if nargin < 1 || isempty(urdf)
                urdf = 'ball.urdf';
            end
            
            if nargin < 2
                time = .005;
            end
            if nargin < 3
                options = struct();
                options.floating = true;
            end
            obj = obj@TimeSteppingRigidBodyManipulator(urdf,time,options);
            %State fed by Vicon-LCM
            state_frame = ViconBallState(obj);
            obj = obj.setStateFrame(state_frame);
            obj = obj.setOutputFrame(state_frame);
        end
        function x = track(obj)    
            v = obj.constructVisualizer();
            while true
                [x,t] = obj.getOutputFrame().getNextMessage(1000);
                v.draw(t,x);
                obj.getInputFrame();
%                 pause(0.01);
            end
        end
    end
end