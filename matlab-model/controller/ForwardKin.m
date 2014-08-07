classdef ForwardKin < MIMODrakeSystem
    %Non Hubo-specific module
    %Assume two input given
    % @param r the robot model. This module will produce the state of the
    % chosen end effector
    % @retval qdot the control velocity for each joint of the robot.
    
    %The module is not yet generalized and tested.
    properties
        robot % Robot Obj (TimeSteppingRigidBodyManipulator)
        rotType%Rotational Type of the given Point
        targetEndEffector%Target EndEffector LinkID
        pts
    end
    methods
        function obj = ForwardKin(r)
            typecheck(r,'TimeSteppingRigidBodyManipulator');
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('EffX',12,'x')}); 
            input_frame = MultiCoordinateFrame.constructFrame({r.getStateFrame()}); 
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true)
            obj.robot = r;
            obj.targetEndEffector = r.findLinkInd('Body_RightWristPitch'); %TODO remove Hubo Specific Parts
            obj.rotType = 0;
            endEff_pts = [];
            for i=1:length(r.getBody(obj.targetEndEffector).getContactShapes),
              endEff_pts = [endEff_pts r.getBody(obj.targetEndEffector).getContactShapes{i}.getPoints];
            end
            obj.pts =  endEff_pts(:,2);;%mean(endEff_pts,2);
        end
        function x=mimoOutput(obj,t,x,varargin)        
            q = varargin{1};
            kinSol = obj.robot.doKinematics(q,false,false);
            [x, J] = obj.robot.forwardKin(kinSol,obj.targetEndEffector,obj.pts,obj.rotType); 
            x(7:9) = J*q(obj.robot.getNumDOF+1:2*obj.robot.getNumDOF);
        end
    end
end