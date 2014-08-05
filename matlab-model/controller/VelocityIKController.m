classdef VelocityIKController < MIMODrakeSystem
    %Non Hubo-specific module
    %Assume two input given
    % @param VelIKCtrlInput_xdot_des the inputframe of the control velocity
    % of the specified end effector of the robot
    % @param r the robot.  This will provide the second input the module
    % need, the state of the robot.
    % It then outputs 1 output
    % @retval qdot the control velocity for each joint of the robot.
    
    %The module is not yet generalized and tested.
    properties
        robot % Robot Obj (TimeSteppingRigidBodyManipulator)
        rotType%Rotational Type of the given Point
        targetEndEffector%Target EndEffector LinkID
        pts
    end
    methods
        function obj = VelocityIKController(r)
            typecheck(r,'TimeSteppingRigidBodyManipulator');
            numQ = r.getNumDOF; %Get number of q (Position Only)   Replace with getNumPositions() when it is available for TimeStepping RigidBodyManipulator
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('VelIKCtrlOutput_qdot_des',numQ,'q')}); %TODO Velocity Control Output Frame
            input_frame = MultiCoordinateFrame({CoordinateFrame('VelIKCtrlInput_xdot_des',3,'x'),r.getStateFrame()}); %TODO setInputFrame
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true)
            obj.robot = r;
            obj.targetEndEffector = r.findLinkInd('Body_RightWristPitch'); %TODO remove Hubo Specific Parts
            obj.rotType = 0;
            endEff_pts = [];
            for i=1:length(r.getBody(obj.targetEndEffector).getContactShapes),
              endEff_pts = [endEff_pts r.getBody(obj.targetEndEffector).getContactShapes{i}.getPoints];
            end
            obj.pts =  endEff_pts(:,2);
        end
        function qdot=mimoOutput(obj,t,~,varargin)        
            xdot_des = varargin{1};
            q = varargin{2};
            kinSol = obj.robot.doKinematics(q,false,false);
            [x,J] = obj.robot.forwardKin(kinSol,obj.targetEndEffector,obj.pts,obj.rotType); %Find Jacobian TODO Find pts
            J_pinv = pinv(J); %Find Inverse of Jacobian (Pseudo)
            qdot = J_pinv*xdot_des;
        end
    end
end