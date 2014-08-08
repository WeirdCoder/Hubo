classdef VelocityRBMController < MIMODrakeSystem
    properties
        numQ
        P
        D
    end
    methods
        function obj = VelocityRBMController(r,P,D)
            typecheck(r, 'TimeSteppingRigidBodyManipulator');
            numQ = r.getNumDOF(); %TODO update to getNumVelocity
            input_frame = MultiCoordinateFrame({r.getStateFrame(),CoordinateFrame('qdot_desired',numQ,'q')});
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('F_control',numQ,'f')});
            obj@MIMODrakeSystem(0,29,input_frame,output_frame,true,true)
            obj.setOutputFrame(output_frame);
            obj.setInputFrame(input_frame);
            obj.P = P;
            obj.D = D;
            obj.numQ = numQ;
        end
        function f_control = mimoOutput(obj,t,x,varargin)
            qqdot_curr = varargin{1};
            qdot_des = varargin{2};
            f_control = obj.P*(qdot_des - qqdot_curr(obj.numQ+1:end));
            if x
               dt = t - x(1);
               qddot_curr = (1/dt)*(qqdot_curr(obj.numQ+1:end)-x(2:end));
               f_control = f_control + obj.D*qddot_curr;
            end
        end
        function xdn = mimoUpdate(obj,t,~,varargin)
            qqdot_curr = varargin{1};
            xdn = [t qqdot_curr(obj.numQ+1:end)'];
        end
        function ts = getSampleTime(obj)
            ts = [0.005;0];
        end
    end
end