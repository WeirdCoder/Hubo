classdef VelocityRBMController < MIMODrakeSystem
    properties
        numQ
        P
        I
    end
    methods
        function obj = VelocityRBMController(r,P,I)
            typecheck(r, 'TimeSteppingRigidBodyManipulator');
            obj.numQ = r.getDOF; %TODO update to getNumVelocity
            input_frame = MultiCoordinateFrame({r.getStateFrame(),CoordinateFrame('qdot_desired',obj.numQ,'q')});
            output_frame = MultiCoordinateFrame.constructFrame(CoordinateFrame('F_control',obj.numQ,'f'));
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true)
            obj.setOutputFrame(output_frame);
            obj.setInputFrame(input_frame);
            obj.P = P;
            obj.I = I;
        end
        function f_control = mimoOutput(obj,t,x,varargin)
            qqdot_curr = varargin(1);
            qdot_des = varagin(2);
            f_control = obj.I*(qdot_des - qqdot_curr(obj.numQ:end));
            if ~empty(x)
               dt = t - x(1);
               qddot_curr = (1/dt)*(qqdot_curr(obj.numQ:end)-x(2:end));
               f_control = f_control + obj.P*qddot_curr;
            end
        end
        function xdn = mimoUpdate(obj,t,~,varargin)
            qqdot_curr = varargin(1);
            xdn = [t qqdot_curr(obj.numQ:end)];
        end
    end
end