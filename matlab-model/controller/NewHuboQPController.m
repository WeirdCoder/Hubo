classdef NewHuboQPController < MIMODrakeSystem
    %Non Hubo-specific module
    % @param input_frames given cells of frames as input{}.
    % @retval joined output from the input.
    
    %The module is not yet tested.
    properties
        dt
        iter
        A
        B
        H
        f
        ub
        lb
    end
    methods
        function obj = NewHuboQPController(dt,iter)
            %iter must be >1.
            input_frame = MultiCoordinateFrame({CoordinateFrame('x_des',6,'x'),CoordinateFrame('x_curr',6,'x')});
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('xdot_des',3,'x')});
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true, true);
            obj.dt = dt;
            obj.iter = iter;
        end
        function xdot_des=mimoOutput(obj,~,~,varargin)        
            %initialize variables
            dt = obj.dt;
            iter = obj.iter;
            x_ball = varargin{1};
            x_hand = varargin{2};
            %Generate unique 1st case.
            A = zeros(12+9*iter,12*iter);
            A(1:12,1:12) = eye(12);
            B = zeros(12+9*iter,1);
            B(1:12) = [x_hand x_ball];
            H = zeros(12*iter);
            f = zeros(12*iter,1);
            %create iterations.
            for i = 2:iter
                %iterate through cases.
                
                %Ball Physics
                A(13+(i-2)*9:13+(i-2)*9+5, (i-2)*12+1:(i)*12) = [[zeros(3,6),eye(3),0.5*dt*eye(3),zeros(3,6),-eye(3),0.5*dt*eye(3)];[zeros(3,9), eye(3),zeros(3,9), -eye(3)]];
                B(18+(i-2)*9) = -9.8*dt;
                %Hand Physics
                A(13+(i-2)*9 + 6:13+(i-2)*9 + 8, (i-2)*12+1:(i)*12) = [eye(3),0.5*dt*eye(3),zeros(3,6),-eye(3),0.5*dt*eye(3),zeros(3,6)];
                
                %Cost Function This iteration in relation to the previous
                    %RMS Trajectory Smoothing
                    seg1 = (i-2)*12+4:(i-2)*12+6;
                    seg2 = (i-1)*12+4:(i-1)*12+6;
                    factor = 5;
                    H(seg1,seg1) = H(seg1,seg1) + factor*eye(3);
                    H(seg2,seg2) = H(seg2,seg2) + factor*eye(3);
                    H(seg1,seg2) = H(seg1,seg2) - factor*eye(3);
                    H(seg2,seg1) = H(seg2,seg1) - factor*eye(3);
                    %Distance Shortening
                    seg1 = (i-1)*12+1:(i-1)*12+3;
                    seg2 = (i-1)*12+7:(i-1)*12+9;
                    factor = 100000.0;
                    H(seg1,seg1) = H(seg1,seg1) + factor*eye(3);
                    H(seg2,seg2) = H(seg2,seg2) + factor*eye(3);
                    H(seg1,seg2) = H(seg1,seg2) - factor*eye(3);
                    H(seg2,seg1) = H(seg2,seg1) - factor*eye(3);
                    %Velocity Shortening
                    pt1 = (i-1)*12+4;
                    pt2 = (i-1)*12+6;
                    pt3 = (i-1)*12+10;
                    pt4 = (i-1)*12+12;
                    factor = 1000;
                    H(pt1,pt1) = H(pt1,pt1) + factor;
                    H(pt2,pt2) = H(pt2,pt2) + factor;
                    H(pt1,pt3) = H(pt1,pt3) - factor;
                    H(pt1,pt4) = H(pt1,pt4) - factor;
                    H(pt3,pt1) = H(pt3,pt1) - factor;
                    H(pt4,pt1) = H(pt4,pt1) - factor;
                    H(pt3,pt3) = H(pt3,pt3) + factor;
                    H(pt4,pt4) = H(pt4,pt4) + factor;
%                     %Velocity Stopping
%                     seg1 = (i-1)*12+4:(i-1)*12+6;
%                     seg2 = (i-1)*12+10:(i-1)*12+12;
%                     factor = 0;
%                     H(seg1,seg1) = H(seg1,seg1) + factor*eye(3);
%                     H(seg2,seg2) = H(seg2,seg2) + factor*eye(3);
%                     H(seg1,seg2) = H(seg1,seg2) - factor*eye(3);
%                     H(seg2,seg1) = H(seg2,seg1) - factor*eye(3);
                    %Velocity Matching
                    seg1 = (i-2)*12+7:(i-1)*12+9;
                    seg2 = (i-2)*12+10:(i-1)*12+12;
                    factor = 0.05;
                    f(seg1) = f(seg1) + factor;
                    f(seg2) = f(seg2) + factor;
                    %Minimize Movement
                    seg1 = (i-1)*12+7:(i-1)*12+9;
                    seg2 = (i-1)*12+10:(i-1)*12+12;
                    factor = 500000;
                    H(seg1,seg1) = H(seg1,seg1) + factor*eye(3);
                    H(seg1,seg2) = H(seg1,seg2) + factor*(-dt)*eye(3);
                    H(seg2,seg1) = H(seg2,seg1) + factor*(-dt)*eye(3);
                    H(seg2,seg2) = H(seg2,seg2) + factor*(dt^2)*eye(3);
                
            end
            %End State
            seg1 = (iter-1)*12+4:(iter-1)*12+6;
            seg2 = (iter-1)*12+10:(iter-1)*12+12;
            factor = 0;
            H(seg1,seg1) = H(seg1,seg1) + factor*eye(3);
            H(seg2,seg2) = H(seg2,seg2) + factor*eye(3);
            H(seg1,seg2) = H(seg1,seg2) - factor*eye(3);
            H(seg2,seg1) = H(seg2,seg1) - factor*eye(3);
            %Assume iter > 1, create exception in the first iteration
            
            qpProblem = QuadraticProgram(H,f,[],[],A,B,[],[]);
            [x,fval,exitflag,output] = qpProblem.solve();
            xdot_des = x(16:18);
        end
    end
end