classdef HuboQPController < MIMODrakeSystem
    %Non Hubo-specific module
    % @param input_frames given cells of frames as input{}.
    % @retval joined output from the input.
    
    %The module is not yet tested.
    properties
        dt
        A
        B
        H
        f
        ub
        lb
    end
    methods
        function obj = HuboQPController(dt)
            input_frame = MultiCoordinateFrame({CoordinateFrame('x_des',6,'x'),CoordinateFrame('x_curr',6,'x')});
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('xdot_des',3,'x')});
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true, true);
            obj.dt = dt;
            load hubo_catching.mat
            obj.A = A;
            obj.B = B;
            obj.H = H;
            obj.f = f;
            obj.ub = ub;
            obj.lb = lb;
        end
        function xdot_des=mimoOutput(obj,~,~,varargin)        
            x_ball = varargin{1};
            x_hand = varargin{2};
            obj.B(1:6) = x_hand;
            obj.B(7:12) = x_ball;
            options.display = 'off';
            qpProblem = QuadraticProgram(obj.H,obj.f,[],[],obj.A,obj.B,[],[]);
            [x,fval,exitflag,output] = qpProblem.solve();
            xdot_des = x(16:18);
            xdot2 = x(28:30);
            xdot3 = x(40:52);
        end
    end
end