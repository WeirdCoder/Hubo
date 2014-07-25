classdef PIController < MIMODrakeSystem
    %Non Hubo-specific module
    %Assume that the two input given are of same dimensions and represents
    %the state and its deritive.  The outputframe is a simple PI controller
    %of the deritive of the differences in the state.
    
    %This PI Controller is created as such, because most models in Drake
    %can be represented in position and velocity, and the use application
    %for this module is a velocity controller.
    %This module has been generalized, but have not been tested.
    properties
        dim
        P
        I
    end
    methods
        function obj = PIController(dim,P,I)
            % @param dim the dimsion of the position or velocity, but not
            % both
            % @param P the proportional constant or matrix of the
            % controller.
            % @param I the integrational constant or matrix of the
            % controller.
            input_frame = MultiCoordinateFrame({CoordinateFrame('x_des',dim,'x'),CoordinateFrame('x_curr',dim,'x')});
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('xdot_des',dim/2,'x')});
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
            obj.dim = dim;
            obj.P = P;
            obj.I = I;
        end
        function xdot_des=mimoOutput(obj,t,~,varargin)        
            % One control step of the 
            x_des = varargin{1};
            x_curr = varargin{2};
            q = obj.dim;
            xdot_des = (x_des(q:end) - x_curr(q:end))*obj.P + (x_des(1:q) - x_curr(1:q))*obj.I;
        end
    end
end