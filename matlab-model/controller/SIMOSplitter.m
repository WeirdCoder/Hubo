classdef SIMOSplitter < MIMODrakeSystem
    %Non Hubo-specific module
    % @param dim dimension of the given input
    % @param out_dims dimensions of each of the outputs, like so [1,3,2,4]
    % => 1st output from 1:3, 2nd output from 2:4.
    % @retval splitted outputs from the given input.
    
    %The module is not yet tested.
    properties
        dim
        out_dims
    end
    methods
        function obj = SIMOSplitter(dim,out_dims)
            input_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('input',dim,'x')});
            outputframe = {};
            out_dims_size = size(out_dims);
            for i = 1:(out_dims_size(2)/2)
                dim_size = out_dims(2*i+1)-out_dims(2*i+1) +1;
                outputframe{i} = CoordinateFrame('output',dim_size,'x');
            end
            output_frame = MultiCoordinateFrame(outputframe);
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true, true);
            obj.dim = dim;
            obj.out_dims = out_dims;
        end
        function varargout=mimoOutput(obj,t,~,varargin)        
            x = varargin;
            out_dims_size = size(out_dims);
            for i = 1:(out_dims_size(2)/2)
                temp_result = x(out_dims(2*i+1):out_dims(2*i+1));
                varargout(i) = temp_result;
                
            end
            
        end
    end
end