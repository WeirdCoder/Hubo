classdef MISOJoiner < MIMODrakeSystem
    %Non Hubo-specific module
    % @param input_frames given cells of frames as input{}.
    % @retval joined output from the input.
    
    %The module is not yet tested.
    properties
        numInFrame
    end
    methods
        function obj = MISOJoiner(input_frames)
            typecheck(input_frames,'cell');
            input_frame = MultiCoordinateFrame(input_frames);
            numOut = 0;
            numInFrame = size(input_frames);
            for i = 1:(max(numInFrame))
                frame = input_frames{i};
                input_size = size(frame.getCoordinateNames());
                numOut = numOut + input_size(1);
                
            end
            output_frame = MultiCoordinateFrame.constructFrame({CoordinateFrame('output',numOut,'y')});
            obj@MIMODrakeSystem(0,0,input_frame,output_frame,true, true);
            obj.numInFrame = max(numInFrame);
        end
        function yout=mimoOutput(obj,t,~,varargin)        
            yout = {};
            x = 1;
            for i = 1:obj.numInFrame
                InFrame = varargin{i};
                InSize = size(InFrame);
                for j  = 1:InSize(1)
                    yout{x} = InFrame(j);
                    x = x + 1;
                end
            end
            yout = yout';
        end
    end
end