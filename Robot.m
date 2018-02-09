classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Number_Motors;          %Number of Motors
        Motors=Motor;                  %Array of Motors
    end
    
    methods
        function obj = Robot(Number_Motors)
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            
            if nargin ~= 0
            obj.Number_Motors=Number_Motors;
            obj.Motors=Motor;
            obj.Motors(Number_Motors)=Motor;
            
            for i=1:Number_Motors
               obj.Motors(i).Name=['Module' int2str(i)];
            end
            else
                disp('Too few arguments');
                return;
            end
        end
    end
end

