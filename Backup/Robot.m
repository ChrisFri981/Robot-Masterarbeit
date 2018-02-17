classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Number_Motors;          %Number of Motors
        Motors;                  %Array of Motors
    end
    
    methods
        function obj = Robot(Number_Motors,Serial)
            %ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            
            if nargin ~= 0
            obj.Number_Motors=Number_Motors;
            obj.Motors(Number_Motors)=Motor(Number_Motors,Serial);
            
            for i=1:Number_Motors-1
               obj.Motors(i)=Motor(i,Serial);
            end
            else
                disp('Too few arguments');
                return;
            end
        end
    end
end

