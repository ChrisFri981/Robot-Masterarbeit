classdef Sender <handle
    %SENDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        
        Name='Arduino Mega';        % Name of Arduino
        %Serial Definition
        Baudrate=9600;              %Baudrate
        Parity='none';              %Parity
        Timeout=10;                %Timeout
        DataBits=8;                 %DataBits
        StopBit=1;                  %StopBit
    end
    
    properties 
        COMPort;                    % Comport
        serial_obj;                 % Serial Object
        %Definition of Motor
        Roboter;                   %Cell with Motors[i]             
    end
    methods (Static)
        function deinit()
            if ~isempty(instrfind)
                fclose(instrfind);
                delete(instrfind);
            end
            close all;
            clc;
            disp('Serial port closed');
        end
    end   
    methods
        function obj=Sender()
            obj.COMPort='';
            obj.serial_obj=[];
        end
        
        function init(obj,COM)
            closeSerial();
            obj.COMPort=COM;
            obj.serial_obj=serial(obj.COMPort);
            set(obj.serial_obj,'DataBits',obj.DataBits);
            set(obj.serial_obj,'StopBit',obj.StopBit);
            set(obj.serial_obj,'BaudRate',obj.Baudrate);
            set(obj.serial_obj,'Parity',obj.Parity);
            pause(2);
            fopen(obj.serial_obj);
            mbox=msgbox('Serial Communication setup');
            uiwait(mbox);
            pause(2);
        end
        
        function init_Robot(obj,i)
           obj.Roboter=Robot(i);
        end
        
        function Gyrodata=getGyro(obj,MotorNumber)
         str=['G' int2str(MotorNumber)];
         fwrite(obj.serial_obj,str);
         Gyrodata=zeros(6,1);
            for i=1:6
                Gyrodata(i)=str2double(fscanf(obj.serial_obj));
            end
          obj.Roboter.Motors(MotorNumber)
        end
        
        function Pipes=find_Motor(obj)
            str='F';
            fwrite(obj.serial_obj,str);
           Pipes=cell(6,1);
           for i=1:6
               Pipes(i)=fscanf(obj.serial_obj);
           end
        end
    end
end

