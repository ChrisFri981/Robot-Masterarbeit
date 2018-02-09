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
        
        function buffer=Sender_write(obj,Outputdata)
          buffer=init_buffer(Outputdata);
         fwrite(obj.serial_obj,Outputdata);
            for i=1:6
                buffer(i)=fscanf(obj.serial_obj);
            end
        end
        function init_buffer(Outputdata)
        switch Outputdata
            case 'A0'
            case 'A1'
            case 'A2'
            case 'A3'
            case 'A4'
            case 'A5'
            case 'A6'
            case 'A7'
            case 'A8'
            case 'A9'
            case 'B1'
            case 'B2'
            case 'B3'
            case 'B4'
            case 'B5'
            case 'B6'
            case 'B7'
            case 'B8'
            case 'B9'
            case 'C1'
            case 'C2'
            case 'C3'
            case 'C4'
            case 'C5'
            case 'C6'
            case 'C7'
            case 'C8'
            case 'C9'
            case 'D1'
            case 'D2'
            case 'D3'  
            case 'D4'    
            case 'D5'    
            case 'D6'    
            case 'D7'    
            case 'D8'  
            case 'D9'     
            case 'E1'     
            otherwise
                disp('FAILURE');
        end       
    end
end

