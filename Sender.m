classdef Sender < handle

    properties (Constant)
        
        Name='Arduino Mega';        % Name of Arduino
        %Serial Definition
        Baudrate=9600;              %Baudrate
        Parity='none';              %Parity
        Timeout=0.1;                %Timeout
        DataBits=8;                 %DataBits
        StopBit=1;                  %StopBit
        InputBufferSize=2048;
        
    end
    
    properties 
        
        COMPort;                    % Comport
        serial_obj;                 % Serial Object
        InputBuffer;                % Inputbuffer
        idx;                        % Index
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
        %Constructor of Sender()
        function obj=Sender()
            obj.COMPort='';
            obj.serial_obj=[];
        end
        
        %Initialization of Sender()
        function init(obj,COM)
            
            Sender.deinit();
            obj.COMPort=COM;
            obj.serial_obj=serial(obj.COMPort);
            set(obj.serial_obj,'DataBits',obj.DataBits);
            set(obj.serial_obj,'StopBit',obj.StopBit);
            set(obj.serial_obj,'BaudRate',obj.Baudrate);
            set(obj.serial_obj,'Parity',obj.Parity);
            set(obj.serial_obj,'Timeout',obj.Timeout);
            set(obj.serial_obj,'InputBufferSize',obj.InputBufferSize);
            obj.InputBuffer=uint8(zeros(1,obj.InputBufferSize));
            obj.idx=1;
            fopen(obj.serial_obj);
            mbox=msgbox('Serial Communication setup');
            uiwait(mbox);
            pause(1);
            
        end
        
        function read_write(obj,Outputdata)
            fwrite(obj.serial_obj,Outputdata,'uint8');
            obj.read();
        end
        
        function read(obj)
           [back,len]=fread(obj.serial_obj,256 ,'uint8');
           obj.InputBuffer(obj.idx:obj.idx+len-1)=back;
           obj.idx=obj.idx+len+1;
        end
        
        function write(obj,Outputdata)
            fwrite(obj.serial_obj,Outputdata,'uint8');
        end
        
        function packet=getPacket(obj)
            for i=1:obj.idx
               if obj.InputBuffer(i)==uint8(83)
                   if obj.InputBuffer(i+1)==uint8(84)
                       if obj.InputBuffer(i+2)==uint8(65)
                           if obj.InputBuffer(i+3)==uint8(82)
                               if obj.InputBuffer(i+4)==uint8(84)
                               len=obj.InputBuffer(i+5);
                               packet=obj.InputBuffer(i+6:i+6+len);
                               InputBuffer_old=obj.InputBuffer(double(i+6+len):end);
                               obj.InputBuffer=zeros(1,obj.InputBufferSize);
                               obj.InputBuffer(1:length(InputBuffer_old))=InputBuffer_old;
                               return;
                               end
                           end
                       end
                   end
               end
            end
        end   
    end
end