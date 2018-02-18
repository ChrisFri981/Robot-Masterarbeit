classdef Motor < handle 
    %Motor Class
    properties
        
        Number;                     %Number of Motor
        ACK;                        %Old Commant ACK
        COMMAND;                    %Actual Command set
        FAULT;                      %Get Fault Message
        pipe;                       %pipe Adress
        Gyroparameter;              %Gyrovalues
        Aktual_Position;            %Actual Postition
        New_Position;               %New Position
        conf;                       %configuration
        Arduino_UART;               %SerialObjekt
        
    end
    
    methods
        
        %Constructor of Motor
        function obj = Motor(number,serial)
            obj.Number=number;
            obj.pipe='';
            obj.Gyroparameter=zeros(1,6);
            obj.Aktual_Position=-1;
            obj.New_Position=-1;
            obj.conf=datatypes();
            obj.Arduino_UART=serial;
        end
        
        function COMM_FW_VERSION(obj)
            disp('TODO');
        end
        function COMM_JUMP_TO_BOOTLOADER(obj)
            disp('TODO');
        end
        function COMM_ERASE_NEW_APP(obj)
            disp('TODO');
        end
        function COMM_WRITE_NEW_APP_DATA(obj)
            disp('TODO');
        end
        function COMM_GET_VALUES(obj)
            disp('TODO');
        end
        function COMM_SET_DUTY(obj)
            disp('TODO');
        end
        
        % Sets the current of the VESC
        function COMM_SET_CURRENT(obj,current)            
            b=buffer_vesc(13);
            b.buffer_append_start();
            b.buffer_append_int8(6);
            b.buffer_append_int8(obj.Number);
            b.buffer_append_int8(datatypes.set_COMM_PACKET_ID('COMM_SET_CURRENT'));
            b.buffer_append_int32(int32(current*1000));
            b.buffer_append_checksum();
            obj.ACK=false;
            obj.COMMAND='COMM_SET_CURRENT';
            count=0;
            while obj.ACK==false
                count=count+1;
                if(count>=obj.Arduino_UART.Max_Retries)
                    break
                end
                obj.Arduino_UART.write(b.buffer_array);   
                obj.Arduino_UART.read();
                obj.check_ACK();
            end
        end
        
        function COMM_SET_CURRENT_BRAKE(obj,brake)
            b=buffer_vesc(6);
            b.buffer_append_int8(obj.Number);
            b.buffer_append_int8(datatypes.set_COMM_PACKET_ID('COMM_SET_CURRENT_BRAKE'));
            b.buffer_append_int32(int32(brake*1000));
            b.buffer_append_checksum();
            obj.Arduino_UART.write(b.buffer_array);
        end
        
        function COMM_SET_RPM(obj)
            disp('TODO');
        end
        function COMM_SET_POS(obj)
            disp('TODO');
        end
        function COMM_SET_DETECT(obj)
            disp('TODO');
        end
        function COMM_SET_SERVO_POS(obj)
            disp('TODO');
        end
        function COMM_SET_MCCONF(obj)
            disp('TODO');
        end
        function COMM_GET_MCCONF(obj)
            disp('TODO');
        end
        function COMM_GET_MCCONF_DEFAULT(obj)
            disp('TODO');
        end
        function COMM_SET_APPCONF(obj)
            disp('TODO');
        end
        function COMM_GET_APPCONF(obj)
            disp('TODO');
        end
        function COMM_GET_APPCONF_DEFAULT(obj)
            disp('TODO');
        end
        function COMM_SAMPLE_PRINT(obj)
            disp('TODO');
        end
        function COMM_TERMINAL_CMD(obj)
           disp('TODO');
        end
        function COMM_PRINT(obj)
            disp('TODO');
        end
        function COMM_ROTOR_POSITION(obj)
            disp('TODO');
        end
        function COMM_EXPERIMENT_SAMPLE(obj)
            disp('TODO');
        end
        function COMM_DETECT_MOTOR_PARAM(obj)
            disp('TODO');
        end
        function COMM_DETECT_MOTOR_R_L(obj)
            disp('TODO');
        end
        function COMM_DETECT_MOTOR_FLUX_LINKAGE(obj)
            disp('TODO');
        end
        function COMM_DETECT_ENCODER(obj)
            disp('TODO');
        end
        function COMM_DETECT_HALL_FOC(obj)
            disp('TODO');
        end
        function COMM_REBOOT(obj)
            disp('TODO');
        end
        function COMM_ALIVE(obj)
            disp('TODO');
        end
        function COMM_GET_DECODED_PPM(obj)
            disp('TODO');
        end
        function COMM_GET_DECODED_ADC(obj)
            disp('TODO');
        end
        function COMM_GET_DECODED_CHUK(obj)
            disp('TODO');
        end
        function COMM_FORWARD_CAN(obj)
            disp('TODO');
        end
        function COMM_SET_CHUCK_DATA(obj)
            disp('TODO');
        end
        function COMM_CUSTOM_APP_DATA(obj)
            disp('TODO');
        end
        function COMM_NON(obj)
            disp('TODO');
        end
        function check_ACK(obj)
         obj.Arduino_UART.getPacket();
         if(obj.Arduino_UART.packet_buf~=-1)
             if(obj.my_packet)
              if(obj.Arduino_UART.packet_buf.buffer_array(3)==datatypes.set_COMM_PACKET_ID(obj.COMMAND)...
                  && obj.Arduino_UART.packet_buf.buffer_array(4)== 65 && obj.Arduino_UART.packet_buf.buffer_array(5)== 67 ...
                  && obj.Arduino_UART.packet_buf.buffer_array(6)== 75)
              obj.ACK=true;
              obj.Arduino_UART.packet_buf=-1;
              end
             end
         end
        end
        function res=my_packet(obj)
            res =obj.Arduino_UART.packet_buf.buffer_array(2)==obj.Number;
        end
    end
end

