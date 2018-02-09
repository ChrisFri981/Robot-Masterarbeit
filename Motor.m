classdef Motor < handle 
    %Motor Class
    properties
        Sender;
        Name;                       %Name of Motor
        pipe;                       %pipe Adress
        Gyroparameter;              %Gyrovalues
        Aktual_Position;            %Actual Postition
        New_Position;               %New Position
        
    end
    
    methods
        function obj = Motor()
        %Constructor of Motor
        obj.Name='';
        obj.pipe='';
        obj.Gyroparameter=zeros(1,6);
        obj.Aktual_Position=-1;
        obj.New_Position=-1;
        end
        
        function set_COMMAND(obj,command_to_vesc)
            switch	command_to_vesc
                case 'COMM_FW_VERSION'
                    obj.COMM_FW_VERSION();
                case 'COMM_JUMP_TO_BOOTLOADER'
                    obj.COMM_JUMP_TO_BOOTLOADER();
                case 'COMM_ERASE_NEW_APP'
                    obj.COMM_ERASE_NEW_APP();
                case 'COMM_WRITE_NEW_APP_DATA'
                    obj.COMM_WRITE_NEW_APP_DATA();
                case 'COMM_GET_VALUES'
                    obj.COMM_GET_VALUES();
                case 'COMM_SET_DUTY'
                    obj.COMM_SET_DUTY();
                case 'COMM_SET_CURRENT'
                    obj.COMM_SET_CURRENT();
                case 'COMM_SET_CURRENT_BRAKE'
                    obj.COMM_SET_CURRENT_BRAKE();
                case 'COMM_SET_RPM'
                    obj.COMM_SET_RPM();
                case 'COMM_SET_POS'
                    obj.COMM_SET_POS();
                case 'COMM_SET_DETECT'
                    obj.COMM_SET_DETECT();
                case 'COMM_SET_SERVO_POS'
                    obj.COMM_SET_SERVO_POS();
                case 'COMM_SET_MCCONF'
                    obj.COMM_SET_MCCONF();
                case 'COMM_GET_MCCONF'
                    obj.COMM_GET_MCCONF();
                case 'COMM_GET_MCCONF_DEFAULT'
                   obj. COMM_GET_MCCONF_DEFAULT();
                case 'COMM_SET_APPCONF'
                    obj.COMM_SET_APPCONF();
                case 'COMM_GET_APPCONF'
                    obj.COMM_GET_APPCONF();
                case 'COMM_GET_APPCONF_DEFAULT'
                    obj.COMM_GET_APPCONF_DEFAULT();
                case 'COMM_SAMPLE_PRINT'
                    obj.OMM_SAMPLE_PRINT();
                case 'COMM_TERMINAL_CMD'
                    obj.COMM_TERMINAL_CMD();
                case 'COMM_PRINT'
                   obj.COMM_PRINT();
                case 'COMM_ROTOR_POSITION'
                    obj.COMM_ROTOR_POSITION();
                case 'COMM_EXPERIMENT_SAMPLE'
                    obj.COMM_EXPERIMENT_SAMPLE();
                case 'COMM_DETECT_MOTOR_PARAM'
                    obj.COMM_DETECT_MOTOR_PARAM();
                case 'COMM_DETECT_MOTOR_R_L'
                    COMM_DETECT_MOTOR_R_L();
                case 'COMM_DETECT_MOTOR_FLUX_LINKAGE'
                    obj.COMM_DETECT_MOTOR_FLUX_LINKAGE();
                case 'COMM_DETECT_ENCODER'
                    obj.COMM_DETECT_ENCODER();
                case 'COMM_DETECT_HALL_FOC'
                    obj.COMM_DETECT_HALL_FOC();
                case 'COMM_REBOOT'
                    obj.COMM_REBOOT();
                case 'COMM_ALIVE'
                    obj.COMM_ALIVE();
                case 'COMM_GET_DECODED_PPM'
                    obj.COMM_GET_DECODED_PPM();
                case 'COMM_GET_DECODED_ADC'
                    obj.COMM_GET_DECODED_ADC();
                case 'COMM_GET_DECODED_CHUK'
                    obj.COMM_GET_DECODED_CHUK();
                case 'COMM_FORWARD_CAN'
                    obj.COMM_FORWARD_CAN();
                case 'COMM_SET_CHUCK_DATA'
                    obj.COMM_SET_CHUCK_DATA();
                case 'COMM_CUSTOM_APP_DATA'
                    obj.COMM_CUSTOM_APP_DATA();
                case 'COMM_NON'
                    obj.COMM_NON();
            end
        end
        function COMM_FW_VERSION(obj)
            obj.Sender.Sender_write('A0');
        end
        function COMM_JUMP_TO_BOOTLOADER(obj)
            obj.Sender.Sender_write('A1');
        end
        function COMM_ERASE_NEW_APP(obj)
            obj.Sender.Sender_write('A2');
        end
        function COMM_WRITE_NEW_APP_DATA(obj)
            obj.Sender.Sender_write('A3');
        end
        function COMM_GET_VALUES(obj)
            obj.Sender.Sender_write('A4');
        end
        function COMM_SET_DUTY(obj)
            obj.Sender.Sender_write('A5');
        end
        function COMM_SET_CURRENT(obj)
            obj.Sender.Sender_write('A6');
        end
        function COMM_SET_CURRENT_BRAKE(obj)
            obj.Sender.Sender_write('A7');
        end
        function COMM_SET_RPM(obj)
            obj.Sender.Sender_write('A8');
        end
        function COMM_SET_POS(obj)
            obj.Sender.Sender_write('A9');
        end
        function COMM_SET_DETECT(obj)
            obj.Sender.Sender_write('B1');
        end
        function COMM_SET_SERVO_POS(obj)
            obj.Sender.Sender_write('B2');
        end
        function COMM_SET_MCCONF(obj)
            obj.Sender.Sender_write('B3');
        end
        function COMM_GET_MCCONF(obj)
            obj.Sender.Sender_write('B4');
        end
        function COMM_GET_MCCONF_DEFAULT(obj)
            obj.Sender.Sender_write('B5');
        end
        function COMM_SET_APPCONF(obj)
            obj.Sender.Sender_write('B6');
        end
        function COMM_GET_APPCONF(obj)
            obj.Sender.Sender_write('B7');
        end
        function COMM_GET_APPCONF_DEFAULT(obj)
            obj.Sender.Sender_write('B8');
        end
        function COMM_SAMPLE_PRINT(obj)
            obj.Sender.Sender_write('B9');
        end
        function COMM_TERMINAL_CMD(obj)
            obj.Sender.Sender_write('C1');
        end
        function COMM_PRINT(obj)
            obj.Sender.Sender_write('C2');
        end
        function COMM_ROTOR_POSITION(obj)
            obj.Sender.Sender_write('C3');
        end
        function COMM_EXPERIMENT_SAMPLE(obj)
            obj.Sender.Sender_write('C4');
        end
        function COMM_DETECT_MOTOR_PARAM(obj)
            obj.Sender.Sender_write('C5');
        end
        function COMM_DETECT_MOTOR_R_L(obj)
            obj.Sender.Sender_write('C6');
        end
        function COMM_DETECT_MOTOR_FLUX_LINKAGE(obj)
            obj.Sender.Sender_write('C7');
        end
        function COMM_DETECT_ENCODER(obj)
            obj.Sender.Sender_write('C8');
        end
        function COMM_DETECT_ENCODER(obj)
            obj.Sender.Sender_write('C9');
        end
        function COMM_DETECT_HALL_FOC(obj)
            obj.Sender.Sender_write('D1');
        end
        function COMM_REBOOT(obj)
            obj.Sender.Sender_write('D2');
        end
        function COMM_ALIVE(obj)
            obj.Sender.Sender_write('D3');
        end
        function COMM_GET_DECODED_PPM(obj)
            obj.Sender.Sender_write('D4');
        end
        function COMM_GET_DECODED_ADC(obj)
            obj.Sender.Sender_write('D5');
        end
        function COMM_GET_DECODED_CHUK(obj)
            obj.Sender.Sender_write('D6');
        end
        function COMM_FORWARD_CAN(obj)
            obj.Sender.Sender_write('D7');
        end
        function COMM_SET_CHUCK_DATA(obj)
            obj.Sender.Sender_write('D8');
        end
        function COMM_CUSTOM_APP_DATA(obj)
            obj.Sender.Sender_write('D9');
        end
        function COMM_NON(obj)
            obj.Sender.Sender_write('E1');
        end
    end
end

