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
        end
        function COMM_JUMP_TO_BOOTLOADER(obj)  
        end
        function COMM_ERASE_NEW_APP(obj)
        end
        function COMM_WRITE_NEW_APP_DATA(obj)
        end
        function COMM_GET_VALUES(obj)
        end
        function COMM_SET_DUTY(obj)
        end
        function COMM_SET_CURRENT(obj)
        end
        function COMM_SET_CURRENT_BRAKE(obj)
        end
        function COMM_SET_RPM(obj)
        end
        function COMM_SET_POS(obj)
        end
        function COMM_SET_DETECT(obj)
        end
        function COMM_SET_SERVO_POS(obj)
        end
        function COMM_SET_MCCONF(obj)
        end
        function COMM_GET_MCCONF(obj)
        end
        function COMM_GET_MCCONF_DEFAULT(obj)
        end
        function COMM_SET_APPCONF(obj)
        end
        function COMM_GET_APPCONF(obj)
        end
        function COMM_GET_APPCONF_DEFAULT(obj)
        end
        function COMM_SAMPLE_PRINT(obj)
        end
        function COMM_TERMINAL_CMD(obj)
        end
        function COMM_PRINT(obj)
        end
        function COMM_ROTOR_POSITION(obj)
        end
        function COMM_EXPERIMENT_SAMPLE(obj)
        end
        function COMM_DETECT_MOTOR_PARAM(obj)
        end
        function COMM_DETECT_MOTOR_R_L(obj)
        end
        function COMM_DETECT_MOTOR_FLUX_LINKAGE(obj)
        end
        function COMM_DETECT_ENCODER(obj)
        end
        function COMM_DETECT_ENCODER(obj)
        end
        function COMM_DETECT_HALL_FOC(obj)
        end
        function COMM_REBOOT(obj)
        end
        function COMM_ALIVE(obj)
        end
        function COMM_GET_DECODED_PPM(obj)
        end
        function COMM_GET_DECODED_ADC(obj)
        end
        function COMM_GET_DECODED_CHUK(obj)
        end
        function COMM_FORWARD_CAN(obj)
        end
        function COMM_SET_CHUCK_DATA(obj)
        end
        function COMM_CUSTOM_APP_DATA(obj)
        end
        function COMM_NON(obj)
        end
    end
end

