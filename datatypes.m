classdef datatypes
  properties 
        
    mc_rpm_dep_struct;
% 	float cycle_int_limit;
% 	float cycle_int_limit_running;
% 	float cycle_int_limit_max;
% 	float comm_time_sum;
% 	float comm_time_sum_min_rpm;
% 	int32_t comms;
% 	uint32_t time_at_comm

    mc_configuration;
% 	// Switching and drive
% 	mc_pwm_mode pwm_mode;
% 	mc_comm_mode comm_mode;
% 	mc_motor_type motor_type;
% 	mc_sensor_mode sensor_mode;
% 	// Limits
% 	float l_current_max;
% 	float l_current_min;
% 	float l_in_current_max;
% 	float l_in_current_min;
% 	float l_abs_current_max;
% 	float l_min_erpm;
% 	float l_max_erpm;
% 	float l_max_erpm_fbrake;
% 	float l_max_erpm_fbrake_cc;
% 	float l_min_vin;
% 	float l_max_vin;
% 	float l_battery_cut_start;
% 	float l_battery_cut_end;
% 	bool l_slow_abs_current;
% 	bool l_rpm_lim_neg_torque;
% 	float l_temp_fet_start;
% 	float l_temp_fet_end;
% 	float l_temp_motor_start;
% 	float l_temp_motor_end;
% 	float l_min_duty;
% 	float l_max_duty;
% 	// Overridden limits (Computed during runtime)
% 	float lo_current_max;
% 	float lo_current_min;
% 	float lo_in_current_max;
% 	float lo_in_current_min;
% 	// Sensorless
% 	float sl_min_erpm;
% 	float sl_min_erpm_cycle_int_limit;
% 	float sl_max_fullbreak_current_dir_change;
% 	float sl_cycle_int_limit;
% 	float sl_phase_advance_at_br;
% 	float sl_cycle_int_rpm_br;
% 	float sl_bemf_coupling_k;
% 	// Hall sensor
% 	int8_t hall_table[8];
% 	float hall_sl_erpm;
% 	// FOC
% 	float foc_current_kp;
% 	float foc_current_ki;
% 	float foc_f_sw;
% 	float foc_dt_us;
% 	float foc_encoder_offset;
% 	bool foc_encoder_inverted;
% 	float foc_encoder_ratio;
% 	float foc_motor_l;
% 	float foc_motor_r;
% 	float foc_motor_flux_linkage;
% 	float foc_observer_gain;
% 	float foc_pll_kp;
% 	float foc_pll_ki;
% 	float foc_duty_dowmramp_kp;
% 	float foc_duty_dowmramp_ki;
% 	float foc_openloop_rpm;
% 	float foc_sl_openloop_hyst;
% 	float foc_sl_openloop_time;
% 	float foc_sl_d_current_duty;
% 	float foc_sl_d_current_factor;
% 	mc_foc_sensor_mode foc_sensor_mode;
% 	uint8_t foc_hall_table[8];
% 	float foc_sl_erpm;
% 	// Speed PID
% 	float s_pid_kp;
% 	float s_pid_ki;
% 	float s_pid_kd;
% 	float s_pid_min_erpm;
% 	// Pos PID
% 	float p_pid_kp;
% 	float p_pid_ki;
% 	float p_pid_kd;
% 	float p_pid_ang_div;
% 	// Current controller
% 	float cc_startup_boost_duty;
% 	float cc_min_current;
% 	float cc_gain;
% 	float cc_ramp_step_max;
% 	// Misc
% 	int32_t m_fault_stop_time_ms;
% 	float m_duty_ramp_step;
% 	float m_duty_ramp_step_rpm_lim;
% 	float m_current_backoff_gain;
% 	uint32_t m_encoder_counts;
% 	sensor_port_mode m_sensor_port_mode;

    ppm_config;
% 	ppm_control_type ctrl_type;
% 	float pid_max_erpm;
% 	float hyst;
% 	float pulse_start;
% 	float pulse_end;
% 	bool median_filter;
% 	bool safe_start;
% 	float rpm_lim_start;
% 	float rpm_lim_end;
% 	bool multi_esc;
% 	bool tc;
% 	float tc_max_diff;

    adc_config;
% 	adc_control_type ctrl_type;
% 	float hyst;
% 	float voltage_start;
% 	float voltage_end;
% 	bool use_filter;
% 	bool safe_start;
% 	bool cc_button_inverted;
% 	bool rev_button_inverted;
% 	bool voltage_inverted;
% 	float rpm_lim_start;
% 	float rpm_lim_end;
% 	bool multi_esc;
% 	bool tc;
% 	float tc_max_diff;
% 	uint32_t update_rate_hz;



    chuk_config;
% 	chuk_control_type ctrl_type;
% 	float hyst;
% 	float rpm_lim_start;
% 	float rpm_lim_end;
% 	float ramp_time_pos;
% 	float ramp_time_neg;
% 	float stick_erpm_per_s_in_cc;
% 	bool multi_esc;
% 	bool tc;
% 	float tc_max_diff;

    nrf_config;
% 	NRF_SPEED speed;
% 	NRF_POWER power;
% 	NRF_CRC crc_type;
% 	NRF_RETR_DELAY retry_delay;
% 	unsigned char retries;
% 	unsigned char channel;
% 	unsigned char address[3];
% 	bool send_crc_ack;

    app_configuration;
% 	// Settings
% 	uint8_t controller_id;
% 	uint32_t timeout_msec;
% 	float timeout_brake_current;
% 	bool send_can_status;
% 	uint32_t send_can_status_rate_hz;
% 
% 	// Application to use
% 	app_use app_to_use;
% 
% 	// PPM application settings
% 	ppm_config app_ppm_conf;
% 
% 	// ADC application settings
% 	adc_config app_adc_conf;
% 
% 	// UART application settings
% 	uint32_t app_uart_baudrate;
% 
% 	// Nunchuk application settings
% 	chuk_config app_chuk_conf;
% 
% 	// NRF application settings
% 	nrf_config app_nrf_conf;

    fault_data;
% 	mc_fault_code fault;
% 	float current;
% 	float current_filtered;
% 	float voltage;
% 	float duty;
% 	float rpm;
% 	int tacho;
% 	int cycles_running;
% 	int tim_val_samp;
% 	int tim_current_samp;
% 	int tim_top;
% 	int comm_step;
% 	float temperature;

    chuck_data;
% 	int js_x;
% 	int js_y;
% 	int acc_x;
% 	int acc_y;
% 	int acc_z;
% 	bool bt_c;
% 	bool bt_z;

    mote_state;
% 	uint8_t js_x;
% 	uint8_t js_y;
% 	bool bt_c;
% 	bool bt_z;
% 	bool bt_push;
% 	float vbat;

    mc_values;
% 	float v_in;
% 	float temp_mos1;
% 	float temp_mos2;
% 	float temp_mos3;
% 	float temp_mos4;
%     float temp_mos5;
%     float temp_mos6;
%     float temp_pcb;
%     float current_motor;
%     float current_in;
%     float rpm;
%     float duty_now;
%     float amp_hours;
%     float amp_hours_charged;
%     float watt_hours;
%     float watt_hours_charged;
%     int tachometer;
%     int tachometer_abs;
%     mc_fault_code fault_code;

    bldcMeasure;
% 	// Values int16_t not read(14 byte)
% 	float avgMotorCurrent;
% 	float avgInputCurrent;
% 	float dutyCycleNow;
% 	long rpm;
% 	float inpVoltage;
% 	float ampHours;
% 	float ampHoursCharged;
% 	//2 values int32_t not read (8 byte)
% 	long tachometer;
% 	long tachometerAbs;

  end
    
    methods (Static)
        
        function obj=datatypes()
            obj.mc_rpm_dep_struct=cell(1,7);
            obj.mc_configuration=cell(1,86);
            obj.ppm_config=cell(1,12);
            obj.adc_config=cell(1,15);
            obj.chuk_config=cell(1,10);
            obj.nrf_config=cell(1,8);
            obj.app_configuration=cell(1,11);
            obj.fault_data=cell(1,13);
            obj.chuck_data=cell(1,7);
            obj.mote_state=cell(1,6);
            obj. mc_values=cell(1,19);
            obj.bldcMeasure=cell(1,10);
        end
        
        function res=set_mc_state(state)
            switch state
                case 'MC_STATE_OFF'
                    res=0;
                case   'MC_STATE_DETECTING'
                    res=1;
                case  'MC_STATE_RUNNING'
                    res=2;
                case 'MC_STATE_FULL_BRAKE'
                    res=3;
                otherwise
                    res=255;
            end
        end
        
        function res=set_mc_pwm_mode(state)
                switch state
                    case 'PWM_MODE_NONSYNCHRONOUS_HISW'
                        res=0;
                    case'PWM_MODE_SYNCHRONOUS'
                        res=1;
                    case'PWM_MODE_BIPOLAR'
                        res=2;
                    otherwise
                        res=255;
                end
        end
        
        function res=set_mc_comm_mode(state)
            switch state
                case 'COMM_MODE_INTEGRATE'
                    res=0;
                case 'COMM_MODE_DELAY'
                    res=1;
                otherwise
                    res=255;
            end
        end
        
        function res=set_mc_sensor_mode(state)
            switch state
                case'SENSOR_MODE_SENSORLESS'
                    res=0;
                case'SENSOR_MODE_SENSORED'
                    res=1;
                case'SENSOR_MODE_HYBRID'
                    res=2;
                otherwise
                    res=255;
            end
        end
        
        function res=set_mc_foc_sensor_mode(state)
            switch state
                case'FOC_SENSOR_MODE_SENSORLESS'
                    res=0;
                case'FOC_SENSOR_MODE_ENCODER'
                    res=1;
                case'FOC_SENSOR_MODE_HALL'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_mc_motor_type(state)
            switch state
                case'MOTOR_TYPE_BLDC'
                    res=0;
                case'MOTOR_TYPE_DC'
                    res=1;
                case'MOTOR_TYPE_FOC'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_mc_fault_code(state)
            switch state
                case'FAULT_CODE_NONE'
                    res=0;
                case  'FAULT_CODE_OVER_VOLTAGE'
                    res=1;
                case   'FAULT_CODE_UNDER_VOLTAGE'
                    res=2;
                case  'FAULT_CODE_DRV8302'
                    res=3;
                case  'FAULT_CODE_ABS_OVER_CURRENT'
                    res=4;
                case  'FAULT_CODE_OVER_TEMP_FET'
                    res=5;
                case  'FAULT_CODE_OVER_TEMP_MOTOR'
                    res=6;
                otherwise
                    res=255;
            end
        end
        
        function res=set_mc_control_mode(state)
            switch state
                case'CONTROL_MODE_DUTY'
                    res=0;
                case'CONTROL_MODE_SPEED'
                    res=1;
                case'CONTROL_MODE_CURRENT'
                    res=2;
                case'CONTROL_MODE_CURRENT_BRAKE'
                    res=3;
                case 'CONTROL_MODE_POS'
                    res=4;
                case'CONTROL_MODE_NONE'
                    res=5;
                otherwise
                    res=255;
            end
        end
        function res=set_disp_pos_mode(state)
                switch state
                    case'DISP_POS_MODE_NONE'
                        res=0;
                    case'DISP_POS_MODE_INDUCTANCE'
                        res=1;
                    case'DISP_POS_MODE_OBSERVER'
                        res=2;
                    case'DISP_POS_MODE_ENCODER'
                        res=3;
                    case'DISP_POS_MODE_PID_POS'
                        res=4;
                    case'DISP_POS_MODE_PID_POS_ERROR'
                        res=5;
                    case'DISP_POS_MODE_ENCODER_OBSERVER_ERROR'
                        res=6;
                    otherwise
                        res=255;
                end
        end
        function res=set_sensor_port_mode(state)
            switch state
                case'SENSOR_PORT_MODE_HALL'
                    res=0;
                case'SENSOR_PORT_MODE_ABI'
                    res=1;
                case'SENSOR_PORT_MODE_AS5047_SPI'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_app_use(state)
            switch state
                case'APP_NONE'
                    res=0;
                case'APP_PPM'
                    res=1;
                case'APP_ADC'
                    res=2;
                case'APP_UART'
                    res=3;
                case'APP_PPM_UART'
                    res=4;
                case'APP_ADC_UART'
                    res=5;
                case'APP_NUNCHUK'
                    res=6;
                case'APP_NRF'
                    res=7;
                case'APP_CUSTOM'
                    res=8;
                otherwise
                    res=255;
            end
        end
        function res=set_ppm_control_type(state)
            switch state
                case'PPM_CTRL_TYPE_NONE'
                    res=0;
                case'PPM_CTRL_TYPE_CURRENT'
                    res=1;
                case'PPM_CTRL_TYPE_CURRENT_NOREV'
                    res=2;
                case'PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE'
                    res=3;
                case'PPM_CTRL_TYPE_DUTY'
                    res=4;
                case'PPM_CTRL_TYPE_DUTY_NOREV'
                    res=5;
                case'PPM_CTRL_TYPE_PID'
                    res=6;
                case'PPM_CTRL_TYPE_PID_NOREV'
                    res=7;
                otherwise
                    res=255;
            end
        end
        function res=set_adc_control_type(state)
            switch(state)
                case'ADC_CTRL_TYPE_NONE'
                    res=0;
                case'ADC_CTRL_TYPE_CURRENT'
                    res=1;
                case'ADC_CTRL_TYPE_CURRENT_REV_CENTER'
                    res=2;
                case'ADC_CTRL_TYPE_CURRENT_REV_BUTTON'
                    res=3;
                case'ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER'
                    res=4;
                case'ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON'
                    res=5;
                case'ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC'
                    res=6;
                case'ADC_CTRL_TYPE_DUTY'
                    res=7;
                case'ADC_CTRL_TYPE_DUTY_REV_CENTER'
                    res=8;
                case'ADC_CTRL_TYPE_DUTY_REV_BUTTON'
                    res=9;
                otherwise
                    res=255;
            end
        end
        function res=set_chuk_control_type(state)
            switch(state)
                case'CHUK_CTRL_TYPE_NONE'
                    res=0;
                case'CHUK_CTRL_TYPE_CURRENT'
                    res=1;
                case'CHUK_CTRL_TYPE_CURRENT_NOREV'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_NRF_SPEED(state)
            switch(state)
                case'NRF_SPEED_250K'
                    res=0;
                case'NRF_SPEED_1M'
                    res=1;
                case'NRF_SPEED_2M'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_NRF_POWER(state)
            switch state
                case 'NRF_POWER_M18DBM'
                    res=0;
                case'NRF_POWER_M12DBM'
                    res=1;
                case'NRF_POWER_M6DBM'
                    res=2;
                case'NRF_POWER_0DBM'
                    res=3;
                otherwise
                    res=255;
            end
        end
        function res=set_NRF_AW(state)
            switch(state)
                case'NRF_AW_3'
                    res=0;
                case'NRF_AW_4'
                    res=1;
                case'NRF_AW_5'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_NRF_CRC(state)
            switch(state)
                case'NRF_CRC_DISABLED'
                    res=0;
                case'NRF_CRC_1B'
                    res=1;
                case'NRF_CRC_2B'
                    res=2;
                otherwise
                    res=255;
            end
        end
        function res=set_NRF_RETR_DELAY(state)
            switch(state)
                case'NRF_RETR_DELAY_250US'
                    res=0;
                case'NRF_RETR_DELAY_500US'
                    res=1;
                case'NRF_RETR_DELAY_750US'
                    res=2;
                case'NRF_RETR_DELAY_1000US'
                    res=3;
                case'NRF_RETR_DELAY_1250US'
                    res=4;
                case'NRF_RETR_DELAY_1500US'
                    res=5;
                case'NRF_RETR_DELAY_1750US'
                    res=6;
                case'NRF_RETR_DELAY_2000US'
                    res=7;
                case'NRF_RETR_DELAY_2250US'
                    res=8;
                case'NRF_RETR_DELAY_2500US'
                    res=9;
                case'NRF_RETR_DELAY_2750US'
                    res=10;
                case'NRF_RETR_DELAY_3000US'
                    res=11;
                case'NRF_RETR_DELAY_3250US'
                    res=12;
                case'NRF_RETR_DELAY_3500US'
                    res=13;
                case'NRF_RETR_DELAY_3750US'
                    res=14;
                case'NRF_RETR_DELAY_4000US'
                    res=15;
                otherwise
                    res=255;
            end
        end
        function res=set_COMM_PACKET_ID(state)
            switch(state)
                case'COMM_FW_VERSION'
                    res=0;
                case'COMM_JUMP_TO_BOOTLOADER'
                    res=1;
                case'COMM_ERASE_NEW_APP'
                    res=2;
                case'COMM_WRITE_NEW_APP_DATA'
                    res=3;
                case'COMM_GET_VALUES'
                    res=4;
                case'COMM_SET_DUTY'
                    res=5;
                case'COMM_SET_CURRENT'
                    res=6;
                case'COMM_SET_CURRENT_BRAKE'
                    res=7;
                case'COMM_SET_RPM'
                    res=8;
                case'COMM_SET_POS'
                    res=9;
                case'COMM_SET_DETECT'
                    res=10;
                case'COMM_SET_SERVO_POS'
                    res=11;
                case'COMM_SET_MCCONF'
                    res=12;
                case'COMM_GET_MCCONF'
                    res=13;
                case'COMM_GET_MCCONF_DEFAULT'
                    res=14;
                case'COMM_SET_APPCONF'
                    res=15;
                case'COMM_GET_APPCONF'
                    res=16;
                case'COMM_GET_APPCONF_DEFAULT'
                    res=17;
                case'COMM_SAMPLE_PRINT'
                    res=18;
                case'COMM_TERMINAL_CMD'
                    res=19;
                case'COMM_PRINT'
                    res=20;
                case'COMM_ROTOR_POSITION'
                    res=21;
                case'COMM_EXPERIMENT_SAMPLE'
                    res=22;
                case'COMM_DETECT_MOTOR_PARAM'
                    res=23;
                case'COMM_DETECT_MOTOR_R_L'
                    res=24;
                case'COMM_DETECT_MOTOR_FLUX_LINKAGE'
                    res=25;
                case'COMM_DETECT_ENCODER'
                    res=26;
                case'COMM_DETECT_HALL_FOC'
                    res=27;
                case'COMM_REBOOT'
                    res=28;
                case'COMM_ALIVE'
                    res=29;
                case'COMM_GET_DECODED_PPM'
                    res=30;
                case'COMM_GET_DECODED_ADC'
                    res=31;
                case'COMM_GET_DECODED_CHUK'
                    res=32;
                case'COMM_FORWARD_CAN'
                    res=33;
                case'COMM_SET_CHUCK_DATA'
                    res=34;
                case'COMM_CUSTOM_APP_DATA'
                    res=35;
                case'COMM_NON'
                    res=36;
                otherwise
                    res=255;
            end
        end
        function res=set_CAN_PACKET_ID(state)
            switch(state)
                case'CAN_PACKET_SET_DUTY'
                    res=0;
                case'CAN_PACKET_SET_CURRENT'
                    res=1;
                case'CAN_PACKET_SET_CURRENT_BRAKE'
                    res=2;
                case'CAN_PACKET_SET_RPM'
                    res=3;
                case'CAN_PACKET_SET_POS'
                    res=4;
                case'CAN_PACKET_FILL_RX_BUFFER'
                    res=5;
                case'CAN_PACKET_FILL_RX_BUFFER_LONG'
                    res=6;
                case'CAN_PACKET_PROCESS_RX_BUFFER'
                    res=7;
                case'CAN_PACKET_PROCESS_SHORT_BUFFER'
                    res=8;
                case'CAN_PACKET_STATUS'
                    res=9;
                otherwise
                    res=255;
            end
        end
        function res=set_LED_EXT_STATE(state)
            switch(state)
                case'LED_EXT_OFF'
                    res=0;
                case'LED_EXT_NORMAL'
                    res=1;
                case'LED_EXT_BRAKE'
                    res=2;
                case'LED_EXT_TURN_LEFT'
                    res=3;
                case'LED_EXT_TURN_RIGHT'
                    res=4;
                case'LED_EXT_BRAKE_TURN_LEFT'
                    res=5;
                case'LED_EXT_BRAKE_TURN_RIGHT'
                    res=6;
                case'LED_EXT_BATT'
                    res=7;
                otherwise
                    res=255;
            end
        end
        function res=set_MOTE_PACKET(state)
            switch(state)
                case'MOTE_PACKET_BATT_LEVEL'
                    res=0;
                case'MOTE_PACKET_BUTTONS'
                    res=1;
                case'MOTE_PACKET_ALIVE'
                    res=2;
                case'MOTE_PACKET_FILL_RX_BUFFER'
                    res=3;
                case'MOTE_PACKET_FILL_RX_BUFFER_LONG'
                    res=4;
                case'MOTE_PACKET_PROCESS_RX_BUFFER'
                    res=5;
                case'MOTE_PACKET_PROCESS_SHORT_BUFFER'
                    res=6;
                otherwise
                    res=255;
            end
        end
                
        function res=get_mc_state(state)
            switch state
                case 0
                    res='MC_STATE_OFF';
                case 1  
                    res='MC_STATE_DETECTING';
                case 2 
                    res='MC_STATE_RUNNING';
                case 3
                    res='MC_STATE_FULL_BRAKE';
                otherwise
                    res='NONE';
            end
        end
        
        function res=get_mc_pwm_mode(state)
                switch state
                    case 0 
                        res='PWM_MODE_NONSYNCHRONOUS_HISW';
                    case 1 
                        res='PWM_MODE_SYNCHRONOUS';
                    case 2 
                        res='PWM_MODE_BIPOLAR';
                    otherwise
                        res='NONE';
                end
        end
        
        function res=get_mc_comm_mode(state)
            switch state
                case 0 
                    res='COMM_MODE_INTEGRATE';
                case 1 
                    res='COMM_MODE_DELAY';
                otherwise
                    res='NONE';
            end
        end
        
        function res=get_mc_sensor_mode(state)
            switch state
                case 0 
                    res='SENSOR_MODE_SENSORLESS';
                case 1 
                    res='SENSOR_MODE_SENSORED';
                case 2 
                    res='SENSOR_MODE_HYBRID';
                otherwise
                    res='NONE';
            end
        end
        
        function res=get_mc_foc_sensor_mode(state)
            switch state
                case 0 
                    res='FOC_SENSOR_MODE_SENSORLESS';
                case 1 
                    res='FOC_SENSOR_MODE_ENCODER';
                case 2 
                    res='FOC_SENSOR_MODE_HALL';
                otherwise
                    res='NONE';
            end
        end
        function res=get_mc_motor_type(state)
            switch state
                case 0 
                    res='MOTOR_TYPE_BLDC';
                case 1 
                    res='MOTOR_TYPE_DC';
                case 2 
                    res='MOTOR_TYPE_FOC';
                otherwise
                    res='NONE';
            end
        end
        function res=get_mc_fault_code(state)
            switch state
                case 0 
                    res='FAULT_CODE_NONE';
                case 1 
                    res='FAULT_CODE_OVER_VOLTAGE';
                case 2 
                    res='FAULT_CODE_UNDER_VOLTAGE';
                case 3 
                    res='FAULT_CODE_DRV8302';
                case 4 
                    res='FAULT_CODE_ABS_OVER_CURRENT';
                case 5 
                    res='FAULT_CODE_OVER_TEMP_FET';
                case 6 
                    res='FAULT_CODE_OVER_TEMP_MOTOR';
                otherwise
                    res='NONE';
            end
        end
        
        function res=get_mc_control_mode(state)
            switch state
                case 0 
                    res='CONTROL_MODE_DUTY';
                case 1 
                    res='CONTROL_MODE_SPEED';
                case 2 
                    res='CONTROL_MODE_CURRENT';
                case 3 
                    res='CONTROL_MODE_CURRENT_BRAKE';
                case 4 
                    res='CONTROL_MODE_POS';
                case 5 
                    res='CONTROL_MODE_NONE';
                otherwise
                    res='NONE';
            end
        end
        function res=get_disp_pos_mode(state)
                switch state
                    case 0 
                        res='DISP_POS_MODE_NONE';
                    case 1 
                        res='DISP_POS_MODE_INDUCTANCE';
                    case 2 
                        res='DISP_POS_MODE_OBSERVER';
                    case 3 
                        res='DISP_POS_MODE_ENCODER';
                    case 4 
                        res='DISP_POS_MODE_PID_POS';
                    case 5 
                        res='DISP_POS_MODE_PID_POS_ERROR';
                    case 6 
                        res='DISP_POS_MODE_ENCODER_OBSERVER_ERROR';
                    otherwise
                        res='NONE';
                end
        end
        function res=get_sensor_port_mode(state)
            switch state
                case 0 
                    res='SENSOR_PORT_MODE_HALL';
                case 1 
                    res='SENSOR_PORT_MODE_ABI';
                case 2 
                    res='SENSOR_PORT_MODE_AS5047_SPI';
                otherwise
                    res='NONE';
            end
        end
        function res=get_app_use(state)
            switch state
                case 0 
                    res='APP_NONE';
                case 1 
                    res='APP_PPM';
                case 2 
                    res='APP_ADC';
                case 3 
                    res='APP_UART';
                case 4 
                    res='APP_PPM_UART';
                case 5 
                    res='APP_ADC_UART';
                case 6 
                    res='APP_NUNCHUK';
                case 7 
                    res='APP_NRF';
                case 8 
                    res='APP_CUSTOM';
                otherwise
                    res='NONE';
            end
        end
        function res=get_ppm_control_type(state)
            switch state
                case 0 
                    res='PPM_CTRL_TYPE_NONE';
                case 1 
                    res='PPM_CTRL_TYPE_CURRENT';
                case 2 
                    res='PPM_CTRL_TYPE_CURRENT_NOREV';
                case 3 
                    res='PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE';
                case 4 
                    res='PPM_CTRL_TYPE_DUTY';
                case 5 
                    res='PPM_CTRL_TYPE_DUTY_NOREV';
                case 6 
                    res='PPM_CTRL_TYPE_PID';
                case 7 
                    res='PPM_CTRL_TYPE_PID_NOREV';
                otherwise
                    res='NONE';
            end
        end
        function res=get_adc_control_type(state)
            switch(state)
                case 0 
                    res='ADC_CTRL_TYPE_NONE';
                case 1 
                    res='ADC_CTRL_TYPE_CURRENT';
                case 2 
                    res='ADC_CTRL_TYPE_CURRENT_REV_CENTER';
                case 3 
                    res='ADC_CTRL_TYPE_CURRENT_REV_BUTTON';
                case 4 
                    res='ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER';
                case 5 
                    res='ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON';
                case 6 
                    res='ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC';
                case 7 
                    res='ADC_CTRL_TYPE_DUTY';
                case 8 
                    res='ADC_CTRL_TYPE_DUTY_REV_CENTER';
                case 9 
                    res='ADC_CTRL_TYPE_DUTY_REV_BUTTON';
                otherwise
                    res='NONE';
            end
        end
        function res=get_chuk_control_type(state)
            switch(state)
                case 0 
                    res='CHUK_CTRL_TYPE_NONE';
                case 1 
                    res='CHUK_CTRL_TYPE_CURRENT';
                case 2 
                    res='CHUK_CTRL_TYPE_CURRENT_NOREV';
                otherwise
                    res='NONE';
            end
        end
        function res=get_NRF_SPEED(state)
            switch(state)
                case 0 
                    res='NRF_SPEED_250K';
                case 1 
                    res='NRF_SPEED_1M';
                case 2 
                    res='NRF_SPEED_2M';
                otherwise
                    res='NONE';
            end
        end
        function res=get_NRF_POWER(state)
            switch state
                case 0 
                    res='NRF_POWER_M18DBM';
                case 1 
                    res='NRF_POWER_M12DBM';
                case 2 
                    res='NRF_POWER_M6DBM';
                case 3 
                    res='NRF_POWER_0DBM';
                otherwise
                    res='NONE';
            end
        end
        function res=get_NRF_AW(state)
            switch(state)
                case 0 
                    res='NRF_AW_3';
                case 1 
                    res='NRF_AW_4';
                case 2 
                    res='NRF_AW_5';
                otherwise
                    res='NONE';
            end
        end
        function res=get_NRF_CRC(state)
            switch(state)
                case 0 
                    res='NRF_CRC_DISABLED';
                case 1 
                    res='NRF_CRC_1B';
                case 2 
                    res='NRF_CRC_2B';
                otherwise
                    res='NONE';
            end
        end
        function res=get_NRF_RETR_DELAY(state)
            switch(state)
                case 0 
                    res='NRF_RETR_DELAY_250US';
                case 1 
                    res='NRF_RETR_DELAY_500US';
                case 2 
                    res='NRF_RETR_DELAY_750US';
                case 3 
                    res='NRF_RETR_DELAY_1000US';
                case 4 
                    res='NRF_RETR_DELAY_1250US';
                case 5 
                    res='NRF_RETR_DELAY_1500US';
                case 6 
                    res='NRF_RETR_DELAY_1750US';
                case 7 
                    res='NRF_RETR_DELAY_2000US';
                case 8 
                    res='NRF_RETR_DELAY_2250US';
                case 9 
                    res='NRF_RETR_DELAY_2500US';
                case 10 
                    res='NRF_RETR_DELAY_2750US';
                case 11 
                    res='NRF_RETR_DELAY_3000US';
                case 12 
                    res='NRF_RETR_DELAY_3250US';
                case 13 
                    res='NRF_RETR_DELAY_3500US';
                case 14 
                    res='NRF_RETR_DELAY_3750US';
                case 15 
                    res='NRF_RETR_DELAY_4000US';
                otherwise
                    res='NONE';
            end
        end
        function res=get_COMM_PACKET_ID(state)
            switch(state)
                case 0 
                    res='COMM_FW_VERSION';
                case 1 
                    res='COMM_JUMP_TO_BOOTLOADER';
                case 2 
                    res='COMM_ERASE_NEW_APP';
                case 3 
                    res='COMM_WRITE_NEW_APP_DATA';
                case 4 
                    res='COMM_GET_VALUES';
                case 5 
                    res='COMM_SET_DUTY';
                case 6 
                    res='COMM_SET_CURRENT';
                case 7 
                    res='COMM_SET_CURRENT_BRAKE';
                case 8 
                    res='COMM_SET_RPM';
                case 9 
                    res='COMM_SET_POS';
                case 10 
                    res='COMM_SET_DETECT';
                case 11 
                    res='COMM_SET_SERVO_POS';
                case 12 
                    res='COMM_SET_MCCONF';
                case 13 
                    res='COMM_GET_MCCONF';
                case 14 
                    res='COMM_GET_MCCONF_DEFAULT';
                case 15 
                    res='COMM_SET_APPCONF';
                case 16 
                    res='COMM_GET_APPCONF';
                case 17 
                    res='COMM_GET_APPCONF_DEFAULT';
                case 18 
                    res='COMM_SAMPLE_PRINT';
                case 19 
                    res='COMM_TERMINAL_CMD';
                case 20 
                    res='COMM_PRINT';
                case 21 
                    res='COMM_ROTOR_POSITION';
                case 22 
                    res='COMM_EXPERIMENT_SAMPLE';
                case 23 
                    res='COMM_DETECT_MOTOR_PARAM';
                case 24 
                    res='COMM_DETECT_MOTOR_R_L';
                case 25 
                    res='COMM_DETECT_MOTOR_FLUX_LINKAGE';
                case 26 
                    res='COMM_DETECT_ENCODER';
                case 27 
                    res='COMM_DETECT_HALL_FOC';
                case 28 
                    res='COMM_REBOOT';
                case 29 
                    res='COMM_ALIVE';
                case 30 
                    res='COMM_GET_DECODED_PPM';
                case 31 
                    res='COMM_GET_DECODED_ADC';
                case 32 
                    res='COMM_GET_DECODED_CHUK';
                case 33 
                    res='COMM_FORWARD_CAN';
                case 34 
                    res='COMM_SET_CHUCK_DATA';
                case 35 
                    res='COMM_CUSTOM_APP_DATA';
                case 36 
                    res='COMM_NON';
                otherwise
                    res='NONE';
            end
        end
        function res=get_CAN_PACKET_ID(state)
            switch(state)
                case 0 
                    res='CAN_PACKET_SET_DUTY';
                case 1 
                    res='CAN_PACKET_SET_CURRENT';
                case 2
                    res='CAN_PACKET_SET_CURRENT_BRAKE';
                case 3 
                    res='CAN_PACKET_SET_RPM';
                case 4 
                    res='CAN_PACKET_SET_POS';
                case 5 
                    res='CAN_PACKET_FILL_RX_BUFFER';
                case 6 
                    res='CAN_PACKET_FILL_RX_BUFFER_LONG';
                case 7 
                    res='CAN_PACKET_PROCESS_RX_BUFFER';
                case 8 
                    res='CAN_PACKET_PROCESS_SHORT_BUFFER';
                case 9 
                    res='CAN_PACKET_STATUS';
                otherwise
                    res='NONE';
            end
        end
        function res=get_LED_EXT_STATE(state)
            switch(state)
                case 0 
                    res='LED_EXT_OFF';
                case 1 
                    res='LED_EXT_NORMAL';
                case 2
                    res= 'LED_EXT_BRAKE';
                case 3 
                    res='LED_EXT_TURN_LEFT';
                case 4 
                    res='LED_EXT_TURN_RIGHT';
                case 5 
                    res='LED_EXT_BRAKE_TURN_LEFT';
                case 6 
                    res='LED_EXT_BRAKE_TURN_RIGHT';
                case 7
                    res= 'LED_EXT_BATT';
                otherwise
                    res='NONE';
            end
        end
        function res=get_MOTE_PACKET(state)
            switch(state)
                case 0 
                    res='MOTE_PACKET_BATT_LEVEL';
                case 1 
                    res='MOTE_PACKET_BUTTONS';
                case 2 
                    res='MOTE_PACKET_ALIVE';
                case 3 
                    res='MOTE_PACKET_FILL_RX_BUFFER';
                case 4 
                    res='MOTE_PACKET_FILL_RX_BUFFER_LONG';
                case 5 
                    res='MOTE_PACKET_PROCESS_RX_BUFFER';
                case 6 
                    res='MOTE_PACKET_PROCESS_SHORT_BUFFER';
                otherwise
                    res='NONE';
            end
        end
    end
end