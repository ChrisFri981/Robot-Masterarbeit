clc;
clear all;
close all;

COMMAND='COMM_FW_VERSION';
switch COMMAND
    case 'COMM_FW_VERSION'
        disp('FW_CF');
    case 'COMM_JUMP_TO_BOOTLOADER'
        disp('JUMP TO BOOTLOADER');
    otherwise 
        disp('No Possible COMMAND');
end