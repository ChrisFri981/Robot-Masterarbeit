 clc;
 clear all;
 close all;
 Sender.deinit;
 
 Number=2;          %Number of Motors
 COM='COM5';
 
 % Object Sender
 S=Sender;
 % Initialization of the Serial-Communication
 S.init(COM);
 % Initialization of the Motors
 S.init_Robot(Number);
 S.getGyro(1);