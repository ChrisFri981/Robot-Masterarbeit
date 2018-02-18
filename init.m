 clc;
 clear all;
 close all;
 Sender.deinit;
 
 Number=2;          %Number of Motors
 COM='COM15';
 
 % Object Sender
 S=Sender;
 % Initialization of the Serial-Communication
 S.init(COM);
 % Initialization of the Motors
M1=Motor(1,S);
M2=Motor(2,S);
M1.COMM_SET_CURRENT(1.0)
S.read()
packet=S.getPacket();

ack=S.InputBuffer.buffer_array(1:30);
Sender.deinit();