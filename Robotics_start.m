%% Function that starts the COM ports for serial communication

closeSerial();
comPort='COM14';
[Sender,serialFlag]=setupSerial(comPort);

while(1)
    res=fscanf(Sender);
    disp(res);
    strcmp();
end