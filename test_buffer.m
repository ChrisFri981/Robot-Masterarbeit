clc;
clear all;
close all;

SIZE=256;
b=buffer_vesc(256);

number=uint32(188820000);
b.buffer_append_int32(number);
number=uint16(320);
b.buffer_append_int16(number);
b.buffer_append_int16(number);
c=b;
c.index=1;
c.buffer_get_int32()
c.buffer_get_int16()
c.buffer_get_int16()

