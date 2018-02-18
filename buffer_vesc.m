classdef buffer_vesc < handle
    properties (Constant)
    MAXSIZE =1024;
    end
    properties 
    buffer_array;
    index;
    end
    
methods 
    function obj=buffer_vesc(SIZE)
        obj.buffer_array=zeros(1,SIZE,'uint8');
        obj.index=1;
    end
    function buffer_append_int8(obj,number)
        obj.buffer_array(obj.index)=number;
        obj.index=obj.index+1;
    end
    function buffer_append_uint8(obj,number)
        obj.buffer_array(obj.index)=number;
        obj.index=obj.index+1;
    end
    function buffer_append_int16(obj,number)
        number=dec2bin(number);
        while length(number)<16
         number=['0' number];   
        end
        obj.buffer_array(obj.index)=bin2dec(number(end-15:end-8));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-7:end));
        obj.index=obj.index+1;
    end
    
     function buffer_append_uint16(obj,number) 
         number=dec2bin(number);
        while length(number)<16
         number=['0' number];   
        end
        obj.buffer_array(obj.index)=bin2dec(number(end-15:end-8));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-7:end));
        obj.index=obj.index+1;
     end
%     
     function buffer_append_int32(obj,number)
        number=dec2bin(number);
        while length(number)<32
         number=['0' number];   
        end
        obj.buffer_array(obj.index)=bin2dec(number(end-31:end-24));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-23:end-16));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-15:end-8));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-7:end));
        obj.index=obj.index+1;
     end
% 
     function buffer_append_uint32(obj, number)
        number=dec2bin(number);
        while length(number)<32
         number=['0' number];   
        end
        obj.buffer_array(obj.index)=bin2dec(number(end-31:end-24));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-23:end-16));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-15:end-8));
        obj.index=obj.index+1;
        obj.buffer_array(obj.index)=bin2dec(number(end-7:end));
        obj.index=obj.index+1;
      end
 
     function buffer_append_float16(obj,number,scale)
     obj.buffer_append_int16(int16_t((number * scale)));
     end
 
     function buffer_append_float32(obj,number,scale)
     obj.buffer_append_int32(int32_t((number * scale)));
     end
 
    function res=buffer_get_int16(obj) 
        tmp1=dec2bin(obj.buffer_array(obj.index));
        while length(tmp1)<8
            tmp1=['0' tmp1];
        end  
        tmp2=dec2bin(obj.buffer_array(obj.index+1));
        while length(tmp2)<8
            tmp2=['0' tmp2];
        end
        res=bin2dec([tmp1 tmp2]);
        obj.index=obj.index+2;
    end
     function res=buffer_get_uint16(obj) 
         tmp1=dec2bin(obj.buffer_array(obj.index));
        while length(tmp1)<8
            tmp1=['0' tmp1];
        end  
        tmp2=dec2bin(obj.buffer_array(obj.index+1));
        while length(tmp2)<8
            tmp2=['0' tmp2];
        end
        res=bin2dec([tmp1 tmp2]);
        obj.index=obj.index+2;
     end

     function  res=buffer_get_int32(obj)
        tmp1=dec2bin(obj.buffer_array(obj.index));
        while length(tmp1)<8
            tmp1=['0' tmp1];
        end  
        tmp2=dec2bin(obj.buffer_array(obj.index+1));
        while length(tmp2)<8
            tmp2=['0' tmp2];
        end
        tmp3=dec2bin(obj.buffer_array(obj.index+2));
        while length(tmp3)<8
            tmp3=['0' tmp3];
        end
        tmp4=dec2bin(obj.buffer_array(obj.index+3));
        while length(tmp4)<8
            tmp4=['0' tmp4];
        end
        res=bin2dec([tmp1 tmp2 tmp3 tmp4]);
        obj.index=obj.index+4;
     end
     function res=buffer_get_uint8(obj)
        res=obj.buffer_array(obj.index);
        obj.index=obj.index+1;
     end
     function res=buffer_get_int8(obj)
        res=obj.buffer_array(obj.index);
        obj.index=obj.index+1;
    end
     function  res=buffer_get_uint32(obj)
        tmp1=dec2bin(obj.buffer_array(obj.index));
        while length(tmp1)<8
            tmp1=['0' tmp1];
        end  
        tmp2=dec2bin(obj.buffer_array(obj.index+1));
        while length(tmp2)<8
            tmp2=['0' tmp2];
        end
        tmp3=dec2bin(obj.buffer_array(obj.index+2));
        while length(tmp3)<8
            tmp3=['0' tmp3];
        end
        tmp4=dec2bin(obj.buffer_array(obj.index+3));
        while length(tmp4)<8
            tmp4=['0' tmp4];
        end
        res=bin2dec([tmp1 tmp2 tmp3 tmp4]);
        obj.index=obj.index+4;
     end
 
     function  res=buffer_get_float16(obj,scale)
        res =obj.buffer_get_int16() / scale;
     end
 
     function res=buffer_get_float32(obj,scale)
     res=obj.buffer_get_int32()/scale;
     end
 
     function res=buffer_get_bool(obj)
 	
        if obj.buffer_array(obj.index) == 1
 		obj.index=obj.index+1;
 			res=true;
        else
 		obj.index=obj.index+1;
 			res=false;
        end
     end

       function  buffer_append_bool(obj, value) 

            if (value == true)
                 obj.buffer_array(obj.index) = 1;
                 obj.index=obj.index+1;
            else
                 obj.buffer_array(obj.index) = 0;
                 obj.index=obj.index+1;
             end
       end
       
       function buffer_append_checksum(obj)
            crc=uint8(mod(sum(obj.buffer_array(6:end)),256));
            obj.buffer_append_uint8(crc);
       end
       function res=buffer_check_checksum(obj)
           crc=uint8(mod(sum(obj.buffer_array(1:end-1)),256));
           res=(crc==obj.buffer_array(end));
       end
       function buffer_append_start(obj)
           obj.buffer_array(obj.index)=uint8(83);
           obj.index=obj.index+1;
           obj.buffer_array(obj.index)=uint8(84);
           obj.index=obj.index+1;
           obj.buffer_array(obj.index)=uint8(65);
           obj.index=obj.index+1;
           obj.buffer_array(obj.index)=uint8(82);
           obj.index=obj.index+1;
           obj.buffer_array(obj.index)=uint8(84);
           obj.index=obj.index+1;
       end
    end
end
