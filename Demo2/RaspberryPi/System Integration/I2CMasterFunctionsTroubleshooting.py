import struct
import ctypes


b=0

def writeNumber(value):
    
    
    if value == None:
        return 'HAHA'
    elif (isinstance(value, float) or (value > 128 or value < -129):
        #print("Float")
        try:
            data=[]
            data = [0 for i in range(4)]
            #For the I2C we communicate the dataType and the read based on the address offset, 
            if(isinstance(value, float)):
                #These next two lines take the bits of the float and interpret them as an int
                s = ctypes.c_float(value) #print(s)
                b = ctypes.c_int.from_address(ctypes.addressof(s)).value#print(b)

                #We can then bitslice the int to isolate the 4 bytes that make up the float and store them in an array of bytes
                data[0] = (b & 0x000000FF) 
                data[1] = (b & 0x0000FF00) >> 8
                data[2] = (b & 0x00FF0000) >> 16
                data[3] = (b & 0xFF000000) >> 24

                #Finally, we can send the byte array over I2C
                bus.write_i2c_block_data(address, 1, output)
            else:
                #If the value passed through the function is already a long, then no python data type conversion is needed and we can just bitslice
          
                data[0] = (value & 0x000000FF) 
                data[1] = (value & 0x0000FF00) >> 8
                data[2] = (value & 0x00FF0000) >> 16
                data[3] = (value & 0xFF000000) >> 24
          
                bus.write_i2c_block_data(address, 2, output)
            #print(*data)
          
            return 2
        except:
            return -2
    elif (value > -129 and value < 128):
        try:
            bus.write_byte_data(address, 0, value)
            return -1
        except:
            return -1
    
    return 0

print(writeNumber(None))
print(writeNumber(3))
print(writeNumber(3.14))


