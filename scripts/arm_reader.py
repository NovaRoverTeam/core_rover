import can

can_interface = 'can1'
bus = can.interface.Bus(can_interface, bustype = 'socketcan_native')
while(True):
    message = bus.recv()
    if len(message.data)==4 and message.arbitration_id==1:
        data1=message.data[0]
        data2=message.data[1]
        data3=message.data[2]
        data4=message.data[3]
        number = int(data1<<24)+int(data2<<16)+int(data3<<8)+int(data4)
        print(number)
