# import thorpy
# import usb
# from serial.tools.list_ports import comports
# from thorpy.comm.port import *
# from thorpy.comm.port import Port
# from thorpy.comm.discovery import *
import time


# def discover_stages2():
#     import usb
#     import os
#     #from .port import Port
#     from serial.tools.list_ports import comports
#     import platform
    
#     serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
#     print(serial_ports)
#     for dev in usb.core.find(find_all=True, custom_match= lambda x: x.bDeviceClass != 9):
        
#         print(dev)  
#         try:
#             #FIXME: this avoids an error related to https://github.com/walac/pyusb/issues/139
#             #FIXME: this could maybe be solved in a better way?
#             dev._langids = (1033, )
#             # KDC101 3-port is recognized as FTDI in newer kernels
#             print(dev.manufacturer)
            
#             if not (dev.manufacturer == 'Thorlabs' or dev.manufacturer == 'FTDI'):
#                 continue
#         except usb.core.USBError:
#             print("except usb core USBerror")
#             print(platform.system())
#             continue
        
        
#         if platform.system() == 'Linux':
#             port_candidates = [x[0] for x in serial_ports if x[2].get('SER', None) == dev.serial_number]
#             print(port_candidates)
#         else:
#             raise NotImplementedError("Implement for platform.system()=={0}".format(platform.system()))
        
#         assert len(port_candidates) == 1
        
#         port = port_candidates[0]
#         print("port :", port, dev.serial_number)

#         p = Port.create(port, dev.serial_number)
# #   p = Port.create('/dev/ttyUSB0','40878582')
#         print("test2thorlabs : port cree ! recuperation des stages")
#         for stage in p.get_stages().values():
#             yield stage


from thorpy.comm.discovery import discover_stages

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print("stages : " ,stages)
    s = stages[0]
    print(s)
    print(" ")
    
    # s.home()
    # time.sleep(5)
    print("JOG1")
    s.move_jog(direction=1)
    time.sleep(4)
    print("JOG2")
    s.move_jog(direction=2)
    time.sleep(4)
    print("position ")
    s.position =-8
    time.sleep(4)
    # print("JOG4")
    # s.move_jog(direction=1)
    # time.sleep(4)

    print("end")
    
    # s.print_config()
    
    # s.print_state()

# if __name__ == '__main__':
#   print(list(discover_stages()))


#comports()
#serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
#print(serial_ports)

#p=Port('/dev/ttyUSB0',40878582)
#discover_stages()
#SingleControllerPort('/dev/ttyUSB0',40878582)
#x=Port.create('/dev/ttyUSB0','40878582')




