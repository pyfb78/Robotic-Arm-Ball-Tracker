import serial
import serial.tools.list_ports

def printCOMPorts():
    for comport in serial.tools.list_ports.comports():
        # vid = comport.vid
        # pid = comport.pid
        # sn  = comport.serial_number
        # print("COM Port:", vid, pid, sn)
        print(comport)
    return


printCOMPorts()
