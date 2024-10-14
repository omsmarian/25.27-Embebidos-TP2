from gui import GUI
import gc
import serial
import psutil


#Bus 001 Device 110: ID 1357:0089 P&E Microcomputer Systems OpenSDA - CDC Serial Port
ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
period_update_cpu = 50
counter_update_cpu = 0

def classifyData(data, matrix):
    station = data[0]

    if(station < '0' or station > '6'):
        return -1
    
    station = int(station)
    
    angle = data[1]
    if(angle == 'O'):
        angle = 2
    elif(angle == 'H'):
        angle = 1
    elif(angle == 'R'):
        angle = 0
    else:
        return -1

    sign = data[2]
    if(sign != '+' and sign != '-'):
        return -1
    
    value1 = data[3]
    value2 = data[4]
    value3 = data[5]

    if(value1 < '0' or value1 > '9' or value2 < '0' or value2 > '9' or value3 < '0' or value3 > '9'):
        return -1

    value = int(value1)*100 + int(value2)*10 + int(value3)

    if(value < 0 or value > 360):
        return -1

    if(sign == '-'):
        matrix[station][angle] = -value
    else:
        matrix[station][angle] = value


class main_:
    ''' Class that launches everything '''

    def __init__(self, *args, **kwargs):
        self._initialise_gui(*args, **kwargs)
        self._update_display()
        self._gui.mainloop()
        classifyData("000000", self._gui.data_matrix)

    def _initialise_gui(self, *args, **kwargs):
        self._gui = GUI(*args, **kwargs)

    def _update_display(self):
        value= ser.readline()
        valueInString=str(value,'UTF-8')
        if(classifyData(valueInString, self._gui.data_matrix)):
            print("Invalid data received")
        cpu_usage = psutil.cpu_percent()
        mem_usage = psutil.virtual_memory().percent
        self._gui.render()
        self._gui.update_rotation_values()
        global counter_update_cpu
        if counter_update_cpu >= period_update_cpu:
            counter_update_cpu = 0
            self._gui.update_cpu_usage(cpu_usage, mem_usage)
        else:
            counter_update_cpu += 1
        self._gui.after(1, self._update_display)

if __name__ == '__main__':
    main_()

gc.collect()