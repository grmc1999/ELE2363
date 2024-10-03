#!/usr/bin/env python

import io
import socket
import select
import struct
import time
import os
import gopigo3
import line_sensor

def run():
    GPG = gopigo3.GoPiGo3()
    
    last_command = time.clock()
    last_battery_good = time.clock()

    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 8002))
    server_socket.listen(0)

    while True:
        print "Data server awaiting connection"
        
        GPG.reset_all()
        connection = server_socket.accept()[0]
        
        print "Data server connected"
        
        while True:
            # Battery check
            battery = GPG.get_voltage_battery()
            if battery < 10:
                print "Low battery: ", battery
                if time.clock() > last_battery_good + 1:
                    print "Battery critically low, shutting down"
                    os.system("shutdown now -h")
            else:
                last_battery_good = time.clock()
            
            ready = select.select([connection], [], [], 0.010)[0] # 10ms timeout
            
            if ready:
                sz = struct.unpack('<L', connection.recv(struct.calcsize('<L')))[0]
                if not sz:
                    break
                    
                msg = ''
                while len(msg) < sz:
                    msg = connection.recv(sz-len(msg))
                msg = struct.unpack('<lll', msg)
                
                # Apply commands
                if battery > 10.5:
                    GPG.set_motor_dps(GPG.MOTOR_LEFT, msg[0])
                    GPG.set_motor_dps(GPG.MOTOR_RIGHT, msg[1])
                    GPG.set_servo(GPG.SERVO_1, msg[2])
                    
                    last_command = time.clock()
                
            # Auto-stop if no commands are sent
            if time.clock() > last_command + 1:
                print "Stopping (no data or low battery). Battery is at ", battery, "V"
                last_command = time.clock() + 3600
                GPG.reset_all()
                
            # Construct status
            msg = struct.pack('<ll', GPG.get_motor_encoder(GPG.MOTOR_LEFT)*GPG.MOTOR_TICKS_PER_DEGREE, GPG.get_motor_encoder(GPG.MOTOR_RIGHT)*GPG.MOTOR_TICKS_PER_DEGREE)
            msg = msg + struct.pack('<lllll', *line_sensor.get_sensorval())
            msg = msg + struct.pack('<f', battery)
            msg = struct.pack('<L', len(msg)) + msg
            
            connection.send(msg)

    server_socket.close()

if __name__ == '__main__':
    run()
