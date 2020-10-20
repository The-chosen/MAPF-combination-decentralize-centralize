import socket
import json
import time
import sys
from pynput import keyboard

class PiRobot:
    def __init__(self,rid,ip):
        self.ip = ip
        self.rid = rid
        # self.ip = '192.168.3.2'
        self.in_port = 4999 #into the robot
        self.out_port = 4999 #outof the robot

    def send_message(self,data):
        data_str = json.dumps(data)
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(bytes(data_str,'utf-8'),(self.ip,self.in_port))

    def receive_message(self):
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.bind(('',self.out_port))
        msg,addr = sk.recvfrom(1024)
        msg = msg.decode('utf-8')
        msg_js = json.loads(msg)
        print(msg_js)

    def set_current_pose(self):
        msg = {}
        msg['msgType'] = 1
        msg['x'] = 1
        msg['y'] = 1
        msg['theta'] = 1
        self.send_message(msg)

    def set_velocities(self):
        msg = {}
        msg['msgType'] = 3
        msg['v'] = 0.1
        msg['w'] = 0
        self.send_message(msg)

    def set_velocities_data(self,v,w):
        msg = {}
        msg['msgType'] = 3
        msg['v'] = v
        msg['w'] = w
        self.send_message(msg)
        

    def set_rps(self):
        msg = {}
        msg['msgType'] = 5
        msg['rpsL'] = 3
        msg['rpsR'] = 3
        self.send_message(msg)

    def set_wifi_and_password(self):
        msg = {}
        msg['ap'] = 'robotarium_setup_ap'
        msg['pass'] = ''
        self.send_message(msg)

    def set_rainbow(self):
        msg = {}
        msg['msgType'] = 19
        msg['duration'] = 10
        self.send_message(msg)

class PiOSFacility:
    def __init__(self):
        self.in_port = 4998
        self.out_port = 4999
        self.ip_prefix = '192.168.3.'
        mac_ip = self.get_all_agents_ip()
        self.agents = {}
        for mac,ip in mac_ip:
            self.agents[mac] = PiRobot(mac,ip)
            
    def get_all_agents_ip(self):
        agents = []
        data_str = json.dumps({'msgType':200})
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.settimeout(1.0)
        for i in range(1,255): 
            ip = self.ip_prefix+str(i)
            print('testing ip: '+ip)
            sk.sendto(bytes(data_str,'utf-8'),(ip,self.in_port))
            msg,addr = sk.recvfrom(1024)
            msg_js = json.loads(msg.decode('utf-8'))
            agents.append((msg_js['macAddress'],msg_js['ipAddress']))
        return agents


    
if __name__ == '__main__':
    # print(sys.argv)
    rb = PiRobot('2','192.168.159.11')
    def on_press(key):
        if key==keyboard.Key.esc:
            return False
        try:
            k = key.char
        except:
            k = key.name
        if k=='i':
            print('forward')
            for i in range(2):
                rb.set_velocities_data(0.08,0.0)
                time.sleep(0.1)
        if k=='k':
            print('stop')
            rb.set_velocities_data(0.0,0)
   
        if k=='l':
            print('left')
            for i in range(2):
                rb.set_velocities_data(0,0.3)
                time.sleep(0.1)
        if k=='j':
            print('right')
            for i in range(2):
                rb.set_velocities_data(0,-0.3)
                time.sleep(0.1)

    if sys.argv[1] == 'client':
        count = 0
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        listener.join()
        while False:
            count += 1
            print('this is iteration %d'%(count))
            rb.set_velocities()
            rb.set_rainbow()
            time.sleep(0.1)
    # elif sys.argv[1] == 'server':
    #     while True:
    #         rb.receive_message()
    #         time.sleep(5)
