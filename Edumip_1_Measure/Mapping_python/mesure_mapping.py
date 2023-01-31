import time, math
import getopt, sys
import rcpy 
import rcpy.servo as servo
from rcpy.servo import servo1
import rcpy.clock as clock
import Adafruit_BBIO.GPIO as GPIO
import time
import socket
import struct

#socket
IP_ADDRESS = "127.0.0.1"
PORT  = 5005
BUFFER_SIZE  = 1024
saddr = (IP_ADDRESS, PORT)

try:
    clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except clientSock.error as err :
    print("ERROR: Socket creation failed with error %s" %(err))

try:
    clientSock.connect(saddr)
    print("Connected to %s\n" % repr(saddr))
except:
    print("ERROR: Connection to %s refused" % repr(saddr))
    sys.exit(1)

# HC-SR04 connection
# red wire
vcc = "5V"

# white wire
trigger = "GPIO1_25"

# blue wire using resistor
echo = "P9_23" 

# black wire
gnd = "GND"
period = 0.02
rcpy.set_state(rcpy.RUNNING)
srvo = servo.Servo(1)
clck = clock.Clock(srvo, period)


GPIO.cleanup()
time.sleep(2)


def distance_measurement(TRIG,ECHO):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    pulseStart = time.time()
    pulseEnd = time.time()
    counter = 0
    while GPIO.input(ECHO) == 0:
        pulseStart = time.time()
        counter += 1
        if counter>100:
            break
    while GPIO.input(ECHO) == 1:
        pulseEnd = time.time()
    pulseDuration = pulseEnd - pulseStart
    distance = pulseDuration * 17150
    distance = round(distance, 2)
    return distance


# Configuration
print("trigger: [{}]".format(trigger))
GPIO.setup(trigger, GPIO.OUT) #Trigger
print("echo: [{}]".format(echo))
GPIO.setup(echo, GPIO.IN)  #Echo
GPIO.output(trigger, False)
print("Setup completed!")

# Security
GPIO.output(trigger, False)
time.sleep(0.5)
n=0
a=0
b=0
Tableau =[]
Mesure=[]
Dest_angle=[]
Elem_de_ref=0
Elem_a_comp=1
Stock_pour_ech=0
distance = distance_measurement(trigger, echo)
servo.enable()
clck.start()
while(True):
    
    while n<17:
        duty = ((-90)+n*10)/90*1.5
        print('duty : {}'.format(duty))
        
        # increment duty
        
        srvo.set(duty)

        # sleep some
        time.sleep(1)
        Tableau = []
        while a<5:
            distance = distance_measurement(trigger, echo)
            Tableau.append(distance)
            a=a+1
            time.sleep(0.2)
        a=0
        Tableau_trie = sorted(Tableau)
        print('tableau : {}'.format(Tableau_trie))
        dist_med = Tableau_trie[2]
        print('dist med : {}'.format(dist_med))
        f=open('mapping.txt','a')
        f.write('\n')
        f.write(str(dist_med))
        f.close()
        Mesure.append(dist_med)
        n=n+1
    Destination= max(Mesure)
    if Destination > 400 :
        dmax = Mesure.index(Destination)
        Destination = max(Mesure[0 : dmax -1],Mesure[dmax+1 : 16])
    Angle = Mesure.index(Destination)
    print('{}'.format(Angle))
    Destination = Destination*1/2
    Angle = ((-90)+Angle*10)/360*4*math.pi
    print('{}'.format(Angle))
    Destination = Destination/3.4
    msg = str(Angle)
    buff = msg.encode()
    ssent = clientSock.sendto(buff, saddr)
    print('sent %s bytes to %s\n' % (ssent, saddr))
    buff, saddr = clientSock.recvfrom(BUFFER_SIZE) # buffer size is 1024 bytes
    msg = buff.decode('utf-8')
    print ("Received message: " + msg)
    print('received %s bytes from %s\n' % (len(buff), saddr))
    msg = str(Destination)
    buff = msg.encode()
    ssent = clientSock.sendto(buff, saddr)
    print('sent %s bytes to %s\n' % (ssent, saddr))
    buff, saddr = clientSock.recvfrom(BUFFER_SIZE) # buffer size is 1024 bytes
    msg = buff.decode('utf-8')
    print ("Received message: " + msg)
    print('received %s bytes from %s\n' % (len(buff), saddr))
    f=open('angle.txt','a')
    f.write('\n')
    f.write(str(Angle))
    f.write('\n')
    f.write(str(msg))
    f.close()
    Mesure=[]
    b=b+1
    n=n-1
    servo.enable()
    while n>0:
        duty = ((-90)+n*10)/90*1.5
        srvo.set(duty)
        time.sleep(0.3)
        n=n-1
    n=0
    
        
    

GPIO.cleanup()
print("Done")