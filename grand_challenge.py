#Import Modules
import email
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as gpio
import serial
import math
from math import pi
import time
import cv2
import numpy as np

from datetime import datetime
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

def init():
    
    #motors
    gpio.setup(31, gpio.OUT)
    gpio.setup(33, gpio.OUT)
    gpio.setup(35, gpio.OUT)
    gpio.setup(37, gpio.OUT)
    
    #encoders
    gpio.setup(7, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down=gpio.PUD_UP)


def gameover():

    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)


def imu_reading():

    while True:

        global count
        global ser
 
        if (ser.in_waiting > 0):

            count += 1 

            line = ser.readline()

            if count > 10:

                line = line.rstrip().lstrip()

                line = str(line)
                line =line.strip("'")
                line = line.strip("b'")
                angle= float(line)

                #clockwise to anticlockwise
                angle = 360-angle
                return angle

def turn(angle):
    current_angle = imu_reading()

    #angle =left/anticlockwise
    if angle > 0:
        #Setting pins
        pin_1 = 33
        pin_2 = 37

        #pwm value(arena)
        val = 60

        #target angle
        target_angle = current_angle + angle

        target_angle = target_angle % 360

    #angle = clockwise
    if angle < 0:
        #Setting pins
        pin_1 = 31
        pin_2 = 35

        #pwm value(arena)
        val = 60

        #target angel
        target_angle = current_angle + angle

        if target_angle < 0:
            target_angle = target_angle + 360

    #initialize
    init()

    pwm1 = gpio.PWM(pin_1, 50) # left wheel
    pwm2 = gpio.PWM(pin_2, 50) # right wheel
    
    pwm1.start(val)
    pwm2.start(val)
    time.sleep(0.1)

    while True:

        current_angle = imu_reading()

        print("target_angle: ", target_angle)
        print("current_angle : ",current_angle )

        error = round((target_angle - current_angle),1)
        print("error :", error)
        print("\n\n")

        if error < 4 and error > -4:

            pwm1.stop()
            pwm2.stop()
            gameover()
            print("goal orientation reached")
            break   

def move(distance):

    # if command == 'forward':
    if distance >=0:
        command = 'forward'

        #Setting pins
        pin_1 = 31
        pin_2 = 37

        #pwm value(arena)
        val = 60

        #No of wheel rev
        wheelrotation = distance/0.2041


    # if command == 'reverse':
    if distance <0:

        command = 'reverse'

        distance = abs(distance)

        #Setting pins
        pin_1 = 33
        pin_2 = 35

        #pwm value
        val = 60

        #No of wheel rev
        wheelrotation = distance/0.2041

    #initialize
    init()
    counterBR = np.int64(0)
    counterFL = np.int64(0)

    buttonBR = int(0) 
    buttonFL = int(0)

    pwm1 = gpio.PWM(pin_1, 50) # left wheel
    pwm2 = gpio.PWM(pin_2, 50) # right wheel
    
    pwm1.start(5)
    pwm2.start(5)
    
    BR_ticks = round(960 * wheelrotation)
    FL_ticks = BR_ticks

    
    # Movement
    while True: 

        left_flag = 0
        right_flag = 0

        #error
        error = counterFL - counterBR  

        if error > 0:
            left_flag = 1

        elif error < 0:
            right_flag = 1
            error = abs(error)

        # Proportional control calculation
        correction = KP * error
        speed = val + correction

        # Ensure speed is within valid range
        speed = max(min(speed, 100), 0)

        # Update motor speed
        if left_flag == 1:
            pwm1.ChangeDutyCycle(val)
            pwm2.ChangeDutyCycle(speed)

        elif right_flag == 1:
            pwm1.ChangeDutyCycle(speed)
            pwm2.ChangeDutyCycle(val)

        elif error == 0:
            pwm1.ChangeDutyCycle(val)
            pwm2.ChangeDutyCycle(val)


        #Stop motors if goal reached
        if counterFL >= FL_ticks:
            pwm1.stop() 

        if counterBR >= BR_ticks:
            pwm2.stop()

        #Increment tick counter
        if int(gpio.input(12) != int(buttonBR)) and counterBR < BR_ticks:
            buttonBR = int(gpio.input(12))
            counterBR += 1
            print (f"BR counter: {counterBR}")

        if int(gpio.input(7) != int(buttonFL)) and counterFL < FL_ticks:
            buttonFL = int(gpio.input(7))
            counterFL += 1
            print (f"FL counter: {counterFL} \n\n")


        #If goal reached -> break
        if counterBR >= BR_ticks and counterFL >= FL_ticks:
            gameover()
            avg_ticks = (counterBR + counterFL)//2
            
            #negative distance
            if command == 'reverse':
                avg_ticks = -avg_ticks

            update_coordinate(avg_ticks)
            break

    return 

def servo_control(val):

    # gpio.setmode(gpio.BOARD)

    # set pin 36 as out
    gpio.setup(36, gpio.OUT)

    # set 50Hz frequency on pin 36
    servo_pwm = gpio.PWM(36, 50)

    #setting the pwm value
    servo_pwm.start(val)
    time.sleep(0.5)

    servo_pwm.stop()


def ultrasonic():
    # Pin Allocations
    trig = 16
    echo = 18

    # Measure distance function
    def measure_distance():
        gpio.setmode(gpio.BOARD)
        gpio.setup(trig, gpio.OUT)
        gpio.setup(echo, gpio.IN)

        gpio.output(trig, False)
        time.sleep(0.01)

        gpio.output(trig, True)
        time.sleep(0.00001)
        gpio.output(trig, False)

        pulse_start = time.time()
        while gpio.input(echo) == 0:
            pulse_start = time.time()

        pulse_end = time.time()
        while gpio.input(echo) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)

        return distance

    distance_values = []
    for _ in range(10):
        distance_values.append(measure_distance())
        time.sleep(0.1)

    average_distance = sum(distance_values) / len(distance_values)
    
    #account for offset
    average_distance += 3
    return round(average_distance,1)

def loc_start():

    #declaring robot coordinates as global
    global x
    global y
    global coordinate_list

    #x-coordinate (start heading) in cm
    x = 304.8 - ultrasonic()

    # turn(-90)
    pose(-90)

    time.sleep(0.2)
    
    #y-coordinate in cm
    y = ultrasonic()

    # turn(-180)
    pose(90)

    coordinate_list.append((x,y))

    start_x = x
    start_y = y

    return start_x,start_y

def polar_cor(x_cor,y_cor):
    global x
    global y

    r = math.sqrt((x_cor - x)**2 + (y_cor - y)**2)
    #cm to meters
    r = r/100

    theta = imu_reading()

    angle = math.atan2((y_cor-y), (x_cor-x))
    if angle < 0:
        angle = angle + 2 * math.pi
        
    angle = math.degrees(angle)
    angle =angle-theta

    return r,angle

def update_coordinate(ticks):
    global x
    global y
    global coordinate_list

    #cm
    distance_traveled = dist_per_tick*ticks*100

    theta = imu_reading()
    theta = math.radians(theta)

    x = round((x + distance_traveled*math.cos(theta)),2)
    y = round((y + distance_traveled*math.sin(theta)),2)

    print("x: ",x)
    print("y: ",y) 

    coordinate_list.append((x,y))

    return

def re_loc():

    global x
    global y
    global coordinate_list

    #search position
    position(80,80)

    #facing the wall for x coordinate
    pose(180)
    x = ultrasonic()

    #facing the wall for y coordinate
    pose(270)
    y = ultrasonic() 

    coordinate_list.append((x,y))
    
def position(x_cor,y_cor):

    #get polar coordinates
    r,angle = polar_cor(x_cor,y_cor)

    # Move the bot to the giveb location
    turn(angle)
    time.sleep(0.2)
    move(r)

def pose(angle):

    angle %= 360

    #get current pose/orientation of the bot
    current_angle = imu_reading()

    diff = angle - current_angle

    # Ensure the difference is within the range [-180, 180)
    if diff >= 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    #turn to the required pose/orientation
    turn(diff)

def capture(color):
    #Pi Camera Setup
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 25
    rawCapture = PiRGBArray(camera, size=(640, 480)) 

    time.sleep(0.1)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output_video.avi', fourcc, 10, (640,480))
    
    if color == 'green':

        #Green limits to be detected(home night)
        lower_limit = np.array([30,79,75])
        upper_limit = np.array([100,255,255])

    
    elif color =='red':

        #Red limits to be detected(arena)
        lower_limit = np.array([170,79,75])
        upper_limit = np.array([255,255,255])

    elif color == 'blue':

        #Blue limits to be detected(arena)
        lower_limit = np.array([57,79,75])
        upper_limit = np.array([122,255,255])

    #searching for the block
    search_counter = 0
    distance_counter = 0 
    goal_flag = 0 

    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True): 
        
        image = frame.array

        # Set the font and position to display text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_position = (50, 50)
        font_scale = 0.7
        font_color = (240, 32, 160)
        font_thickness = 2

        #camera parameters
        known_width = 4.0
        focal_length = 906.66

        # Make the top half of the frame black
        input_img = image.copy()
        input_img[:480//2, :] = [0,0,0]

        #masking
        hsv_image = cv2.cvtColor(input_img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image,lower_limit,upper_limit)

        #reducing the noise
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,kernel, iterations=1)

        #Capturing Contour
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        #choose the largest contour(closest object)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        # print(f"countours : {len(contours)}")

        # Tracking if block found or not
        flag =0 

        if len(contours)!=0:
            for contour in contours:
                if cv2.contourArea(contour)>200:

                    flag = 1 
                    search_counter = 0
                    # print(f"flag: {flag}")

                    #Drawing Circle Contour 
                    (x_axis,y_axis),radius = cv2.minEnclosingCircle(contour) 
                    
                    center = (int(x_axis),int(y_axis)) 
                    radius = int(radius)

                    orientatin_error = 290 - center[0]

                    p_width = radius*2
                    distance = round(((known_width * focal_length) / p_width),1)
                    distance_error = distance-20

                    angle_flag = 0
                    distance_flag = 0
                    goal_flag = 0

                    #orient the bot
                    if orientatin_error<20 and orientatin_error>-20:
                        angle_flag=1
                    else:
                        angle = round((orientatin_error * 0.061),1)
                        turn(angle)
                    
                    time.sleep(0.2)
                    
                    # Grab the block
                    if distance_error//8 >2:
                        # move(0.2)
                        move((((distance_error//8) -2)*8)/100)

                    elif distance_error//8 >1 and distance_error//8 <=2:
                        #20cm
                        move(0.2)

                    else:
                        if distance_error>8:
                            #4cm
                            move(0.04)
                        elif distance_error<=8 and distance_error>0:
                            #open the servo
                            servo_control(7.5)
                            #2cm
                            move(0.02)
                        elif distance_error <= 0 and distance_error>-3 and distance_counter <= 8:
                            #open the servo
                            servo_control(7.5)
                            #0.5cm
                            move(0.005)

                            distance_counter += 1

                        elif distance_error<=-3 or distance_counter > 8:
                            distance_flag =1
                            distance_counter = 0

                    if distance_flag == 1 and angle_flag == 1:
                        goal_flag = 1

                        #close servo
                        servo_control(3)
                        servo_control(3)
                        time.sleep(0.2)
                                  
                    cv2.circle(image,center,radius,(0, 255, 255),3)
                    cv2.circle(image,center,3,(0, 0, 255),-1)
                    
                    # Draw text on the image
                    text2 = f"Block Color: {color}   camera_distance:{distance}cm"                
                    cv2.putText(image, text2, text_position, font, font_scale, font_color, font_thickness)

                    if goal_flag == 1:
                        break

        # if the block not found, move to a better location
        if flag == 0:
            global x
            global y
            
            #Add buffer
            search_counter += 1
            
            #Add limit on y coordinate
            # y<274
            if y < 245 :
                #if in zone 1
                if x >154:
                    if search_counter ==10 or search_counter == 20 or search_counter ==30 :
                        turn(-45)
                        time.sleep(0.2)
                        
                    elif search_counter == 40:
                        pose(90)
                        time.sleep(0.1)
                        move(0.2)
                        pose(150)
                        search_counter =0

                #if in zone2
                else:
                    if search_counter ==10 or search_counter == 20 or search_counter==30 :
                        turn(45)
                        time.sleep(0.2)
                        
                    elif search_counter == 40:
                        pose(90)
                        time.sleep(0.1)
                        move(0.2)
                        pose(30)
                        search_counter =0
            
            else :
                if search_counter% 10 == 0:
                    turn(45)

        #Display frames
        # cv2.imshow("Frame", image)
        # cv2.imshow("Frame2", input_img)
        #Writing a video
        # out.write(image)

        key = cv2.waitKey(1) & 0xFF 
        rawCapture.truncate(0)
        if goal_flag == 1:
            cv2.imwrite('grand_challenge.jpg',image)
            img_email()
            camera.close()
            camera = None
            break

    cv2.destroyAllWindows()  

def img_email():
    
    # Define time stamp & record an image
    pic_time= datetime.now().strftime('%Y%m%d%H%M%S')

    # Email information
    smtpUser = 'sharmaabhey100@gmail.com'
    smtpPass = 'mazoiizwynyilzep'

    # Destination email information
    toAdd =  ['ENPM809TS19@gmail.com','jsuriya@umd.edu','sharmaabhey100@gmail.com']
    # toAdd =  'sharmaabhey100@gmail.com'
    fromAdd = smtpUser
    subject = 'ENPM701-Grand_Challenge-image_captured-' +  pic_time + '-Abhey_Sharma'
    msg =  MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    # msg['To'] =  toAdd
    msg['To'] = ",".join(toAdd)
    msg.preamble = "Image recorded at" + pic_time

    # Email text
    body = MIMEText("Image recorded at " + pic_time)
    msg.attach(body)

    # Attach image
    fp = open ('grand_challenge.jpg','rb')
    img = MIMEImage (fp.read())
    fp.close()
    msg.attach(img)

    # Send email
    s = smtplib.SMTP('smtp.gmail.com', 587)
    s.ehlo() 
    s.starttls()
    s.ehlo()

    s.login (smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()

    print("Email delivered!")


if __name__ == "__main__":

    # Identify serial connection
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    count = 0

    # gain value
    KP =2

    #Initializing variables
    x = 0
    y = 0
    theta = 0
    dist_per_tick = (2*pi*(0.0325))/960
    coordinate_list =[]

    #Setup GPIO pins
    gpio.setmode(gpio.BOARD)

    # localize itself
    start_x,start_y = loc_start()

    #move to search location - 1
    position(182,122)
    pose(150)

    loop_counter = 0
    for _ in range(3):
        for color in ['red','green','blue']:

            #incrementing counter
            loop_counter += 1

            #Tracking the object and grabing the block 
            capture(color)
            
            #Relocalization
            re_loc()

            #go to landing zone
            position(50,50)
            
            #Droping the block
            servo_control(7.5)

            #Reverse
            move(-0.2)
            
            #Close Servo
            servo_control(3)

            if loop_counter <=8:
                #search location- 2
                position(122,122)
                pose(30)
            
            #if all blocks picked up
            else:
                #move to start location
                position(start_x,start_y)
                pose(0)


    #write the data in text file
    # print(coordinate_list)
    with open("grand_challenge.txt", "w") as file:

        for coordinate in coordinate_list:
            file.write(f"{coordinate[0]} {coordinate[1]}\n")

    print("code end")

    #Cleanup gpio pins
    gpio.cleanup()