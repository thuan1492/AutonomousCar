from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
import time
import numpy as np
import socket
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import os
import eventlet
from eventlet import tpool

#eventlet.monkey_patch()
AUTO_STATE = 0
manual_state = 0
mode = 0
front_s =0
right_s = 0

blue_percentage = 0
wall_percentage = 0
obstacle_percentage = 0
gate_percentage = 0

#path = 'ImageQuery'
#orb = cv2.ORB_create()

latest_video_frame = []
#images = []
#classNames= []
#myList = os.listdir(path)

cycles_without_web_contact = 0

# dictionary for converting serial incoming data regarding the current manual state:
manual_states = {
    0: "Stop",
    1: "Forward",
    2: "Backward",
    3: "Right",
    4: "Left",
    5: "ShiftRight",
    6: "ShiftLeft",
    7: "UpperRight",
    8: "UpperLeft",
    9: "LowerRight",
    10:"LowerLeft",
    13:"LiftUp",
    14:"LiftDown",
    15:"Drop",
    16:"Hold",
}

# dictionary for converting serial incoming data regarding the current auto state:
auto_states = {
    1: "STOP",
    2: "FOLLOWING_WALL",
    3: "OUT",
    4: "OUT_OF_WALL",
    5: "DETERMINE_OBSTACLE",
    6: "TURN_LEFT",
    7: "TURN_LEFT",
    8: "JUNCTION_B_R",
    9: "TURN_LEFT",
    10: "TURN_LEFT",
    11: "STOP",
    12: "DETERMINE_IF_SIGN",
    13: "JUNCTION_C_GO_RIGHT",
    14: "END_OF_COURSE",
    15: "OUT_OF_DEAD_END"
}

# dictionary for converting serial incoming data regarding the current mode:
mode_states = {
    11: "Manual",
    12: "Auto"
}

# initialize serial communication with the arduino: (9600 is the baudrate, should match serial baudrate in arduino)
serial_port = serial.Serial("/dev/ttyACM0", 115200) 

# initialize the web server:
app = Flask(__name__)
socketio = SocketIO(app, async_mode = "threading") # without "async_mode = "threading", sending stuff to the cliend (via socketio) doesn't work!

#### Import Images

#for cl in myList:
  #  imgCur = cv2.imread(f'{path}/{cl}',0)
   # images.append(imgCur)
   # classNames.append(os.path.splitext(cl)[0])
    
#def findDes(images):
 #   desList=[]
 #   for img in images:
 #       kp,des = orb.detectAndCompute(img,None)
 #       desList.append(des)
 #   return desList
 
#def findID(img, desList,thres=15):
#    kp2,des2 = orb.detectAndCompute(img,None)
#    bf = cv2.BFMatcher()
 #   matchList=[]
 #   finalVal = -1
 #   try:
 #       for des in desList:
 #           matches = bf.knnMatch(des, des2, k=2)
 #           good = []
  #          for m, n in matches:
  #              if m.distance < 0.75 * n.distance:
  #                  good.append([m])
   #         matchList.append(len(good))
   # except:
     #   pass
    #if len(matchList)!=0:
    #    if max(matchList) > thres:
     #       finalVal = matchList.index(max(matchList))
    #return finalVal

def video_thread():
    # enable the thread to modify the global variable 'latest_video_frame': (this variable will be accessed by functions doing some sort of video analysis or video streaming)
    global latest_video_frame   
    
    # create an instance of the RPI camera class:
    camera = PiCamera() 
    
    # rotate the camera view 180 deg (I have the RPI camera mounted upside down):
    camera.hflip = True
    camera.vflip = True 
    
    # set resolution and frame rate:
    camera.resolution = (640, 480)
    camera.framerate = 30
    
    # create a generator 'video_frame_generator' which will continuously capture video frames 
    # from the camera and save them one by one in the container 'generator_output': ('video_frame_generator' is an infinte iterator which on every iteration (every time 'next()' is called on it, like eg in a for loop) gets a video frame from the camera and saves it in 'generator_output'))  
    generator_output = PiRGBArray(camera, size=(640, 480))
    video_frame_generator = camera.capture_continuous(generator_output, format="bgr", use_video_port=True)
    
    # allow the camera to warm up:
    time.sleep(0.1)
    
    for item in video_frame_generator:
        # get the numpy array representing the latest captured video frame from the camera
        # and save it globally for everyone to access:
        latest_video_frame = generator_output.array 
        
        # clear the output container so it can store the next frame in the next loop cycle:
        # (please note that this is absolutely necessary!)
        generator_output.truncate(0)        
        
        # delay for 0.033 sec (for ~ 30 Hz loop frequency):
        time.sleep(0.033) 
        
def stop_runaway_robot():
    
   # set mode to manual and manual_state to stop: (and everything else to the maximum number for their data type to mark that they are not to be read)
    start_byte = np.uint8(100)
    manual_state = np.uint8(0)
    mode = np.uint8(11)
    
    checksum = np.uint8(manual_state + mode) 
            
    # # send all data bytes:
    serial_port.write(start_byte.tobytes())
    serial_port.write(manual_state.tobytes())
    serial_port.write(mode.tobytes())
    # send checksum:
    serial_port.write(checksum.tobytes())

def web_thread():
    global cycles_without_web_contact
    
    while 1:
        # send all data for display on the web page:
        socketio.emit("new_data", {"AUTO_STATE": AUTO_STATE, "manual_state": manual_state, "mode": mode,
                                   "Front_Sensor": front_s, "Right_Sensor": right_s,
                                   "blue_percentage": blue_percentage, "wall_percentage": wall_percentage,  "obstacle_percentage": obstacle_percentage,"gate_percentage": gate_percentage})
        #print(front_s)
        cycles_without_web_contact += 1
        if cycles_without_web_contact > 5: # if we havn't had wifi contact for ~ 0.5 sec: stop the robot!
            stop_runaway_robot()
        # delay for 0.1 sec (for ~ 10 Hz loop frequency):
        time.sleep(0.1) 

def read_serial_thread():
    # all global variables this function can modify:
    global AUTO_STATE, manual_state, mode, blue_percentage, wall_percentage, obstacle_percentage,gate_percentage, front_s, right_s
    #print("Read")
    while 1:
        #print("check")
        no_of_bytes_waiting = serial_port.inWaiting()
        if no_of_bytes_waiting >11: # the ardu sends 17 bytes at the time (15 data, 2 control)
            # read the first byte (read 1 byte): (ord: gives the actual value of the byte)
            #print("Sen", front_s)
            first_byte = np.uint8(ord(serial_port.read(size = 1))) 
            
            # read all data bytes if first byte was the start byte:
            if first_byte == 100:
                serial_data = []
                # read all data bytes:
                for counter in range(9): # 17 data bytes is sent from the ardu
                    serial_data.append(ord(serial_port.read(size = 1)))
                
                # read the received checksum:
                checksum = np.uint8(ord(serial_port.read(size = 1)))
                
                # calculate checksum for the received data bytes: (pleae note that the use of uint8 and int8 exactly match what is sent from the arduino)
                calc_checksum = np.uint8(np.uint8(serial_data[0]) + np.uint8(serial_data[1]) + 
                    np.uint8(serial_data[2]) + np.uint8(serial_data[3]) + np.uint8(serial_data[4])
                + np.uint8(serial_data[5])+ np.uint8(serial_data[6])+ np.uint8(serial_data[7]) + np.uint8(serial_data[8]))

                # update the variables with the read serial data only if the checksums match:
                if calc_checksum == checksum:
                    AUTO_STATE = auto_states[int(np.uint8(serial_data[0]))] # look up the received integer in the auto_states dict
                    manual_state = manual_states[int(np.uint8(serial_data[1]))]
                    mode = mode_states[int(np.uint8(serial_data[2]))]
                    front_s = int(np.uint8(serial_data[3]))
                    right_s = int(np.uint8(serial_data[4]))
                    blue_percentage = int(np.uint8(serial_data[5]))
                    wall_percentage = int(np.uint8(serial_data[6]))
                    obstacle_percentage = int(np.uint8(serial_data[7]))
                    gate_percentage = int(np.uint8(serial_data[8]))
                   

                else: # if the checksums doesn't match: something weird has happened during transmission: flush input buffer and start over
                    serial_port.flushInput()
                    print("Something went wrong in the transaction: checksums didn't match!")                      
            else: # if first byte isn't the start byte: we're not in sync: just read the next byte until we get in sync (until we reach the start byte)
                pass
        else: # if not enough bytes for entire transmission, just wait for more data:
            pass

        time.sleep(0.025) # Delay for ~40 Hz loop frequency (faster than the sending frequency)

def get_ip():
    ip_add = ''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8",80))
    ip_add = s.getsockname()[0]
    s.close()
    return ip_add

def gen_normal():
    font = cv2.FONT_ITALIC
    while 1:
        if len(latest_video_frame) > 0: # if we have started receiving actual frames:
            # convert the latest read video frame to jpg format:
            #desList = findDes(images)
            img = latest_video_frame.copy();
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            

            lower_object = np.array([90, 130, 50])
            upper_object = np.array([140, 255, 255])

            mask = cv2.inRange(hsv, lower_object, upper_object)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel)

            # Contours detection
            if int(cv2.__version__[0]) > 2:
                # Opencv 4.x.x
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            else:
                # Opencv 3.x.x
                _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                approx = cv2.approxPolyDP(cnt, 0.0099*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]

                if area > 450:
                    cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)
                    if len(approx) == 6:
                        # print(len(approx))
                        cv2.putText(img, "Cube", (x, y), font, 2, (0, 0, 0))
                    elif 11 < len(approx) < 20:
                        # print(len(approx))
                        cv2.putText(img, "Sphere", (x, y), font, 2, (0, 0, 0))
                    elif 7 < len(approx) <= 11:
                        # print(len(approx))
                        cv2.putText(img, "Cylinder", (x, y), font, 2, (0, 0, 0))
            


            #imgOriginal = latest_video_frame.copy()

            #img2 = cv2.cvtColor(imgOriginal,cv2.COLOR_BGR2GRAY)
         
            #id = findID(imgOriginal,desList)
            #if id != -1:
             #   cv2.putText(imgOriginal,classNames[id],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(255,0,255),2)
            
            ret, jpg = cv2.imencode(".jpg", img) 
            
            # get the raw data bytes of the jpg image: (convert to binary)
            frame = jpg.tobytes()
            
            # yield ('return') the frame: (yield: returns value and saves the current state of the generator function, the next time this generator function is called execution will resume on the next line of code in the function (ie it will in this example start a new cycle of the while loop and yield a new frame))
            # what we yield looks like this, but in binary: (binary data is a must for multipart)
            # --frame
            # Content-Type: image/jpeg
            #
            # <frame data>
            #
            yield (b'--frame\nContent-Type: image/jpeg\n\n' + frame + b'\n')
            
            
def gen_mask():
    
    while 1:
        if len(latest_video_frame) > 0: # if we have started receiving actual frames:
            
            # convert the latest read video frame to HSV (Hue, Saturation, Value) format:
            hsv = cv2.cvtColor(latest_video_frame, cv2.COLOR_BGR2HSV)
            
            lower_gate = np.array([170, 70, 50], dtype=np.uint8) # = [H-20, 100, 100]
            upper_gate = np.array([180, 255, 255], dtype=np.uint8)
            gate_mask = cv2.inRange(hsv, lower_gate, upper_gate)
            ratio_gate = cv2.countNonZero(gate_mask)/(latest_video_frame.size/3)
            # specify lower and upper "blueness" filter boundries:
            lower_blue = np.array([90, 130, 50], dtype=np.uint8) # = [H-20, 100, 100]
            upper_blue = np.array([140, 255, 255], dtype=np.uint8) # = [H+20, 100, 100]
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            ratio_blue = cv2.countNonZero(blue_mask)/(latest_video_frame.size/3)
            #lower_wall = np.array([10, 40, 80])
            #upper_wall = np.array([50, 255, 255])
            lower_wall = np.array([10, 65, 0], dtype=np.uint8)
            upper_wall = np.array([30, 145, 255], dtype=np.uint8)
            wall_mask = cv2.inRange(hsv, lower_wall, upper_wall)
            ratio_wall = cv2.countNonZero(wall_mask)/(latest_video_frame.size/3)

            lower_obstacle = np.array([0, 0, 0], dtype=np.uint8)
            upper_obstacle = np.array([180, 255, 30], dtype=np.uint8)
            ob_mask = cv2.inRange(hsv, lower_obstacle, upper_obstacle)
            
            ob_output = cv2.bitwise_and(latest_video_frame, latest_video_frame, mask=ob_mask+blue_mask+wall_mask+gate_mask)
            ratio_ob = cv2.countNonZero(ob_mask)/(latest_video_frame.size/3)
            # mask the image according to the "blueness" filter: (pixels which are IN the "blueness" range specified above will be made white, pixels which are outside the "blueness" range (which aren't blue enough) will be made black)
            wall_mask = cv2.cvtColor(wall_mask, cv2.COLOR_GRAY2BGR)
            ob_mask = cv2.cvtColor(ob_mask, cv2.COLOR_GRAY2BGR)
            blue_mask = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)
            gate_mask = cv2.cvtColor(gate_mask, cv2.COLOR_GRAY2BGR)
            
            hStack = np.hstack([ob_mask+blue_mask+wall_mask+gate_mask,ob_output])

             
             # np.nonzero(range_mask) is in this case a tuple where the first element is an array containing all row/column indices of the non-zero elements (pixels), and the second is an array containing all column/row indices. Number of non-zero elements (pixels) thus = size of the first element = size of the second element
            wall_percentage = int(np.round(ratio_wall*100, 2))
            blue_percentage = int(np.round(ratio_blue*100, 2))
            obstacle_percentage = int(np.round(ratio_ob*100, 2))
            gate_percentage = int(np.round(ratio_gate*100, 2))

            # # set start byte and set everything else to the maximum number for their data type to mark that they are not to be read:
            start_byte = np.uint8(100)
            manual_state = np.uint8(0xFF)
            mode = np.uint8(0xFF)
            blue_percentage = np.uint8(blue_percentage)
            wall_percentage = np.uint8(wall_percentage)
            obstacle_percentage = np.uint8(obstacle_percentage)
            gate_percentage = np.uint8(gate_percentage)
            # # caculate checksum for the data bytes to be sent:
            checksum = np.uint8(manual_state + mode+ blue_percentage + wall_percentage + obstacle_percentage + gate_percentage)
            
            # # send all data bytes:
            serial_port.write(start_byte.tobytes())
            serial_port.write(manual_state.tobytes())
            serial_port.write(mode.tobytes())
            serial_port.write(blue_percentage.tobytes())
            serial_port.write(wall_percentage.tobytes())
            serial_port.write(obstacle_percentage.tobytes())
            serial_port.write(gate_percentage.tobytes())
            
            # # send checksum:
            serial_port.write(checksum.tobytes())
            
            # convert the result to jpg format:
            ret, jpg = cv2.imencode(".jpg", hStack )
            # get the raw data bytes of the jpg image: (convert to binary)
            frame = jpg.tobytes()
            
            # yield the frame:
            # what we yield looks like this, but in binary: (binary data is a must for multipart)
            # --frame
            # Content-Type: image/jpeg
            #
            # <frame data>
            #
            yield (b'--frame\nContent-Type: image/jpeg\n\n' + frame + b'\n')
            #time.sleep(0.1)
                
@app.route("/camera_normal")
def camera_normal():
    # return a Respone object with a 'gen_normal()' generator function as its data generating iterator. We send a MIME multipart message of subtype Mixed-replace, which means that the browser will read data parts (generated by gen_obj_normal) one by one and immediately replace the previous one and display it. We never close the connection to the client, pretending like we haven't finished sending all the data, and constantly keeps sending new data parts generated by gen_obj_normal.
    # what over time will be sent to the client is the following:
    # Content-Type: multipart/x-mixed-replace; boundary=frame
    #
    # --frame
    # Content-Type: image/jpeg
    #
    #<jpg data>
    #
    # --frame
    # Content-Type: image/jpeg
    #
    #<jpg data>
    #
    # etc, etc
    # where each '--frame' enclosed section represents a jpg image taken from the camera that the browser will read and display one by one, replacing the previous one, thus generating a video stream
    gen_obj_normal = gen_normal()
    return Response(gen_obj_normal, mimetype = "multipart/x-mixed-replace; boundary=frame")
    
@app.route("/camera_mask")
def camera_mask():
    # return a Respone object with a 'gen_mask()' generator function as its data generating iterable, se "camera_normal"
    gen_obj_mask = gen_mask()
    return Response(gen_obj_mask, mimetype = "multipart/x-mixed-replace; boundary=frame")    
   
@app.route("/index")
def index():
    try:
        # start a thread constantly reading frames from the camera:
        thread_video = Thread(target = video_thread)
        thread_video.start()
        
        # start a thread constantly sending sensor/status data to the web page: 
        thread_web = Thread(target = web_thread)
        thread_web.start()
        
        # start a thread constantly reading sensor/status data from the arduino:
        thread_read_serial = Thread(target = read_serial_thread)
        thread_read_serial.start()
        
        return render_template("index.html") 
    except Exception as e:
        return render_template("500.html", error = str(e))
    
@app.route("/")        
@app.route("/phone")
def phone():
    try:
        # start a thread constantly reading frames from the camera:
        thread_video = Thread(target = video_thread)
        thread_video.start()
        #tpool.execute(thread_video)
        # start a thread constantly sending sensor/status data to the web page: 
        thread_web = Thread(target = web_thread)
        thread_web.start()
       # tpool.execute(web_thread, thread.get_ident())
        
        # start a thread constantly reading sensor/status data from the arduino:
        thread_read_serial = Thread(target = read_serial_thread)
        thread_read_serial.start()
        #tpool.execute(thread_read_serial)
        return render_template("phone.html") 
    except Exception as e:
        return render_template("500.html", error = str(e))        
        
@socketio.on("my event")
def handle_my_custom_event(sent_dict):
    print("Recieved message: " + sent_dict["data"])
       
# handle data which is sent from the web page when the user are manually controlling the robot, and send it to the arduino: (the user is either pressing the direction arrows or using WASD)
@socketio.on("arrow_event")
def handle_arrow_event(sent_dict):
    
    print("Recieved message: " + str(sent_dict["data"]))
    
    # get manual state, set start byte and set everything else to the maximum number for their data type to mark that they are not to be read:
    start_byte = np.uint8(100)
    manual_state = np.uint8(sent_dict["data"])
    mode = np.uint8(0xFF)
   
    blue_percentage = np.uint8(0xFF)
    wall_percentage = np.uint8(0xFF)
    obstacle_percentage = np.uint8(0xFF)
    gate_percentage = np.uint8(0xFF)
    # caculate checksum for the data bytes to be sent:
    checksum = np.uint8(manual_state + mode + blue_percentage + wall_percentage + obstacle_percentage + gate_percentage)
    
    # send all data bytes:
    serial_port.write(start_byte.tobytes())
    serial_port.write(manual_state.tobytes())
    serial_port.write(mode.tobytes())

    serial_port.write(blue_percentage.tobytes())
    serial_port.write(wall_percentage.tobytes())
    serial_port.write(obstacle_percentage.tobytes())
    serial_port.write(gate_percentage.tobytes())
    # send checksum:
    serial_port.write(checksum.tobytes())
    
# handle data which is sent from the web page when the user are switching mode (manual or auto), and send i to the arduino:
@socketio.on("mode_event")
def handle_mode_event(sent_dict):
    print("Recieved message: " + str(sent_dict["data"]))
    
    # get mode state, set start byte and set everything else to the maximum number for their data type to mark that they are not to be read:
    start_byte = np.uint8(100)
    manual_state = np.uint8(0xFF)
    mode = np.uint8(sent_dict["data"])
    blue_percentage = np.uint8(0xFF)
    wall_percentage = np.uint8(0xFF)
    obstacle_percentage = np.uint8(0xFF)
    gate_percentage = np.uint8(0xFF)
    
    # caculate checksum for the data bytes to be sent:
    checksum = np.uint8(manual_state + mode + blue_percentage + wall_percentage + obstacle_percentage + gate_percentage)
    
    # send all data bytes:
    serial_port.write(start_byte.tobytes())
    serial_port.write(manual_state.tobytes())
    serial_port.write(mode.tobytes())
    
    serial_port.write(blue_percentage.tobytes())
    serial_port.write(wall_percentage.tobytes())
    serial_port.write(obstacle_percentage.tobytes())
    serial_port.write(gate_percentage.tobytes())
    # send checksum:
    serial_port.write(checksum.tobytes())

# reset the control counter everytime we receive the control message from the web page: (if this counter ever gets to big, we know that we have lost wifi contact)
@socketio.on("control_event")
def handle_control_event(sent_dict):
    global cycles_without_web_contact
    cycles_without_web_contact = 0
        
@app.errorhandler(404)
def page_not_found(e):
    try:
        return render_template("404.html") 
    except Exception as e:
        return render_template("500.html", error = str(e))
  
if __name__ == '__main__':
    socketio.run(app, get_ip(), 8080)
    #app.run(get_ip(), 8080, threaded=True)

