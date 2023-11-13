
# 라즈베리 코드
import serial 
import cv2
import socket
import threading
import serial
import RPi.GPIO as GPIO
import time

# 소켓 열고 영상 전송 시작
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,10000000)
server_ip = "10.10.141.210"
server_port = 6666
cap = cv2.VideoCapture(-1)
cap.set(3, 640)
cap.set(4, 480)

# Bluetooth and GPIO Initialization
bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)
gData = ""
data_lock = threading.Lock()

SWs = [5, 6, 13, 19]  # Switch Pins
LEDs = [26, 16, 20, 21]  # LED Pins
MotorPins = [(18, 22, 27), (23, 25, 24)]  # Motor Pins (PWM, IN1, IN2)
BUZZER = 12  # Buzzer Pin

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering

# Initialize Switch Pins as INPUT
for pin in SWs:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize LED Pins as OUTPUT
for pin in LEDs:
    GPIO.setup(pin, GPIO.OUT)

# Initialize Motor Pins as OUTPUT
for pwm, in1, in2 in MotorPins:
    GPIO.setup(pwm, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)

# Initialize PWM for Motors
L_Motor = GPIO.PWM(MotorPins[0][0], 500)  # Initialize PWM on pwmPinA frequency=500Hz
L_Motor.start(0)  # start it with 0% duty cycle
R_Motor = GPIO.PWM(MotorPins[1][0], 500)  # Initialize PWM on pwmPinB frequency=500Hz
R_Motor.start(0)  # start it with 0% duty cycle

# Initialize Buzzer
GPIO.setup(BUZZER, GPIO.OUT)  # BUZZER pin set as OUTPUT
p = GPIO.PWM(BUZZER, 391)  # Initialize PWM on BUZZER frequency=391Hz
p.stop()  # s

def pi_server_thread():
    pi_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_server_socket.bind(('0.0.0.0', 6667))
    pi_server_socket.listen(1)
    conn, addr = pi_server_socket.accept()
    
    while True:
        data = conn.recv(1024)
        if data == b'armed_person_detected':
            # 부저를 작동시킵니다.
            p.start(50)
            p.ChangeFrequency(391)
            time.sleep(1)
            p.stop()
            
def set_leds(*states):
    for pin, state in zip(LEDs, states):
        GPIO.output(pin, state)

def arduino_serial_thread():
    global gData
    ser = serial.Serial('/dev/ttyACM0', 9600)
    while True:
        try:
          sonic = ser.readline()
          with data_lock:
                gData = sonic.strip()
        except Exception as e:
            print(f"Error in serial_thread: {e}")
      

def motor_control(L_A1, L_A2, R_A1, R_A2, speed):
    GPIO.output(MotorPins[0][1], L_A1)
    GPIO.output(MotorPins[0][2], L_A2)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(MotorPins[1][1], R_A1)
    GPIO.output(MotorPins[1][2], R_A2)
    R_Motor.ChangeDutyCycle(speed)

def serial_thread():
    global gData
    while True:
        try: 
            data = bleSerial.readline().decode()
            with data_lock:
                gData = data.strip()
        except Exception as e:
            print(f"Error in serial_thread: {e}")

def video_thread():
    try:
        while cap.isOpened():
            ret, img = cap.read()
          
            img = cv2.resize(img, (640, 480))
            ret, buffer = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            data = buffer.tobytes()
            
            if data:
                s.sendto(data, (server_ip, server_port))
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
    except Exception as e:
        print(f"Video error: {e}")
    finally:
        cv2.destroyAllWindows()
        cap.release()


def main():
    global gData

    task1 = threading.Thread(target=video_thread)
    task2 = threading.Thread(target=serial_thread)
    task3 = threading.Thread(target=pi_server_thread)
    task4 = threading.Thread(target=arduino_serial_thread)
    
    task1.start()
    task2.start()
    task3.start()
    task4.start()
    
    try:
        while True:
            with data_lock:
                local_data = gData
                
            if "go" in local_data:
                print("ok go")
                motor_control(0, 1, 0, 1, 50)
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "back" in local_data:
                print("ok back")
                motor_control(1, 0, 1, 0, 50)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
                with data_lock:
                    gData = ""
            elif "left" in local_data:
                print("ok left")
                motor_control(1, 0, 0, 1, 50)
                set_leds(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "right" in local_data:
                print("ok right")
                motor_control(0, 1, 1, 0, 50)
                set_leds(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
                with data_lock:
                    gData = ""
            elif "stop" in local_data:
                print("ok stop")
                motor_control(0, 1, 0, 1, 0)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "bz_on" in local_data:
                print("ok buzzer on")
                p.start(50)
                p.ChangeFrequency(391)
                with data_lock:
                    gData = ""
            elif "bz_off" in local_data:
                print("ok buzzer off")
                p.stop()
                with data_lock:
                    gData = ""

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        bleSerial.close()
        p.stop()
        GPIO.cleanup()
    
if __name__ == '__main__':
    main()