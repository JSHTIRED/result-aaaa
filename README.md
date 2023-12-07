# result-aiot
2023한강 aiot대회 참가 프로그램

this project is detect person and knife by moblienet V2
```mermaid
sequenceDiagram
    participant RC_car_cam
    participant PC
    participant Contorler
    participant Police
    participant RC_car_arduino

loop process
    RC_car_cam->>PC : connect server(pc) by UDP
    RC_car_cam->>PC : send image
    PC->>PC : AI image processing
    loop if_detect_person
      PC->>PC : detect knife
      loop if_detect_knife
        PC->>Police : send message position and time
        PC->>RC_car_cam : call buzzer
      end
    end
    Contorler->>RC_car_arduino : send move signal
    RC_car_arduino->>RC_car_arduino : Obstacle Identification by ultrasonic sensor
    loop if_detct_Obstacle
      RC_car_arduino->>RC_car_arduino : avoidance of Obstacles
    end
end
```

rc_car.py
```py
pip install serial
pip install opencv-python
pip install GPIO
```
detection.py
```py
pip install numpy
pip install Twilio
pip install opencv-python
```

![image](https://github.com/JSHTIRED/result-aaaa/assets/143377935/dbdc6da9-276f-403e-b090-9f7483dc466d)



![image](https://github.com/JSHTIRED/result-aaaa/assets/143377935/62eea04e-1edd-418f-895c-7940cd48afa4)


![image](https://github.com/JSHTIRED/result-aaaa/assets/143377935/03032181-883c-40b1-8db0-d88e30d4a17a)
