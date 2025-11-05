import RPi.GPIO as GPIO
import time
import serial
import threading

PWMA = 18
AIN1 = 22
AIN2 = 27

PWMB = 23
BIN1 = 25
BIN2 = 24

SPEED = 50
gData = "stop"

Command = {
    "m1": "go",
    "m2": "back",
    "m3": "left",
    "m4": "right",
    "m5": "stop"
}

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 500)
R_Motor = GPIO.PWM(PWMB, 500)
L_Motor.start(0)
R_Motor.start(0)

try:
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)  

except serial.SerialException as e:
    ser = None 

def stop():
    L_Motor.ChangeDutyCycle(0)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 0)

def go(): 
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 1)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 1)
    L_Motor.ChangeDutyCycle(SPEED)
    R_Motor.ChangeDutyCycle(SPEED)

def back(): 
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)
    L_Motor.ChangeDutyCycle(SPEED)
    R_Motor.ChangeDutyCycle(SPEED)

def right(): 
    GPIO.output(AIN1, 0)
    GPIO.output(AIN2, 1)
    GPIO.output(BIN1, 1)
    GPIO.output(BIN2, 0)
    L_Motor.ChangeDutyCycle(SPEED)
    R_Motor.ChangeDutyCycle(SPEED)

def left(): 
    GPIO.output(AIN1, 1)
    GPIO.output(AIN2, 0)
    GPIO.output(BIN1, 0)
    GPIO.output(BIN2, 1)
    L_Motor.ChangeDutyCycle(SPEED)
    R_Motor.ChangeDutyCycle(SPEED)

def serial_thread():
    global gData, ser
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip().lower()
                if line:
                    if line in Command:
                        gData = Command[line]
                        
                    else:
                       
                        gData = line
                        
            time.sleep(0.01)
        except Exception:
            print(f"시리얼 통신 오류: {Exception}")
            break

def main():
    global gData
    current_action = "stop" 

    if ser is not None:
        task1 = threading.Thread(target=serial_thread)
        task1.daemon = True 
        task1.start()
    
    print("로봇 제어 시작.")

    try:
        while True:
            command = gData 
            
            if command != current_action:
                
                if command == "go":
                    go()
                elif command == "back":
                    back()
                elif command == "left":
                    left()
                elif command == "right":
                    right()
                elif command == "stop":
                    stop()
                
                if command in command.values(): 
                     current_action = command
                     
            time.sleep(0.02) 

    except KeyboardInterrupt:
        pass

    finally:
        stop()
        if ser:
            ser.close()

        L_Motor.stop()
        R_Motor.stop()
        GPIO.cleanup()
        

if __name__ == "__main__":
    main()