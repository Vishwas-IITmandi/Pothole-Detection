from flask import Flask, render_template_string, request
import RPi.GPIO as GPIO
import threading
import time

# Pin setup for 4 motors (Front Left, Front Right, Back Left, Back Right)
# Motor 1 - Front Left
FL_IN1 = 24
FL_IN2 = 23
FL_ENA = 25

# Motor 2 - Front Right
FR_IN1 = 22
FR_IN2 = 27
FR_ENA = 18

# Motor 3 - Back Left
BL_IN1 = 17
BL_IN2 = 4
BL_ENA = 12

# Motor 4 - Back Right
BR_IN1 = 16
BR_IN2 = 20
BR_ENA = 21

GPIO.setmode(GPIO.BCM)
pins = [FL_IN1, FL_IN2, FL_ENA, FR_IN1, FR_IN2, FR_ENA, BL_IN1, BL_IN2, BL_ENA, BR_IN1, BR_IN2, BR_ENA]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# PWM setup
pwm_fl = GPIO.PWM(FL_ENA, 1000)
pwm_fr = GPIO.PWM(FR_ENA, 1000)
pwm_bl = GPIO.PWM(BL_ENA, 1000)
pwm_br = GPIO.PWM(BR_ENA, 1000)

speed = 70
for pwm in [pwm_fl, pwm_fr, pwm_bl, pwm_br]:
    pwm.start(speed)

app = Flask(__name__)

def motor_control(fl_dir, fr_dir, bl_dir, br_dir):
    # Front Left
    GPIO.output(FL_IN1, GPIO.HIGH if fl_dir == 1 else GPIO.LOW)
    GPIO.output(FL_IN2, GPIO.HIGH if fl_dir == -1 else GPIO.LOW)
    # Front Right
    GPIO.output(FR_IN1, GPIO.HIGH if fr_dir == 1 else GPIO.LOW)
    GPIO.output(FR_IN2, GPIO.HIGH if fr_dir == -1 else GPIO.LOW)
    # Back Left
    GPIO.output(BL_IN1, GPIO.HIGH if bl_dir == 1 else GPIO.LOW)
    GPIO.output(BL_IN2, GPIO.HIGH if bl_dir == -1 else GPIO.LOW)
    # Back Right
    GPIO.output(BR_IN1, GPIO.HIGH if br_dir == 1 else GPIO.LOW)
    GPIO.output(BR_IN2, GPIO.HIGH if br_dir == -1 else GPIO.LOW)

def forward():
    motor_control(1, 1, 1, 1)

def backward():
    motor_control(-1, -1, -1, -1)

def left():
    motor_control(-1, 1, -1, 1)

def right():
    motor_control(1, -1, 1, -1)

def stop():
    motor_control(0, 0, 0, 0)

page = """
<!DOCTYPE html>
<html>
<head>
<title>4WD Motor Control</title>
<style>
  body { text-align: center; font-family: Arial; background-color: #f0f0f0; }
  h1 { color: #333; }
  .controls { display: inline-block; margin: 20px; }
  button { 
    width: 80px; 
    height: 80px; 
    font-size: 18px; 
    margin: 5px; 
    border-radius: 10px;
    border: none;
    color: white;
    background-color: #007BFF;
    cursor: pointer;
    transition: all 0.1s;
  }
  button:hover { background-color: #0056b3; }
  .pressed { background-color: #dc3545 !important; transform: scale(0.95); }
  .info { margin-top: 20px; color: #555; }
</style>
<script>
let moveInterval = null;
let isMoving = false;

function startMove(direction) {
  if (!isMoving) {
    isMoving = true;
    console.log('Moving:', direction);
    fetch('/move?dir=' + direction); 
    moveInterval = setInterval(() => {
      fetch('/move?dir=' + direction);
    }, 100);
  }
}

function stopMove() {
  if (isMoving) {
    clearInterval(moveInterval);
    isMoving = false;
    moveInterval = null;
    console.log('Stopped');
    fetch('/move?dir=stop');
  }
}

document.addEventListener('keydown', (event) => {
  if (event.repeat) return;
  switch(event.key.toLowerCase()) {
    case 'w':
      document.getElementById('forwardBtn').classList.add('pressed');
      startMove('forward');
      break;
    case 'a':
      document.getElementById('leftBtn').classList.add('pressed');
      startMove('left');
      break;
    case 's':
      document.getElementById('backwardBtn').classList.add('pressed');
      startMove('backward');
      break;
    case 'd':
      document.getElementById('rightBtn').classList.add('pressed');
      startMove('right');
      break;
  }
});

document.addEventListener('keyup', (event) => {
  switch(event.key.toLowerCase()) {
    case 'w':
      document.getElementById('forwardBtn').classList.remove('pressed');
      stopMove();
      break;
    case 'a':
      document.getElementById('leftBtn').classList.remove('pressed');
      stopMove();
      break;
    case 's':
      document.getElementById('backwardBtn').classList.remove('pressed');
      stopMove();
      break;
    case 'd':
      document.getElementById('rightBtn').classList.remove('pressed');
      stopMove();
      break;
  }
});
</script>
</head>
<body>
  <h1>4WD Raspberry Pi Control</h1>
  <p class="info">Use WASD keys or buttons to control the robot</p>
  <div class="controls">
    <div>
      <button id="forwardBtn" onmousedown="startMove('forward')" onmouseup="stopMove()">↑<br>W</button>
    </div>
    <div>
      <button id="leftBtn" onmousedown="startMove('left')" onmouseup="stopMove()">←<br>A</button>
      <button id="backwardBtn" onmousedown="startMove('backward')" onmouseup="stopMove()">↓<br>S</button>
      <button id="rightBtn" onmousedown="startMove('right')" onmouseup="stopMove()">→<br>D</button>
    </div>
  </div>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(page)

@app.route("/move")
def move():
    dir = request.args.get("dir")
    if dir == "forward":
        forward()
    elif dir == "backward":
        backward()
    elif dir == "left":
        left()
    elif dir == "right":
        right()
    else:
        stop()
    return "ok"

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000)
    finally:
        print("Cleaning up GPIO...")
        stop()
        for pwm in [pwm_fl, pwm_fr, pwm_bl, pwm_br]:
            pwm.stop()
        GPIO.cleanup()