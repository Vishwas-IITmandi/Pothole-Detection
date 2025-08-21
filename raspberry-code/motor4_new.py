#### code due to mismatch of keybindings
from flask import Flask, render_template_string, request
import RPi.GPIO as GPIO
import time

# --- MODIFIED: Pin setup for 2 BTS7960 motor drivers ---
# Left side motors are connected to one BTS7960
LEFT_RPWM = 17  # Forward
LEFT_LPWM = 27  # Backward

# Right side motors are connected to the other BTS7960
RIGHT_RPWM = 23 # Forward
RIGHT_LPWM = 17 # Backward

GPIO.setmode(GPIO.BCM)
pins = [LEFT_RPWM, LEFT_LPWM, RIGHT_RPWM, RIGHT_LPWM]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# --- MODIFIED: PWM setup for 2 BTS7960 drivers ---
# Create four PWM objects, one for each control pin
pwm_left_fwd = GPIO.PWM(LEFT_RPWM, 1000)
pwm_left_bwd = GPIO.PWM(LEFT_LPWM, 1000)
pwm_right_fwd = GPIO.PWM(RIGHT_RPWM, 1000)
pwm_right_bwd = GPIO.PWM(RIGHT_LPWM, 1000)

# Start all PWM objects with a duty cycle of 0 (stopped)
for pwm in [pwm_left_fwd, pwm_left_bwd, pwm_right_fwd, pwm_right_bwd]:
    pwm.start(0)

speed = 70  # Set a global speed percentage (0-100)

app = Flask(__name__)

# --- NEW: Helper function to control both sides ---
def set_motor_speeds(left_speed, right_speed):
    """Controls the speed and direction of left and right motor pairs.
    Speed is a value from -100 to 100. Positive is forward, negative is backward.
    """
    # Control Left Side
    if left_speed > 0:
        pwm_left_fwd.ChangeDutyCycle(left_speed)
        pwm_left_bwd.ChangeDutyCycle(0)
    else:
        pwm_left_fwd.ChangeDutyCycle(0)
        pwm_left_bwd.ChangeDutyCycle(abs(left_speed))

    # Control Right Side
    if right_speed > 0:
        pwm_right_fwd.ChangeDutyCycle(right_speed)
        pwm_right_bwd.ChangeDutyCycle(0)
    else:
        pwm_right_fwd.ChangeDutyCycle(0)
        pwm_right_bwd.ChangeDutyCycle(abs(right_speed))

# --- MODIFIED: High-level movement functions ---
def forward():
    set_motor_speeds(speed, speed)

def backward():
    set_motor_speeds(-speed, -speed)

def left():
    # Pivot turn: left side reverses, right side goes forward
    set_motor_speeds(-speed, speed)

def right():
    # Pivot turn: right side reverses, left side goes forward
    set_motor_speeds(speed, -speed)

def stop():
    set_motor_speeds(0, 0)

# --- NO CHANGES BELOW THIS LINE (Flask and HTML) ---

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
      startMove('left');
      break;
    case 'a':
      document.getElementById('leftBtn').classList.add('pressed');
      startMove('forward');
      break;
    case 's':
      document.getElementById('backwardBtn').classList.add('pressed');
      startMove('right');
      break;
    case 'd':
      document.getElementById('rightBtn').classList.add('pressed');
      startMove('backward');
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
        # --- MODIFIED: Cleanup for new PWM objects ---
        for pwm in [pwm_left_fwd, pwm_left_bwd, pwm_right_fwd, pwm_right_bwd]:
            pwm.stop()
        GPIO.cleanup()
