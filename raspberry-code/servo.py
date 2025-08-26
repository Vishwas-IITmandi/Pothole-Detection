from flask import Flask, render_template_string, request
import RPi.GPIO as GPIO
import time

# --- Servo GPIO pins ---
SERVO1 = 18
SERVO2 = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO1, GPIO.OUT)
GPIO.setup(SERVO2, GPIO.OUT)

# --- Servo PWM setup (50 Hz typical for servos) ---
servo1 = GPIO.PWM(SERVO1, 50)
servo2 = GPIO.PWM(SERVO2, 50)

servo1.start(0)
servo2.start(0)

app = Flask(__name__)

# --- Helper function to set servo angle ---
def set_angle(servo, angle):
    """Move servo to given angle (0–180)."""
    duty = 2 + (angle / 18)   # Convert angle to duty cycle
    GPIO.output(servo, True)
    pwm = servo1 if servo == SERVO1 else servo2
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    pwm.ChangeDutyCycle(0)   # Stop sending continuous signal

def open_servos():
    set_angle(SERVO1, 90)  # Example open position
    set_angle(SERVO2, 90)

def close_servos():
    set_angle(SERVO1, 0)   # Example close position
    set_angle(SERVO2, 0)

page = """
<!DOCTYPE html>
<html>
<head>
<title>Servo Control</title>
<style>
  body { text-align: center; font-family: Arial; background-color: #f0f0f0; }
  h1 { color: #333; }
  .controls { display: inline-block; margin: 20px; }
  button { 
    width: 120px; 
    height: 80px; 
    font-size: 18px; 
    margin: 10px; 
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
document.addEventListener('keydown', (event) => {
  if (event.repeat) return;
  switch(event.key.toLowerCase()) {
    case 'w':
      document.getElementById('openBtn').classList.add('pressed');
      fetch('/move?dir=open');
      break;
    case 's':
      document.getElementById('closeBtn').classList.add('pressed');
      fetch('/move?dir=close');
      break;
  }
});

document.addEventListener('keyup', (event) => {
  switch(event.key.toLowerCase()) {
    case 'w':
      document.getElementById('openBtn').classList.remove('pressed');
      break;
    case 's':
      document.getElementById('closeBtn').classList.remove('pressed');
      break;
  }
});
</script>
</head>
<body>
  <h1>Servo Control</h1>
  <p class="info">Use W to open and S to close the servos</p>
  <div class="controls">
    <button id="openBtn" onclick="fetch('/move?dir=open')">Open (W)</button>
    <button id="closeBtn" onclick="fetch('/move?dir=close')">Close (S)</button>
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
    if dir == "open":
        open_servos()
    elif dir == "close":
        close_servos()
    return "ok"

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000)
    finally:
        print("Cleaning up GPIO...")
        servo1.stop()
        servo2.stop()
        GPIO.cleanup()
