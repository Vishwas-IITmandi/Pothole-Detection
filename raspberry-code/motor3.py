from flask import Flask, render_template_string, request
import RPi.GPIO as GPIO
import threading
import time

# Pin setup
IN1 = 24
IN2 = 23
ENA = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# PWM setup
pwm = GPIO.PWM(ENA, 1000)  # 1kHz
speed = 50
pwm.start(speed)

# Flask app
app = Flask(__name__)

# Motor control
def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)



# HTML template with keyboard controls
page = """
<!DOCTYPE html>
<html>
<head>
<title>Motor Control</title>
<style>
  body { text-align: center; font-family: Arial; background-color: #f0f0f0; }
  h1 { color: #333; }
  button { 
    width: 140px; 
    height: 70px; 
    font-size: 22px; 
    margin: 12px; 
    border-radius: 10px;
    border: none;
    color: white;
    background-color: #007BFF;
    box-shadow: 0 4px 6px rgba(0,0,0,0.1);
    cursor: pointer;
    transition: background-color 0.3s, transform 0.1s;
  }
  button:hover { background-color: #0056b3; }
  button:active { transform: scale(0.95); background-color: #0056b3; }
  .pressed { background-color: #dc3545 !important; transform: scale(0.95); }
  .info { margin-top: 20px; color: #555; }
</style>
<script>
let moveInterval = null;
let isMoving = false;

function startMove(direction) {
  if (!isMoving) {
    isMoving = true;
    console.log('Motor moving:', direction);
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
    console.log('Motor stopped');
    fetch('/move?dir=stop');
  }
}

document.addEventListener('keydown', (event) => {
  if (event.repeat) return; // Prevent multiple triggers on hold
  switch(event.key) {
    case 'w':
    case 'W':
      document.getElementById('forwardBtn').classList.add('pressed');
      startMove('forward');
      break;
    case 's':
    case 'S':
      document.getElementById('backwardBtn').classList.add('pressed');
      startMove('backward');
      break;
  }
});

document.addEventListener('keyup', (event) => {
  switch(event.key) {
    case 'w':
    case 'W':
      document.getElementById('forwardBtn').classList.remove('pressed');
      stopMove();
      break;
    case 's':
    case 'S':
      document.getElementById('backwardBtn').classList.remove('pressed');
      stopMove();
      break;
  }
});

</script>
</head>
<body>
  <h1>Raspberry Pi Motor Control</h1>
  <p class="info">Use the buttons below or your keyboard ('w' for forward, 's' for backward)</p>
  <button id="forwardBtn" onmousedown="startMove('forward')" onmouseup="stopMove()" ontouchstart="startMove('forward')" ontouchend="stopMove()">Forward (W)</button><br>
  <button id="backwardBtn" onmousedown="startMove('backward')" onmouseup="stopMove()" ontouchstart="startMove('backward')" ontouchend="stopMove()">Backward (S)</button>
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
    else:
        stop()
    return "ok"



if __name__ == "__main__":
    try:
        # Make sure to run with host='0.0.0.0' to access from your laptop
        app.run(host="0.0.0.0", port=5000)
    finally:
        print("Cleaning up GPIO...")
        stop()
        pwm.stop()
        GPIO.cleanup()
