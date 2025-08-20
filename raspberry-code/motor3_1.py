from flask import Flask, render_template_string, request
import RPi.GPIO as GPIO
import threading
import time

# --- MODIFIED: Pin Definitions for BTS7960 ---
# Use any two available GPIO pins
RPWM_PIN = 24  # RPWM (Right PWM) - Controls forward motion
LPWM_PIN = 23  # LPWM (Left PWM) - Controls backward motion

GPIO.setmode(GPIO.BCM)
GPIO.setup(RPWM_PIN, GPIO.OUT)
GPIO.setup(LPWM_PIN, GPIO.OUT)

# --- MODIFIED: PWM Setup for BTS7960 ---
# Create two PWM objects, one for each direction
pwm_forward = GPIO.PWM(RPWM_PIN, 1000)  # 1kHz frequency
pwm_backward = GPIO.PWM(LPWM_PIN, 1000) # 1kHz frequency

speed = 75  # Set a speed percentage (0-100)

# Start both PWM objects with a duty cycle of 0 (motor stopped)
pwm_forward.start(0)
pwm_backward.start(0)

# Flask app
app = Flask(__name__)

# --- MODIFIED: Motor control functions for BTS7960 ---
def forward():
    """Drives the motor forward by activating the forward PWM."""
    pwm_backward.ChangeDutyCycle(0)      # Ensure backward is off
    pwm_forward.ChangeDutyCycle(speed)   # Set forward speed

def backward():
    """Drives the motor backward by activating the backward PWM."""
    pwm_forward.ChangeDutyCycle(0)       # Ensure forward is off
    pwm_backward.ChangeDutyCycle(speed)  # Set backward speed

def stop():
    """Stops the motor by setting both PWM duty cycles to 0."""
    pwm_forward.ChangeDutyCycle(0)
    pwm_backward.ChangeDutyCycle(0)


# --- NO CHANGES BELOW THIS LINE ---

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
  <h1>Raspberry Pi Motor Control (BTS7960)</h1>
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
        stop() # Ensure motor is stopped
        pwm_forward.stop() # Stop PWM signals
        pwm_backward.stop()
        GPIO.cleanup()