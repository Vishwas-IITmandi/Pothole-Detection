import tkinter as tk
import RPi.GPIO as GPIO

# Motor GPIO pins
IN1 = 17
IN2 = 27

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Tkinter window
root = tk.Tk()
root.title("Raspberry Pi Motor Control")

# Debug label
status_label = tk.Label(root, text="Ready", font=("Arial", 14))
status_label.pack(pady=10)

def set_motor(forward=False, backward=False):
    if forward:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        status_label.config(text="Motor: FORWARD")
    elif backward:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        status_label.config(text="Motor: BACKWARD")
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        status_label.config(text="Motor: STOPPED")

# Button controls
tk.Button(root, text="Forward (W)", width=20, height=2, command=lambda: set_motor(forward=True)).pack(pady=5)
tk.Button(root, text="Backward (S)", width=20, height=2, command=lambda: set_motor(backward=True)).pack(pady=5)
tk.Button(root, text="Stop (Space)", width=20, height=2, command=lambda: set_motor()).pack(pady=5)
tk.Button(root, text="Quit (Q)", width=20, height=2, command=root.quit).pack(pady=5)

# Keyboard bindings
root.bind("<w>", lambda event: set_motor(forward=True))
root.bind("<s>", lambda event: set_motor(backward=True))
root.bind("<space>", lambda event: set_motor())
root.bind("<q>", lambda event: root.quit())

def on_closing():
    set_motor(False, False)  # Stop motor
    GPIO.cleanup()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start GUI loop
root.mainloop()
