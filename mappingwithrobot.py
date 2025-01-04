import numpy as np
import matplotlib.pyplot as plt
import serial
import threading
import keyboard
from PIL import Image
from flask import Flask, Response
from io import BytesIO
import telebot
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
CHAT_ID = "1243982228"

# Initialize the Flask app
app = Flask(__name__)

# Initialize grid map
grid_size = (20, 20)  # 20x20 grid
grid_map = np.zeros(grid_size)  # Empty grid

# Initially empty obstacle list
obstacles = []

start_pos = (0, 0)  # Starting position

# Directions for moving (right, down, left, up)
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

@app.route('/')
def index():
    return app.send_static_file('index.html')

# Flask route to serve the map
@app.route('/map')
def get_map():
    global map_image_buffer
    if map_image_buffer is None:
        return "Map is not ready", 503
    
    return Response(map_image_buffer, mimetype='image/png')

def send_notification(message):
    """Send obstacle notification."""
    bot = telebot.TeleBot(token=BOT_TOKEN)
    bot.send_message(CHAT_ID, message)

# Function to check if a position is within bounds and not an obstacle
def is_valid_move(x, y):
    if 0 <= x < grid_map.shape[1] and 0 <= y < grid_map.shape[0]:  # Ensure within bounds
        return True
    return False

# Function to check for 'q' press outside the main loop to stop
def check_exit():
    return keyboard.is_pressed("q")

def is_clear(direction, robot_position, grid_map, obstacles):
    """Check if the direction is clear (no obstacle)."""
    x, y = robot_position
    dx, dy = direction
    new_x, new_y = x + dx, y + dy
    if is_valid_move(new_x, new_y) and (new_x, new_y) not in obstacles:
        return True
    return False

# Function to detect obstacles based on distance
def detect_obstacle(distance, x, y):
    # Consider any distance less than 30 cm as an obstacle
    if distance < 10:  
        send_notification(f"Obstacle detected at ({x}, {y})!")
        return True
    return False

# Set up serial communication (make sure to replace the correct COM port)
ser = serial.Serial('COM7', 9600, timeout=1)  # Replace 'COM3' with your ESP32's port

# Function to read ultrasonic data from serial
def read_distance():
    if ser.in_waiting > 0:
        return float(ser.readline().decode('utf-8').strip())  # Read the serial data and convert to float
    return None

# Directions (right, down, left, up)
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

def move_with_wall_following(robot_position, current_direction, directions):
    """Move the robot using wall-following logic."""
    # Find the left, front, and right directions relative to the current direction
    current_idx = directions.index(current_direction)
    left_direction = directions[(current_idx - 1) % len(directions)]  # Turn left
    front_direction = current_direction  # Move straight
    right_direction = directions[(current_idx + 1) % len(directions)]  # Turn right
    back_direction = directions[(current_idx + 2) % len(directions)]  # Move backward

    # Turn left if there's space
    if is_clear(left_direction, robot_position, grid_map, obstacles):
        new_x, new_y = robot_position[0] + left_direction[0], robot_position[1] + left_direction[1]
        return (new_x, new_y), left_direction

    # Move straight if there's no obstacle in front
    if is_clear(front_direction, robot_position, grid_map, obstacles):
        new_x, new_y = robot_position[0] + front_direction[0], robot_position[1] + front_direction[1]
        return (new_x, new_y), front_direction

    # Turn right if blocked in front
    if is_clear(right_direction, robot_position, grid_map, obstacles):
        new_x, new_y = robot_position[0] + right_direction[0], robot_position[1] + right_direction[1]
        return (new_x, new_y), right_direction

    # If surrounded, move backward
    if is_clear(back_direction, robot_position, grid_map, obstacles):
        new_x, new_y = robot_position[0] + back_direction[0], robot_position[1] + back_direction[1]
        return (new_x, new_y), back_direction

    # If no move is possible, stay in the same position
    return robot_position, current_direction

# Main loop to move the robot with wall-following logic
robot_position = start_pos
current_direction = directions[0]  # Start moving to the right

# Load the grass image for the background
grass_image = Image.open("grass.jpg")  # Replace with your grass image path

# Convert it to a NumPy array
grass_image = np.array(grass_image)

# Run Flask server in a separate thread
flask_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000, debug=False), daemon=True)
flask_thread.start()

# Main logic
def robot_control():
    global robot_position, current_direction, obstacles
    while True:
        if check_exit():
            plt.ioff()
            plt.show()
            print("Exiting...")
            break

        # Read the distance from the ultrasonic sensor
        distance = read_distance()

        if distance is not None:
            print(f"Distance: {distance} cm")
            
            # Check if the robot detects an obstacle
            if detect_obstacle(distance, *robot_position):
                # Calculate the position 2 grids ahead in the robot's current direction
                obstacle_x = robot_position[0] + 1 * current_direction[0]
                obstacle_y = robot_position[1] + 1 * current_direction[1]

                if is_valid_move(obstacle_x, obstacle_y) and (obstacle_x, obstacle_y) not in obstacles:
                    obstacles.append((obstacle_x, obstacle_y))  # Add to obstacles list
                    grid_map[obstacle_x, obstacle_y] = 1  # Mark obstacle in grid

        # Move the robot using wall-following logic
        robot_position, current_direction = move_with_wall_following(
            robot_position, current_direction, directions
        )

        save_map_image()  # Save map image to a buffer
        # Pause to update the plot in real-time
        plt.pause(0.5)

def save_map_image():
    global grid_map, robot_position, obstacles
    
    # Create and save the map image (to be used by Flask)
    plt.clf()
    
    # Display the grass image as the background
    plt.imshow(grass_image, origin="upper", extent=(-0.5, grid_size[1]-0.5, grid_size[0]-0.5, -0.5))  # Set the background to grass image
    
    # Plot obstacles and robot position
    for x, y in obstacles:
        plt.plot(y, x, 'ws')  # Plot obstacles
        # Use scatter for better alignment
        plt.scatter(y, x, color='white', s=150, marker='s', label="Obstacle")
    
    plt.text(robot_position[1] + 0.2, robot_position[0] - 0.2, 
             f"({robot_position[1]}, {robot_position[0]})", color='red', fontsize=10, weight='bold')
    plt.plot(robot_position[1], robot_position[0], 'ro', label="Robot")  # Plot robot
    plt.axis('off')

    # Save the plot into a buffer to return as a PNG image
    buf = BytesIO()
    plt.savefig(buf, format='png')
    buf.seek(0)
    
    # You can store the buffer in a global variable or use it directly in the Flask route.
    # This image will be served by the '/map' route
    global map_image_buffer
    map_image_buffer = buf

if __name__ == "__main__":
    # Run Flask server in a separate thread
    flask_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000, debug=False), daemon=True)
    flask_thread.start()

    # Start the robot control function
    robot_control()
