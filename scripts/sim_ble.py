import asyncio
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Rectangle
import matplotlib.transforms as transforms
from bleak import BleakClient
import sys

# BLE Device info of the Arduino Nano on controller
DEVICE_MAC = "AD:71:06:8B:F7:02"
deviceServicePacketCharUUID = "891763a3-0d7f-48f1-b9fc-e403014730a2"

# Shared data (global variables)
power, rotate, driveX, driveY, driveW = 0,0,0,0,0
exit_flag = 0

# handle BLE notifications
# unpack string and update global variables with information from controller
def data_unpacker(sender, data):
    global power, rotate, driveX, driveY, driveW
    values = data.decode('utf-8').split("/")
    
    # Update shared variables   
    power = int(values[0])
    rotate = int(values[1])
    driveX = float(values[2])
    driveY = float(values[3])
    driveW = float(values[4])

async def listen_for_key():
    # Listens for keyboard input and stops when '0' is pressed
    global exit_flag            
    loop = asyncio.get_event_loop()
    while True:
        key = await loop.run_in_executor(None, sys.stdin.read, 1)  # Reads one character
        if key == "0":
            print("\nDetected '0' key press. Exiting...")
            exit_flag = 1
            return

# BLE Connection function
async def connect_to_KIWI():
    try:
        async with BleakClient(DEVICE_MAC) as client:
            if client.is_connected:
                print("Connected to BLE device!")
                print("Press 0 to exit program")
                await client.start_notify(deviceServicePacketCharUUID, data_unpacker) # listen for BLE packets on Notify flag
                await listen_for_key() # wait for key press to end listening
                await client.stop_notify(deviceServicePacketCharUUID)

    except Exception as e:
        print(f"Failed to connect: {e}")

# Matplotlib visualization function
async def update_plot():
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal', 'box')
    fig.set_size_inches(9,9)

    ax.set_title("KIWI DRIVE CONTROL", fontsize=16, color='black')
    
    # Draw the circle at the origin
    circle = plt.Circle((0, 0), 0.5, facecolor='blue', fill=True, edgecolor='black',linewidth=1)
    ax.add_artist(circle)

    def place_rectangle(angle):
        # Circle radius and rectangle size
        radius = 0.5
        rect_width, rect_height = 0.4, 0.2
        
        # Calculate position on the circle's circumference
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        rect_center = (x,y)

        # Create the rectangle
        rect = Rectangle(
            (x - rect_width/2, y - rect_height/2),
            rect_width,
            rect_height,
            facecolor='green',
            fill=True,
            edgecolor='black',
            linewidth=1
        )

        # Create a transformation for rotation around the rectangle's center
        trans = transforms.Affine2D().rotate_deg_around(x, y, np.degrees(angle + np.pi / 2)) + ax.transData
        rect.set_transform(trans)

        return rect,rect_center

    # Place three rectangles on the tangents, 120 degrees apart
    angles = [np.pi/6,np.pi/6 + 2 * np.pi / 3,np.pi/6 + 4 * np.pi / 3]
    rectangles = [place_rectangle(angle) for angle in angles]

    quiver_wheels = []
    for i,rect in enumerate(rectangles):
        ax.add_patch(rect[0])
        
        quiver_wheel = ax.quiver(rect[1][0],rect[1][1],0,0,scale_units='xy',scale=1,color='red',linewidth=1)
        quiver_wheels.append(quiver_wheel)

    # Initial force vector using quiver
    quiver_main = ax.quiver(0,0,1,0,angles='xy',scale_units='xy',scale=1,color='orange')

    # Initialize rotational vector
    rot_arrow = FancyArrowPatch((0, -0.25), (0, 0.25), connectionstyle="arc3,rad=1", 
                                    arrowstyle='->', color='yellow',linewidth=2,mutation_scale=10)
    ax.add_patch(rot_arrow)

    # Display Power and Rotate button states
    power_text = ax.text(-1.9, 1.7, f"Power: {power}", fontsize=16, color='white', bbox=dict(facecolor='black', alpha=0.6))
    rotate_text = ax.text(-1.9, 1.5, f"Rotate: {rotate}", fontsize=16, color='white', bbox=dict(facecolor='black', alpha=0.6))

    scale_factor = 1.0

    while not exit_flag:
        # calculate motor outputs same as Robot logic does
        # and update UI with values + main drive vector
        x_mag = -driveX
        y_mag = driveY
        w_mag = -driveW
        if not power:
            x_mag = y_mag = w_mag = 0
        if not rotate:
            scale_factor = 2.0
            w_mag = 0
        if rotate:
            scale_factor = 2.0
            x_mag *= 0.5
            y_mag *= 0.5
        
        x_mag = np.clip(x_mag, -1, 1)
        y_mag = np.clip(y_mag, -1, 1)
        w_mag = np.clip(w_mag, -1, 1)

        inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
        drive_vector = [x_mag,y_mag,w_mag]
        force_vector = np.matmul(inverse_force,drive_vector) * scale_factor
        force_vector = np.clip(force_vector, -1, 1)
        
        # Update the quiver (arrow) direction
        quiver_main.set_UVC(x_mag, y_mag)
        for i in range(3):
            dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
            dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
            quiver_wheels[i].set_UVC(dx,dy)

        rot_arrow.remove()

        if w_mag > 0.0:    
            rot_arrow = FancyArrowPatch((0, -0.25), (0, 0.25), connectionstyle="arc3,rad=1", 
                                        arrowstyle='->', color='yellow', linewidth=2, mutation_scale=10)
        elif w_mag < 0.0:
            rot_arrow = FancyArrowPatch((0, -0.25), (0, 0.25), connectionstyle="arc3,rad=-1", 
                                        arrowstyle='->', color='yellow', linewidth=2, mutation_scale=10)
        else:
            rot_arrow = FancyArrowPatch((0, 0), (0, 0), arrowstyle='->', color='yellow', linewidth=2, mutation_scale=10)

        ax.add_patch(rot_arrow)

        power_text.set_text(f"Power: {power}")
        rotate_text.set_text(f"Rotate: {rotate}")
        
        fig.canvas.draw()
        plt.pause(0.01)  # Allow UI to update
        await asyncio.sleep(0.01)  # Non-blocking wait for asyncio BLE updates
    
    plt.ioff()

# Run BLE and Matplotlib together
async def main():
    await asyncio.gather(
        connect_to_KIWI(),   # BLE Handling
        update_plot()        # Matplotlib Visualization
    )

# Start the async event loop
asyncio.run(main())
