import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Rectangle
import matplotlib.transforms as transforms
import sys

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

# Draw the circle at the origin
circle = plt.Circle((0, 0), 0.5, color='blue', fill=True, linewidth=2)
ax.add_artist(circle)

# Function to calculate tangent rectangle coordinates based on angle
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
        color='green',
        fill=True
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
    quiver_wheel = ax.quiver(rect[1][0],rect[1][1],0,0,scale_units='xy',scale=1,color='red')
    quiver_wheels.append(quiver_wheel)

# Initial force vector using quiver
quiver_main = ax.quiver(0,0,1,0,angles='xy',scale_units='xy',scale=1,color='orange')

# Initialize rotational vector
rot_arrow = FancyArrowPatch((0, -0.25), (0, 0.25), connectionstyle="arc3,rad=1", 
                                 arrowstyle='->', color='yellow',linewidth=2,mutation_scale=10)
ax.add_patch(rot_arrow)

# Animation function
def animate_line_x(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = np.cos(angle)
    y_mag = 0
    w_mag = 0

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)
    
    # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))

    return quiver_main,

def animate_line_y(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = 0
    y_mag = np.sin(angle)
    w_mag = 0

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)

        # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))
    return quiver_main,

def animate_diagonal(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = np.cos(angle)
    y_mag = np.cos(angle)
    w_mag = 0

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)

    # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))
        
    return quiver_main,

def animate_rotate(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = 0
    y_mag = 0
    w_mag = np.cos(angle)

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)
    
    # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))

    return quiver_main,

def animate_circle(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = np.cos(angle)
    y_mag = np.sin(angle)
    w_mag = 0

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)
    
    # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))

    return quiver_main,

def animate_circle_and_rotate(i):
    # Calculate the end point of the vector based on angle
    angle = np.radians(i)
    x_mag = np.cos(angle)
    y_mag = np.sin(angle)
    w_mag = 1

    inverse_force = [[-0.33,0.58,0.33],[-0.33,-0.58,0.33],[0.66,0,0.33]]
    drive_vector = [x_mag,y_mag,w_mag]
    force_vector = np.matmul(inverse_force,drive_vector)
    
    # Update the quiver (arrow) direction
    quiver_main.set_UVC(x_mag, y_mag)
    for i in range(3):
        dx = np.cos(angles[i] + np.pi/2) * force_vector[i]
        dy = np.sin(angles[i] + np.pi/2) * force_vector[i]
        quiver_wheels[i].set_UVC(dx,dy)

    # update rotation vector
    if w_mag > 0:    
        rot_arrow.set_connectionstyle("arc3,rad=1")
    elif w_mag < 0:
        rot_arrow.set_connectionstyle("arc3,rad=-1")
    elif w_mag == 0:
        rot_arrow.set_positions((0,0),(0,0))
        
    return quiver_main,


def main():
    if len(sys.argv) != 2:
        print("Invalid arguments")
        return
    
    dig = sys.argv[1]

    # Run the animation based on user input
    match int(dig):
        case 1: # line x
            ani = animation.FuncAnimation(fig, animate_line_x, frames=360, interval=20, blit=False)
        case 2: # line y
            ani = animation.FuncAnimation(fig, animate_line_y, frames=360, interval=20, blit=False)    
        case 3: # line diagonal
            ani = animation.FuncAnimation(fig, animate_diagonal, frames=360, interval=20, blit=False)
        case 4: # circle
            ani = animation.FuncAnimation(fig, animate_circle, frames=360, interval=20, blit=False)
        case 5: # rotate
            ani = animation.FuncAnimation(fig, animate_rotate, frames=360, interval=20, blit=False)
        case 6: # circle + rotate
            ani = animation.FuncAnimation(fig, animate_circle_and_rotate, frames=360, interval=20, blit=False)
    plt.show()


if __name__ == "__main__":
    main()
