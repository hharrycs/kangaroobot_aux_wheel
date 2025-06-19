import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt

xml_path = 'practice.xml'
simend = 15

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

kp = 3000.0      # proportional gain 
kd = 300.0       # derivative gain
target_slide_pos = 0     #(within -0.19~0)

def controller(model, data):
    # Access joint positions and velocities by name
    left_slide_pos = data.joint('left_slide').qpos
    left_slide_vel = data.joint('left_slide').qvel
    right_slide_pos = data.joint('right_slide').qpos
    right_slide_vel = data.joint('right_slide').qvel

    # PD control for left slide
    error_left = target_slide_pos - left_slide_pos
    derror_left = -left_slide_vel
    data.ctrl[0] = kp * error_left + kd * derror_left

    # PD control for right slide
    error_right = target_slide_pos - right_slide_pos
    derror_right = -right_slide_vel
    data.ctrl[1] = kp * error_right + kd * derror_right


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right
    
    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

model = mj.MjModel.from_xml_path('Modelling/practice.xml')
data = mj.MjData(model)

height_positions = []
time_stamps = []

def plot_graph(time_stamps, height_positions):
    plt.figure(figsize=(10, 6))
    plt.plot(time_stamps, height_positions, label='Height Position (m)')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.title('Height Position vs Time')
    plt.legend()
    plt.grid(True)
    plt.show()

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth = 160
cam.elevation = -20
cam.distance = 3.0
cam.lookat[:] = np.array([0.0, 0.0, 0.0]) #make the cam look at (0,0,1)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

#set the controller
mj.set_mjcb_control(controller)

#print(os.name)

while not glfw.window_should_close(window):
    simstart = data.time

    while (data.time - simstart < 1.0/60.0):
        mj.mj_step(model, data)
        
        # record data of height and time
        current_height = data.sensordata[2]
        height_positions.append(current_height)
        time_stamps.append(data.time)

    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

plot_graph(time_stamps, height_positions)

glfw.terminate()
