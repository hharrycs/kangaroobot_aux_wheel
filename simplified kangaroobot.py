import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import matplotlib.pyplot as plt

xml_path = 'simplified kangaroobot.xml'
simend = 15

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

model = mj.MjModel.from_xml_path('simplified kangaroobot.xml')
data = mj.MjData(model)

# find out the angle between ground and tail
mj.mj_step(model, data)
tail_body_id = model.body('tail').id
tail_rotmat = data.xmat[tail_body_id].reshape(3, 3)
tail_direction = tail_rotmat[:, 1]

ground_normal = np.array([0, 0, 1])
cos_theta = np.abs(np.dot(tail_direction, ground_normal)) / np.linalg.norm(tail_direction)
angle_rad = np.arccos(cos_theta)
angle_deg = np.degrees(angle_rad)

# to pull out the mass from the xml file
wheel_body_id = model.body("wheel").id

tail_geom_id = np.where(model.geom_bodyid == tail_body_id)[0][0]
wheel_geom_id = np.where(model.geom_bodyid == wheel_body_id)[0][0]
tail_mass = model.body_mass[tail_geom_id]
wheel_mass = model.body_mass[wheel_geom_id]
print(tail_mass, wheel_mass)

torque = None

# for ploting torque after the simulation
time_values = []
torque_values = []

def controller(model, data):
     global torque
     
     # Get actuator ID once
     if torque is None:
        torque = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "torque")
    
     tail_rotmat = data.xmat[tail_body_id].reshape(3, 3)
     tail_direction = tail_rotmat[:, 1]  # Using y-axis (adjust if needed)
    
     ground_normal = np.array([0, 0, 1])
     cos_theta = np.abs(np.dot(tail_direction, ground_normal)) / np.linalg.norm(tail_direction)
     angle_rad = np.arccos(cos_theta)
     angle_deg = np.degrees(angle_rad)

     data.ctrl[torque] = tail_mass * 9.81 * 0.5 * np.cos(np.radians(angle_deg)) + wheel_mass * 9.81 * np.cos(np.radians(angle_deg))
     
     
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

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
#model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
#data = mj.MjData(model)                # MuJoCo data
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
cam.azimuth = 165
cam.elevation = -20
cam.distance = 13.0
cam.lookat[:] = np.array([0.0, 0.0, 1.0]) #make the cam look at (0,0,1)

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

        # record torque and time at each step
        time_values.append(data.time)
        torque_values.append(data.ctrl[torque])

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

glfw.destroy_window(window)


plt.figure(figsize=(10, 6))
plt.plot(time_values, torque_values, label='Actuator Torque')
plt.xlabel('Time (s)')
plt.ylabel('Torque (Nm)')
plt.title('Torque Application Over Time')
plt.legend()
plt.grid(True)
plt.show()

glfw.terminate()