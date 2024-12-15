import mujoco
import numpy as np
import matplotlib.pyplot as plt
import mediapy as media
import time
import itertools
import cv2


# Load the MuJoCo model
xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
        <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
        <geom name="red_box" type="box" size=".2 .2 .2" rgba="0.5 0 0.5 1"/>
        <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
model.opt.gravity = (0, 0, 10)

duration = 3.8
framerate = 60

frames = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
    # Initialize OpenCV video writer (MP4 format with H.264 codec)
    frame_height, frame_width, _ = renderer.render().shape  # Assuming RGB image format
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
    out = cv2.VideoWriter('simulation_output.mp4', fourcc, framerate, (frame_width, frame_height))

    while data.time < duration:
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data, scene_option=scene_option)
            pixels = renderer.render()
            frames.append(pixels)

            frame_bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
            out.write(frame_bgr)

    out.release()

#with mujoco.Renderer(model) as renderer:
#    mujoco.mj_forward(model, data)
#    renderer.update_scene(data)
#
#    image = renderer.render()
#    cv2.imwrite("render.png", np.array(image))

