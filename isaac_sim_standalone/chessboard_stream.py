# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES.
# SPDX-License-Identifier: Apache-2.0

from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
import carb
import numpy as np
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# ROS2 bridge
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Load main stage
usd_path = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/isaac_sim_urdfs/chessboard.usd"
omni.usd.get_context().open_stage(usd_path)

simulation_app.update()
simulation_app.update()

print("Loading stage...")
while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

# Simulation context
simulation_context = SimulationContext(stage_units_in_meters=1.0)
simulation_context.play()
simulation_context.step()

# ----------------------------
# SPAWN WHITE BISHOP
# ----------------------------
white_bishop_usd = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_bishop.usd"
white_bishop_prim_path = "/World/white_bishop"

add_reference_to_stage(
    usd_path=white_bishop_usd,
    prim_path=white_bishop_prim_path
)

bishop = XFormPrim(
    prim_path=white_bishop_prim_path,
    name="white_bishop"
)

rpy_deg = np.array([90.0, 0.0, 90.0])
rpy_rad = np.deg2rad(rpy_deg)

quat = euler_angles_to_quat(rpy_rad)  # w,x,y,z

bishop.set_world_pose(
    position=np.array([0.0371875 / 2 + 0.008, 0.0371875 / 2 + 0.008, 0.01526]),
    orientation=quat  # w,x,y,z
)

bishop.set_local_scale(
    np.array([0.0007, 0.0007, 0.0007])
)

# ----------------------------
# MAIN LOOP
# ----------------------------
while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
