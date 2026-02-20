import random
import numpy as np
from isaacsim import SimulationApp
import random, math
import os

simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux
from isaacsim.core.utils import prims
from pxr import UsdPhysics
import omni.replicator.core as rep
import omni.kit.app

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)


omni.usd.get_context().open_stage("C:\\Users\\Gorkem\\Desktop\\isaac_sim_urdfs\\warehouse_stack_env.usd")

stage = omni.usd.get_context().get_stage()

my_world = World(stage_units_in_meters=1.0)
my_world.reset()

while True:
    rep.orchestrator.step(rt_subframes=25)
    
simulation_app.close()