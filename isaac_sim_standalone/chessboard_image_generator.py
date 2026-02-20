from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

# -------------------------------------------------
# IMPORTS
# -------------------------------------------------
import omni
import numpy as np
import omni.replicator.core as rep
from pxr import Semantics, Usd, UsdLux

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import is_stage_loading, add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# -------------------------------------------------
# LOAD STAGE
# -------------------------------------------------
usd_path = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/isaac_sim_urdfs/chessboard_data_generator.usd"
omni.usd.get_context().open_stage(usd_path)

while is_stage_loading():
    simulation_app.update()

# -------------------------------------------------
# START TIMELINE (ZORUNLU)
# -------------------------------------------------
sim = SimulationContext(stage_units_in_meters=1.0)
sim.play()
simulation_app.update()

# -------------------------------------------------
# SPAWN OBJECT
# -------------------------------------------------

usd_white_bishop = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_bishop.usd"
usd_white_rook =   "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_rook.usd"
usd_white_knight = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_knight.usd"
usd_white_queen = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_queen.usd"
usd_white_king = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_king.usd"
usd_white_pawn = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/white_pawn.usd"
usd_black_bishop = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_bishop.usd"
usd_black_rook =   "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_rook.usd"
usd_black_knight = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_knight.usd"
usd_black_queen = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_queen.usd"
usd_black_king = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_king.usd"
usd_black_pawn = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/assets/chess_set/obj/black_pawn.usd"

white_bishop_path = "/World/white_bishop"
white_rook_path = "/World/white_rook"
white_knight_path = "/World/white_knight"
white_queen_path = "/World/white_queen"
white_king_path = "/World/white_king"
white_pawn_path = "/World/white_pawn"
black_bishop_path = "/World/black_bishop"
black_rook_path = "/World/black_rook"
black_knight_path = "/World/black_knight"
black_queen_path = "/World/black_queen"
black_king_path = "/World/black_king"

black_pawn_path = "/World/black_pawn"


add_reference_to_stage(usd_white_bishop, white_bishop_path)
add_reference_to_stage(usd_white_rook, white_rook_path)
add_reference_to_stage(usd_white_knight, white_knight_path)
add_reference_to_stage(usd_white_queen, white_queen_path)
add_reference_to_stage(usd_white_king, white_king_path)
add_reference_to_stage(usd_white_pawn, white_pawn_path)
add_reference_to_stage(usd_black_bishop, black_bishop_path)
add_reference_to_stage(usd_black_rook, black_rook_path)
add_reference_to_stage(usd_black_knight, black_knight_path)
add_reference_to_stage(usd_black_queen, black_queen_path)
add_reference_to_stage(usd_black_king, black_king_path)

add_reference_to_stage(usd_black_pawn, black_pawn_path)

for _ in range(100):
    simulation_app.update()

white_bishop = XFormPrim(white_bishop_path, "white_bishop")
white_bishop.set_local_scale([0.07, 0.07, 0.07])

white_rook = XFormPrim(white_rook_path, "white_rook")
white_rook.set_local_scale([0.07, 0.07, 0.07])

white_knight = XFormPrim(white_knight_path, "white_knight")
white_knight.set_local_scale([0.07, 0.07, 0.07])

white_queen = XFormPrim(white_queen_path, "white_queen")
white_queen.set_local_scale([0.07, 0.07, 0.07])

white_king = XFormPrim(white_king_path, "white_king")
white_king.set_local_scale([0.07, 0.07, 0.07])

white_pawn = XFormPrim(white_pawn_path, "white_pawn")
white_pawn.set_local_scale([0.07, 0.07, 0.07])

black_bishop = XFormPrim(black_bishop_path, "black_bishop")
black_bishop.set_local_scale([0.07, 0.07, 0.07])

black_rook = XFormPrim(black_rook_path, "black_rook")
black_rook.set_local_scale([0.07, 0.07, 0.07])

black_knight = XFormPrim(black_knight_path, "black_knight")
black_knight.set_local_scale([0.07, 0.07, 0.07])

black_queen = XFormPrim(black_queen_path, "black_queen")
black_queen.set_local_scale([0.07, 0.07, 0.07])

black_king = XFormPrim(black_king_path, "black_king")
black_king.set_local_scale([0.07, 0.07, 0.07])

black_pawn = XFormPrim(black_pawn_path, "black_pawn")
black_pawn.set_local_scale([0.07, 0.07, 0.07])

# -------------------------------------------------
# SEMANTIC LABELS (bbox icin gerekli)
# -------------------------------------------------
stage = omni.usd.get_context().get_stage()
sem_pieces = {
    white_bishop_path: "white_bishop",
    white_rook_path: "white_rook",
    white_knight_path: "white_knight",
    white_queen_path: "white_queen",
    white_king_path: "white_king",
    white_pawn_path: "white_pawn",
    black_bishop_path: "black_bishop",
    black_rook_path: "black_rook",
    black_knight_path: "black_knight",
    black_queen_path: "black_queen",
    black_king_path: "black_king",
    black_pawn_path: "black_pawn",
}
def apply_semantics_recursive(stage, root_path, label):
    root_prim = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root_prim):
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr().Set("class")
        sem.CreateSemanticDataAttr().Set(label)

for prim_path, label in sem_pieces.items():
    apply_semantics_recursive(stage, prim_path, label)

tolerance = 0.005


# -------------------------------------------------
# CAMERA
# -------------------------------------------------
camera_path = "/World/Camera"

# -------------------------------------------------
# DATASET PARAMS
# -------------------------------------------------
num_images = 20
base_position = np.array([
    0.0371875 / 2 + tolerance,
    0.0371875 / 2 + tolerance,
    0.01526
])

# -------------------------------------------------
# REPLICATOR (WRITER ONLY)
# -------------------------------------------------
position_camera_x = 0.0
position_camera_y = 0.0
position_camera_z = 0.6

tolerance_camera_x = 0.0
tolerance_camera_y = 0.0
tolerance_camera_z = 0.0

render_product = rep.create.render_product(
    camera_path,
    resolution=(1280, 720)
)

output_base = "/home/ustunovadesktop/Documents/projects/bizon_chess_player/datasets/chessboard_val"

# -------------------------------------------------
# MANUAL CAPTURE LOOP
# -------------------------------------------------
start_pose = (-0.0371875 * 3.5, -0.0371875 * 3.5, 0.01526)

def get_position(r, c, box_width, tolerance, offset):
    cc = (c + offset) % 8
    rr = (r + (offset // 8)) % 8

    rand_tol = np.random.uniform(-tolerance, tolerance, size=2)
    position = np.array([
        start_pose[0] + cc * box_width + rand_tol[0],
        start_pose[1] + rr * box_width + rand_tol[1],
        start_pose[2]
    ])
    return position

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir=output_base,
    rgb=True,
    bounding_box_2d_tight=True
)
writer.attach([render_product])

# -------------------------------------------------
# LIGHT RANDOMIZATION
# -------------------------------------------------
# Mevcut sahne isigini kullan
light_prim = stage.GetPrimAtPath("/World/SimpleRoom/RectLight")
light_usd = UsdLux.RectLight(light_prim)

camera_prim = XFormPrim(camera_path)

for r in range(8):
    for c in range(8):
        for i in range(num_images):
            # Randomize light intensity
            light_usd.GetIntensityAttr().Set(
                float(np.random.uniform(5000.0, 20000.0))
            )

            # Randomize camera position
            cam_x = position_camera_x + np.random.uniform(-tolerance_camera_x, tolerance_camera_x)
            cam_y = position_camera_y + np.random.uniform(-tolerance_camera_y, tolerance_camera_y)
            cam_z = position_camera_z
            camera_prim.set_world_pose(position=np.array([cam_x, cam_y, cam_z]))

            yaw = i * (360.0 / num_images)
            quat = euler_angles_to_quat(
                np.deg2rad([0.0, 0.0, yaw])
            )
            
            position = get_position(r, c, 0.0371875, tolerance, offset=0)
            white_bishop.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=1)
            white_rook.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=2)
            white_knight.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=3)
            white_queen.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=4)
            white_king.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=5)
            white_pawn.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=6)
            black_bishop.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=7)
            black_rook.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=8)
            black_knight.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=9)
            black_queen.set_world_pose(
                position=position,
                orientation=quat
            )

            position = get_position(r, c, 0.0371875, tolerance, offset=10)
            black_king.set_world_pose(
                position=position,
                orientation=quat
            )
            

            position = get_position(r, c, 0.0371875, tolerance, offset=11)
            black_pawn.set_world_pose(
                position=position,
                orientation=quat
            )



            # Temporal accumulation buffer'i temizlemek icin yeterli frame bekle
            for _ in range(8):
                simulation_app.update()

            rep.orchestrator.step(rt_subframes=4, pause_timeline=False)

print(f"DATASET DONE - {num_images} images generated")

simulation_app.close()
