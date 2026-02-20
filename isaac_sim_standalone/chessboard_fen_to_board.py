import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets", "chess_set", "obj")

 
#fen = "4k3/6P1/8/8/8/8/8/4K3"
fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"
# fen = "rnbqkbnr/pppppppp/8/6qq/QQ6/8/PPPPPPPP/RNBQKBNR" #default starting position with extra queens for promotion testing
box_size = 0.03718857142
save_promotion_queens = True # add 2 extra queens for promotion testing, set to False to only have 1 queen per color


def fen_to_dict(fen, box_size):
    board = {}
    rows = fen.split("/")
    for rank_idx, row in enumerate(rows):
        rank = 8 - rank_idx
        file_idx = 0
        for ch in row:
            if ch.isdigit():
                for _ in range(int(ch)):
                    square = chr(ord('a') + file_idx) + str(rank)
                    x = (rank - 4.5) * box_size
                    y = (3.5 - file_idx) * box_size
                    board[square] = {"piece": "e", "x": x, "y": y}
                    file_idx += 1
            else:
                square = chr(ord('a') + file_idx) + str(rank)
                x = (rank - 4.5) * box_size
                y = (3.5 - file_idx) * box_size
                board[square] = {"piece": ch, "x": x, "y": y}
                file_idx += 1
    if save_promotion_queens:
        # Add extra queens for promotion testing
        #promotion white queens to a1.x and a1.y + 2 * box_size, a2.x and a2.y + 2 * box_size
        #promotion black queens to h7.x and h7.y + 2 * box_size, h8.x and h8.y + 2 * box_size

        for color in ['Q', 'q']:
            for i in range(2):
                if color == 'Q':
                    square = f"a{1 + i}"
                    x = (int(square[1]) - 4.5) * box_size
                    y = (3.5 - (ord(square[0]) - ord('a'))) * box_size + 2 * box_size
                else:
                    square = f"h{7 + i}"
                    x = (int(square[1]) - 4.5) * box_size
                    y = (3.5 - (ord(square[0]) - ord('a'))) * box_size - 2 * box_size
                board[square + "_promo"] = {"piece": color, "x": x, "y": y}
    return board

board = fen_to_dict(fen, box_size)

usd_white_bishop = os.path.join(ASSETS_DIR, "white_bishop.usd")
usd_white_rook =   os.path.join(ASSETS_DIR, "white_rook.usd")
usd_white_knight = os.path.join(ASSETS_DIR, "white_knight.usd")
usd_white_queen =  os.path.join(ASSETS_DIR, "white_queen.usd")
usd_white_king =   os.path.join(ASSETS_DIR, "white_king.usd")
usd_white_pawn =   os.path.join(ASSETS_DIR, "white_pawn.usd")
usd_black_bishop = os.path.join(ASSETS_DIR, "black_bishop.usd")
usd_black_rook =   os.path.join(ASSETS_DIR, "black_rook.usd")
usd_black_knight = os.path.join(ASSETS_DIR, "black_knight.usd")
usd_black_queen =  os.path.join(ASSETS_DIR, "black_queen.usd")
usd_black_king =   os.path.join(ASSETS_DIR, "black_king.usd")
usd_black_pawn =   os.path.join(ASSETS_DIR, "black_pawn.usd")

usd_paths = {
    "B": (usd_white_bishop, "white_bishop"),
    "R": (usd_white_rook, "white_rook"),
    "N": (usd_white_knight, "white_knight"),
    "Q": (usd_white_queen, "white_queen"),
    "K": (usd_white_king, "white_king"),
    "P": (usd_white_pawn, "white_pawn"),
    "b": (usd_black_bishop, "black_bishop"),
    "r": (usd_black_rook, "black_rook"),
    "n": (usd_black_knight, "black_knight"),
    "q": (usd_black_queen, "black_queen"),
    "k": (usd_black_king, "black_king"),
    "p": (usd_black_pawn, "black_pawn"),
}

is_paths_ok = True
for upath, _ in usd_paths.values():
    if not os.path.exists(upath):
        print(f"USD file not found: {upath}")
        is_paths_ok = False
print(f"All USD paths valid: {is_paths_ok}")



from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

# -------------------------------------------------
# IMPORTS
# -------------------------------------------------
import omni
import carb
carb.settings.get_settings().set("/log/level", "error")

import numpy as np
from pxr import Semantics, Usd, UsdPhysics, UsdGeom, PhysxSchema, UsdShade

from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import is_stage_loading, add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# -------------------------------------------------
# ENABLE EXTENSIONS (before stage so node types are registered)
# -------------------------------------------------
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("omni.physx", True)
manager.set_extension_enabled_immediate("omni.physx.cooking", True)
manager.set_extension_enabled_immediate("omni.physx.tensors", True)
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

for _ in range(10):
    simulation_app.update()

# -------------------------------------------------
# LOAD STAGE
# -------------------------------------------------

usd_path = os.path.join(PROJECT_ROOT, "assets", "isaac_sim_urdfs", "chessboard.usd")

omni.usd.get_context().open_stage(usd_path)

while is_stage_loading():
    simulation_app.update()

for _ in range(100):
    simulation_app.update()

# -------------------------------------------------
# SPAWN CHESS PIECES
# -------------------------------------------------
pieces_added = []
def piece_counter(piece_name):
    count = 0
    for p in pieces_added:
        if piece_name in p[0]:
            count += 1
    return count

stage = omni.usd.get_context().get_stage()

for square, info in board.items():
    piece = info["piece"]
    if piece != "e":
        usd_path, prim_name = usd_paths[piece]

        p_count = piece_counter(prim_name)
        prim_path = f"/World/{prim_name}" if p_count == 0 else f"/World/{prim_name}_{p_count}"
        add_reference_to_stage(usd_path, prim_path)
        print(f"prim_path: {prim_path}, prim_name: {prim_name}, piece: {piece}, square: {square}")
        pieces_added.append((prim_path, prim_name, info["x"], info["y"]))


while is_stage_loading():
    simulation_app.update()

for _ in range(200):
    simulation_app.update()


for prim_path, prim_name, x, y in pieces_added:
    prim = XFormPrim(prim_path, prim_name)
    if("pawn" in prim_name):
        prim.set_local_scale((0.081, 0.081, 0.081))
    elif ("rook" in prim_name):
        prim.set_local_scale((0.068, 0.068, 0.07))
    else:
        prim.set_local_scale((0.07, 0.07, 0.07))

    prim.set_world_pose(position=(x, y, 0.01526), orientation=euler_angles_to_quat(np.array([0, 0, 0])))

    # Add rigid body to parent prim (gravity)
    stage = omni.usd.get_context().get_stage()
    usd_prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.RigidBodyAPI.Apply(usd_prim)
    UsdPhysics.MassAPI.Apply(usd_prim)
    UsdPhysics.MassAPI(usd_prim).GetMassAttr().Set(0.001)

    # Add collision to child mesh prims
    for child in Usd.PrimRange(usd_prim):
        if child.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI(child).GetApproximationAttr().Set("convexHull")


# -------------------------------------------------
# START TIMELINE
# -------------------------------------------------
sim = SimulationContext(stage_units_in_meters=1.0)
sim.play()
simulation_app.update()

# -------------------------------------------------
# ENABLE COLLISION DEBUG VISUALIZATION
# -------------------------------------------------
settings = carb.settings.get_settings()
settings.set("/persistent/physics/visualizationDisplayColliders", True)
settings.set("/persistent/physics/visualizationDisplayCollidersFaces", True)
settings.set("/persistent/physics/visualizationDisplayCollidersEdges", True)
settings.set("/physics/debugDraw", True)

# -------------------------------------------------
# SPAWN OBJECT
# -------------------------------------------------

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
