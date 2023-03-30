from dataclasses import dataclass


KNOWN_PATHS = [
    "ShelfSystemH200T7L12W/ShelfSystemH200T7L12W.dae",
    "ShelfSystemH180T5L10G/ShelfSystemH180T5L10G.dae",
    "ShelfSystemH160T5L10G/ShelfSystemH160T5L10G.dae",
    "ShelfSystemH180T5L6W/SM_ShelfSystemH180T5L6W.dae",
    "ShelfSystemH200T6L12W/ShelfSystemH200T6L12W.dae",
    "ShelfSystemH200T6L10W/ShelfSystemH200T6L10W.dae",
    "ShelfSystemH200T5L12W/SM_ShelfSystemH200T5L12W.dae",
    "ShelfSystemH200T6L7W/ShelfSystemH200T6L7W.dae",
    "ShelfSystemH160T6L10W/SM_ShelfSystemH160T6L10W.dae",
    "ShelfSystemH200T5L6W/ShelfSystemH200T5L6W.dae",
    "ShelfSystemH160T5L10W/ShelfSystemH160T5L10W.dae",
    "ShelfSystemH160T6L10G/ShelfSystemH160T6L10G.dae",
    "ShelfSystemH200T5L10W/ShelfSystemH200T5L10W.dae",
    "ShelfSystemH180T5L10W/SM_ShelfSystemH180T5L10W.dae",
    "ShelfSystemH180T510W/SM_ShelfSystemH180T5L10W.dae",
    "ShelfSystemH160T4L10W/SM_ShelfSystemH160T4L10W.dae",
    "ShelfSystemH200T7L10W/ShelfSystemH200T7L10W.dae",
    "ShelfSystemH200T6L6W/ShelfSystemH200T6L6W.dae"
]


@dataclass
class Shelf:
    shelf_id: str # e.g. from uri or or reference-id
    dtapi_id: int # 0
    uri: str # "http://knowrob.org/kb/dm-market.owl#DMShelfW100_EVZDYXFU"
    # path to 3D model (stl/dae-file)
    model: str # package://refills_models/shelf_models/<name>/<name>.dae
    # 
    # this data gets forwarded to ROS as well, see k4r_data_msgs/Shelf
    # 
    # position and orientation
    position: list # vector3
    orientation: list  # quaternion
    depth: float
    width: float
    height: float
    # defines if the pose rotation is a quaternion or a list of angles
    quaternion: bool # default = True
    # defines if the values for the orientation angles are radians or euler
    radians: bool # default = False

    def guess_model(self):
        # we assume all models are only one-sided, so 'W', not 'G'
        path_name = f'ShelfSystemH{int(self.height*100)}' \
            f'T{int(self.depth*10)}L{int(self.width)*10}W'
        for p in KNOWN_PATHS:
            if p.startswith(path_name):
                path_name = p
                self.model = f'package://refills_models/shelf_models/{p}'
                return self.model
        return ''