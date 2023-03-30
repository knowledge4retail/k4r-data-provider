from dataclasses import dataclass

@dataclass
class Product:
    # this data gets forwarded to ROS as well, see k4r_data_msgs/Product
    # should be in the form ANXXXXXXX
    gtin: str
    # path to 3D model (stl/dae-file)
    product_id: str # package://refills_models/product_model/<name>/<name>.dae
    # Human-readyble description for the product
    name: str
    # list of shelves that this product is in.
    shelf_ids: list
    # position and orientation on the shelf
    position: list
    orientation: list
    # defines if the pose rotation is a quaternion or a list of angles
    quaternion: bool # default = True
    # defines if the values for the orientation angles are radians or euler
    radians: bool # default = False