import os
from robosuite.models.objects import MujocoXMLObject

class BananaObject(MujocoXMLObject):
    """
    Banana object
    """
    def __init__(self, name):
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        super().__init__(
            cur_dir + "/assets/banana.xml",
            name=name,
            joints=[dict(type="free", damping="0")],
            obj_type="all",
            duplicate_collision_geoms=True
        )
