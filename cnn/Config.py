import os
class Config:
    def __init__(self):
        cur_path = 
        self.DATA_PATH = os.path.abspath('.') + "/training_data"

        self.class_map = ['Button', 'Text', 'Noise']

        self.image_shape = (32, 32, 3)
        self.class_number = len(self.class_map)
