
class Config:
    def __init__(self):

        self.DATA_PATH = "./training_data"

        # self.MODEL_PATH = 'E:/Mulong/Model/rico_compos/cnn2-textview.h5'
        self.class_map = ['Button', 'Text', 'Noise']

        self.image_shape = (32, 32, 3)
        self.class_number = len(self.class_map)
