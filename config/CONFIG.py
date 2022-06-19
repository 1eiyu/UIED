from os.path import join as pjoin
import os
parent_path = os.path.abspath('..')

class Config:

    def __init__(self):
        # setting CNN (graphic elements) model
        self.image_shape = (32, 32, 3)
        self.CNN_PATH = os.path.join(parent_path, 'cnn','weights','cnn-test.h5')
        self.element_class = ['Button', 'Text', 'Noise', 'Slidebar']
        self.class_number = len(self.element_class)
        self.model_saved = os.path.join(parent_path, 'cnn','weights','cnn-test.h5')
        # setting EAST (ocr) model
        self.EAST_PATH = os.path.join(parent_path, 'east_icdar2015_resnet_v1_50_rbox')

        self.COLOR = {'Button': (0, 255, 0),
                      'Text': (0, 0, 255),
                      'Noise': (6, 6, 255),
                      'Slidebar': (255, 0, 0)}

        # self.COLOR = {'Button': (0, 255, 0),
        #               'Text': (0, 0, 255),
        #               'Noise': (255, 0, 0),}
    def build_output_folders(self):
        # setting data flow paths
        self.ROOT_INPUT = "./data"
        self.ROOT_OUTPUT = "./model"

        self.ROOT_IMG_ORG = pjoin(self.ROOT_INPUT, "org")
        self.ROOT_IP = pjoin(self.ROOT_OUTPUT, "ip")
        self.ROOT_OCR = pjoin(self.ROOT_OUTPUT, "ocr")
        self.ROOT_MERGE = pjoin(self.ROOT_OUTPUT, "merge")
        self.ROOT_IMG_COMPONENT = pjoin(self.ROOT_OUTPUT, "components")
        if not os.path.exists(self.ROOT_IP):
            os.mkdir(self.ROOT_IP)
        if not os.path.exists(self.ROOT_OCR):
            os.mkdir(self.ROOT_OCR)
        if not os.path.exists(self.ROOT_MERGE):
            os.mkdir(self.ROOT_MERGE)
