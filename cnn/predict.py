#!/usr/bin/env python
# -*- coding:utf-8 -*-
from cnn.CNN import CNN
from cnn.Data import Data
import cv2

model1 = CNN('Elements')
x = cv2.imread('../training_data/Button/11_104.jpg')
print(model1.predict_single(x))