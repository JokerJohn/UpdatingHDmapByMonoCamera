from mmdet.apis import init_detector, inference_detector, show_result
import mmcv
import pickle
import cv2
import os
import numpy as np
import pickle

config_file = '/home/kyj/mmdetection/configs/cascade_mask_rcnn_x101_64x4d_fpn_1x.py'
checkpoint_file = '/home/kyj/mmdetection/cascade_mask_rcnn_x101_64x4d_fpn_1x_20190501-827e0a70.pth'
model = init_detector(config_file, checkpoint_file, device='cuda:0')

root = 'home/data/kyj/mmdetection/images'
imgs = os.listdir(root)
imgs.sort()
for img in imgs:
	path = os.path.join(root, img)
	result = inference_detector(model, path)
	show_result(path, result, model.CLASSES)