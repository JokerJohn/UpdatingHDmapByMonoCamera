import os
import sys
import argparse
import time
import math

from yolo import YOLO, detect_video, detect_cam
from PIL import Image


def detext_imgs(yolo, imgs_folder_path, output_path):
    img_names = os.listdir(imgs_folder_path)
    if '.DS_Store' in img_names:
        img_names.remove('.DS_Store')
    img_names.sort()

    for img_name in img_names:
        img_path = os.path.join(imgs_folder_path, img_name)
        try:
            image = Image.open(img_path)
        except:
            print('Open Error! Try again!')
            continue
        else:
            r_image, out_boxes, out_scores, out_classes = yolo.detect_image(image)
            # r_image.show(title=img_name)
            # if output_path != "":
                # r_image.save(os.path.join(output_path, img_name))
            # r_image.close()
            with open("traffic_sign_bbox/" + os.path.splitext(img_name)[0] + '@traffic_sign' + '.txt', 'w') as f:
                assert len(out_boxes) == len(out_classes)
                for i in range(len(out_boxes)):
                    print(out_boxes, out_scores, out_classes)
                    x_min = math.ceil(out_boxes[i][0])
                    y_min = math.ceil(out_boxes[i][1])
                    x_max = math.floor(out_boxes[i][2])
                    y_max = math.floor(out_boxes[i][3])
                    label = out_classes[i] + 1
                    score = float(out_scores[i])
                    result = str(x_min) + " " + str(y_min) + " " + str(x_max) + " " + str(y_max) + " " + str(label) + " " + str(score)[:4] + '\n'
                    # print(result)
                    f.write(result)
    yolo.close_session()


def detect_img(yolo):
    while True:
        img = input('Input image filename:')
        try:
            image = Image.open(img)
        except:
            print('Open Error! Try again!')
            continue
        else:
            r_image = yolo.detect_image(image)
            print(type(r_image))
            r_image.show()
    yolo.close_session()


FLAGS = None

if __name__ == '__main__':
    # class YOLO defines the default value, so suppress any default here
    parser = argparse.ArgumentParser(argument_default=argparse.SUPPRESS)
    '''
    Command line options
    '''
    parser.add_argument(
        '--model', type=str,
        help='path to model weight file, default ' + YOLO.get_defaults("model_path")
    )

    parser.add_argument(
        '--anchors', type=str,
        help='path to anchor definitions, default ' + YOLO.get_defaults("anchors_path")
    )

    parser.add_argument(
        '--classes', type=str,
        help='path to class definitions, default ' + YOLO.get_defaults("classes_path")
    )

    parser.add_argument(
        '--gpu_num', type=int,
        help='Number of GPU to use, default ' + str(YOLO.get_defaults("gpu_num"))
    )

    parser.add_argument(
        '--image', default=False, action="store_true",
        help='Image detection mode, will ignore all positional arguments'
    )

    parser.add_argument(
        '--img_folder', default=False, action="store_true",
        help='Image folder detection mode, will ignore all positional arguments'
    )

    '''
    Command line positional arguments -- for video detection mode
    '''
    parser.add_argument(
        "--input", nargs='?', type=str, required=False, default='./path2your_video',
        help="Video input path"
    )

    parser.add_argument(
        "--output", nargs='?', type=str, default="",
        help="[Optional] Video output path"
    )

    parser.add_argument(
        "--camera", nargs='?', type=str, default="",
        help="Camera detection mode."
    )

    FLAGS = parser.parse_args()

    if FLAGS.img_folder:
        print("Image folder detection mode")
        detext_imgs(YOLO(**vars(FLAGS)), FLAGS.input, FLAGS.output)
    elif FLAGS.image:
        """
        Image detection mode, disregard any remaining command line arguments
        """
        print("Image detection mode")
        if "input" in FLAGS:
            print(" Ignoring remaining command line arguments: " + FLAGS.input + "," + FLAGS.output)
        detect_img(YOLO(**vars(FLAGS)))
    elif "camera" in FLAGS:
        detect_cam(YOLO(**vars(FLAGS)), FLAGS.output)
    elif "input" in FLAGS:
        detect_video(YOLO(**vars(FLAGS)), FLAGS.input, FLAGS.output)
    else:
        print("Must specify at least video_input_path.  See usage with --help.")
