import argparse
import time
import rospy
import sys, os
import cv2
import torch
import numpy as np
import torch.backends.cudnn as cudnn
from numpy import random
from cv_bridge import CvBridge
from pathlib import Path
from sensor_msgs.msg import Image

from utils.constants import Topics

sys.path.append("./yolov7")
PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..", ""))
try:
    from .models.experimental import attempt_load
    from .utils.datasets import LoadStreams, LoadImages, LoadImagesDelivered
    from .utils.general import (
        check_img_size,
        check_requirements,
        check_imshow,
        non_max_suppression,
        apply_classifier,
        scale_coords,
        xyxy2xywh,
        strip_optimizer,
        set_logging,
        increment_path,
    )
    from .utils.plots import plot_one_box, plot_3d_box
    from .utils.torch_utils import (
        select_device,
        load_classifier,
        time_synchronized,
        TracedModel,
    )
    from .utils.convert2d_to_3d import convert_2d_3d
except:
    from models.experimental import attempt_load
    from utils.datasets import LoadStreams, LoadImages, LoadImagesDelivered
    from utils.general import (
        check_img_size,
        check_requirements,
        check_imshow,
        non_max_suppression,
        apply_classifier,
        scale_coords,
        xyxy2xywh,
        strip_optimizer,
        set_logging,
        increment_path,
    )
    from utils.plots import plot_one_box, plot_3d_box
    from utils.torch_utils import (
        select_device,
        load_classifier,
        time_synchronized,
        TracedModel,
    )
    from utils.convert2d_to_3d import convert_2d_3d


def load_model_and_conf(opt, calc_3d=True, plot_3d=True):
    """load the detection model and the settings
    @param opt: the args of the detection
    @param calc_3d: if the 3d boxes should be calculated
    @param plot_3d: if the 3d boxes should be plot
    @return limg: the img to run detection on
    @return imgz: size of the img
    @return stride: stride for letterboxing
    @return device: if gpu should be used
    @return model: the nn model
    @return opt: set of arguments used in detection
    @return names: name of the object types that can be detected
    @return view_img: if the detection should be displayed
    @return colors: colors used for different types when showing the bounding boxes
    @return calc_3d: if the 3d boxes should calculated
    @return p_matrix: the projection matrix of the image to display the 3d boxes
    @return plot_3d: if the 3d boxes should be plot

    """
    # the projection matrix, still needs to be placed properly
    p_matrix = np.array(
        [
            [8.145377000000e02, 0.000000000000e00, 3.991493000000e02],
            [0.000000000000e00, 8.185377000000e02, 3.490000000000e02],
            [0.000000000000e00, 0.000000000000e00, 1.000000000000e00],
        ]
    )

    weights, view_img, imgsz, trace = (
        opt.weights,
        opt.view_img,
        opt.img_size,
        not opt.no_trace,
    )

    # Initialize
    set_logging()
    device = select_device(opt.device)
    # Load model

    try:
        # If run with ros launchfile
        import yaml
        from .models.yolo import Model
    except:
        # If run from a regular python file
        sys.path.append("./yolov7")
        import yaml
        from .models.yolo import Model
    # load state dict
    loaded_state_dict = torch.load(
        weights[0],
        map_location=device,
    )
    # get hyperparamters, these are the same for both models
    hyp_loc = f"{PATH}/src/yolov7/data/hyp.scratch.custom.yaml"
    with open(hyp_loc) as f:
        hyp = yaml.load(f, Loader=yaml.SafeLoader)  # load hyps

    # load cfg, this is different for each
    if not "tiny_robotino" in weights[0].lower():
        ### this is for workstations###
        cfg = f"{PATH}/src/yolov7/cfg/training/yolov7-tiny_robo_ws.yaml"
        nc = 2  # amount of classes, the same as in the cfg file
        names = ["workstation_c", "workstation_a"]
    else:
        ### this is for movable ###
        cfg = f"{PATH}/src/yolov7/cfg/training/yolov7-tiny_robo_movable.yaml"
        nc = 11
        names = [
            "sklappbox_c",
            "sklappbox_a",
            "box_c",
            "box_a",
            "hocker_c",
            "klappbox_c",
            "klappbox_a",
            "sbox_c",
            "sbox_a",
            "robotino",
            "hocker_a",
        ]

    model = Model(cfg, ch=3, nc=nc, anchors=hyp.get("anchors")).to("cpu")
    model.load_state_dict(loaded_state_dict)
    model.eval()
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    if trace:
        model = TracedModel(model, device, opt.img_size)
    # Get names and colors
    # names = model.module.names if hasattr(model, "module") else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    return (
        imgsz,
        stride,
        device,
        model,
        opt,
        names,
        view_img,
        colors,
        calc_3d,
        p_matrix,
        plot_3d,
    )


def loaded_detect(
    limg,
    imgsz,
    stride,
    device,
    model,
    opt,
    names,
    view_img,
    colors,
    calc_3d,
    p_matrix,
    plot_3d,
    show=False,
    node="localization",
):
    """run the detection for a passed image, also passes the bb to the 2d->3d converter
    @param limg: the img to run detection on
    @param imgz: size of the img
    @param stride: stride for letterboxing
    @param device: if gpu should be used
    @param model: the nn model
    @param opt: set of arguments used in detection
    @param names: name of the object types that can be detected
    @param view_img: if the detection should be displayed
    @param colors: colors used for different types when showing the bounding boxes
    @param calc_3d: if the 3d boxes should calculated
    @param p_matrix: the projection matrix of the image to display the 3d boxes
    @param plot_3d: if the 3d boxes should be plot
    @return h_options[(0,2),4:8]: the birdseye view corners for the highest confidence box that is fully in view,non if no detection
    @return h_shifts[-1]: the detected rotation, None if no detection
    """
    t0 = time.time()
    dataset = LoadImagesDelivered(img_size=imgsz, stride=stride, limg=limg)
    detected_3d_boxes = []
    for img, im0 in dataset:
        img = torch.from_numpy(img).to(device)
        img = img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():  # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()
        # Apply NMS
        pred = non_max_suppression(
            pred,
            opt.conf_thres,
            opt.iou_thres,
            classes=opt.classes,
            agnostic=opt.agnostic_nms,
        )
        t3 = time_synchronized()
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    label = f"{names[int(cls)]} {conf:.2f}"
                    # TODO REMOVE  next line and indent once chair no longer an object
                    if not label[0:3] == "cha":
                        if calc_3d:
                            corners_3d, boundry, shifts = convert_2d_3d(
                                xyxy, im0, label
                            )
                        if view_img:  # Add bbox to image
                            plot_one_box(
                                xyxy,
                                im0,
                                label=label,
                                color=colors[int(cls)],
                                line_thickness=1,
                            )
                        if calc_3d and not boundry:
                            for options in corners_3d:
                                # plot_3d_box(corners_3d, im0, p_matrix, label=label,color=colors[int(cls)], line_thickness=1)
                                if view_img and plot_3d:  # Add bbox to image
                                    plot_3d_box(
                                        options,
                                        im0,
                                        p_matrix,
                                        label=label,
                                        color=[
                                            random.randint(0, 255),
                                            random.randint(0, 255),
                                            random.randint(0, 255),
                                        ],
                                        line_thickness=1,
                                    )
                                    if show:
                                        cv2.imshow("Detections", im0)
                                        cv2.waitKey(1)  # 1 millisecond

                                h_shifts = shifts
                                h_options = options
                                detec_dict = {
                                    "label": label[0:-5],
                                    "birds_eye": h_options[(0, 2), 4:8],
                                    "rotation": h_shifts[-1],
                                    "conf": float(label[-5:]),
                                    "boundry": boundry,
                                }
                                detected_3d_boxes.append(detec_dict)

                                if "localization" in node:
                                    rospy.Publisher(
                                        Topics.IMAGE_BB_WS.value, Image, queue_size=10
                                    ).publish(
                                        CvBridge().cv2_to_imgmsg(im0, "passthrough")
                                    )
                                else:
                                    rospy.Publisher(
                                        Topics.IMAGE_BB_MOVEABLE.value,
                                        Image,
                                        queue_size=10,
                                    ).publish(
                                        CvBridge().cv2_to_imgmsg(im0, "passthrough")
                                    )
    # if we did not detect any boxes
    if not detected_3d_boxes:
        return {}
    # the detection is split between workstations and movable objects, if one is present the other isn't
    elif detected_3d_boxes[0]["label"][0:-2] == "workstation":
        # for the worstations we will simply use the one with the highest confidence as the location
        highest_conf = 0
        for i, detection in enumerate(detected_3d_boxes):
            if highest_conf < detection["conf"]:
                highest_conf = detection["conf"]
                index_highest_conf = i
        return detected_3d_boxes[i]
    else:
        # for the moveable boxes we use the highest conf for each type, possibly using a preference for the larger face
        # TODO Needs testing which is better
        # we currently only work with one object per class, if you want to use more this would need to be expanded to check ...
        # wether two objects are on the same space or not
        best_in_class_detection = []
        # determining which detection of each class was best
        highest_conf_box = 0
        highest_conf_klapp = 0
        highest_conf_hocker = 0
        highest_conf_robotino = 0
        index_highest_conf_box = -1
        index_highest_conf_klapp = -1
        index_highest_conf_hocker = -1
        index_highest_conf_robotino = -1
        for i, detection in enumerate(detected_3d_boxes):
            if detection["label"][0:-2] in ["box", "sbox"]:
                # TODO here we could be checking if the object we are detection is the large face and give it for example a 0.1 improvement in conf
                if highest_conf_box < detection["conf"]:
                    highest_conf_box = detection["conf"]
                    index_highest_conf_box = i
            if detection["label"][0:-2] in ["klappbox", "sklappbox"]:
                if highest_conf_klapp < detection["conf"]:
                    highest_conf_klapp = detection["conf"]
                    index_highest_conf_klapp = i
            if detection["label"][0:-2] in ["hocker"]:
                if highest_conf_hocker < detection["conf"]:
                    highest_conf_hocker = detection["conf"]
                    index_highest_conf_hocker = i
            if detection["label"][0:-2] in ["robotino"]:
                if highest_conf_robotino < detection["conf"]:
                    highest_conf_robotino = detection["conf"]
                    index_highest_conf_robotino = i
        # if a box or klapp was detected, add the best one to detections
        if index_highest_conf_box != -1:
            box_detection = detected_3d_boxes[index_highest_conf_box]
            box_detection["label"] = "box"
            best_in_class_detection.append(box_detection)
        if index_highest_conf_klapp != -1:
            klapp_detection = detected_3d_boxes[index_highest_conf_klapp]
            klapp_detection["label"] = "klapp"
            best_in_class_detection.append(klapp_detection)
        if index_highest_conf_hocker != -1:
            hocker_detection = detected_3d_boxes[index_highest_conf_hocker]
            hocker_detection["label"] = "hocker"
            # TODO uncomment this when using the hocker
            best_in_class_detection.append(hocker_detection)
        if index_highest_conf_robotino != -1:
            robotino_detection = detected_3d_boxes[index_highest_conf_robotino]
            robotino_detection["label"] = "robotino"
            # TODO uncomment this when using the hocker
            best_in_class_detection.append(robotino_detection)
        return best_in_class_detection

    # if detected_3d_boxes
    # if highest_conf:
    #     highest_conf = 0
    #     if highest_conf< float(label[-5:]):
    #         highest_conf = float(label[-5:])
    #     return h_options[(0,2),4:8], h_shifts[-1]


def detect_args_parse(args):
    """add some args that are normally keept the same to the args
    @param calc_3d: if the 3d boxes should be calculated
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights", nargs="+", type=str, default="yolov7.pt", help="model.pt path(s)"
    )
    parser.add_argument(
        "--source", type=str, default="inference/images", help="source"
    )  # file/folder, 0 for webcam
    parser.add_argument(
        "--img-size", type=int, default=640, help="inference size (pixels)"
    )
    parser.add_argument(
        "--conf-thres", type=float, default=0.25, help="object confidence threshold"
    )
    parser.add_argument(
        "--iou-thres", type=float, default=0.45, help="IOU threshold for NMS"
    )
    parser.add_argument(
        "--device", default="cpu", help="cuda device, i.e. 0 or 0,1,2,3 or cpu"
    )
    parser.add_argument("--view-img", action="store_true", help="display results")
    parser.add_argument("--save-txt", action="store_true", help="save results to *.txt")
    parser.add_argument(
        "--save-conf", action="store_true", help="save confidences in --save-txt labels"
    )
    parser.add_argument(
        "--nosave", action="store_true", help="do not save images/videos"
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        type=int,
        help="filter by class: --class 0, or --class 0 2 3",
    )
    parser.add_argument(
        "--agnostic-nms", action="store_true", help="class-agnostic NMS"
    )
    parser.add_argument("--augment", action="store_true", help="augmented inference")
    parser.add_argument("--update", action="store_true", help="update all models")
    parser.add_argument(
        "--project", default="runs/detect", help="save results to project/name"
    )
    parser.add_argument("--name", default="exp", help="save results to project/name")
    parser.add_argument(
        "--exist-ok",
        action="store_true",
        help="existing project/name ok, do not increment",
    )
    parser.add_argument("--no-trace", action="store_true", help="don`t trace model")
    opt, _ = parser.parse_known_args()
    opt = parser.parse_args(args)
    opt.nosave = True
    opt.view_img = True
    opt.save_txt = False
    # print(opt)
    # check_requirements(exclude=('pycocotools', 'thop'))

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ["yolov7.pt"]:
                conf = load_model_and_conf(opt)
                strip_optimizer(opt.weights)
        else:
            conf = load_model_and_conf(opt)
            return conf


def get_conf_and_model(weights="yolov7/weights/ws_tiny5.pt"):
    """the most frequently changed arguments are here, others above
    loads the model and returns it with its' parameters
    @return conf: the model and its' args
    """
    args = []

    # args.extend(['--weights', 'yolov7_robo_first.pt'])
    args.extend(["--weights", weights])
    args.extend(["--conf", "0.50", "--view-img", "--no-trace"])
    args.extend(["--img-size", "640"])

    conf = detect_args_parse(args)
    return conf


if __name__ == "__main__":
    print("use other to start")
