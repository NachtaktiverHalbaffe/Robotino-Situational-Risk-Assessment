import yaml
from models.yolo import Model
import torch

# load full model, give to the model you want to strip here
model_normal = torch.load("/home/nachtaktiverhalbaffe/dev/catkin_ws/src/robotino/src/yolov7/weights/tiny10_hocker.pt")
# save state dict
torch.save(
    model_normal["model"].state_dict(),
    "/home/nachtaktiverhalbaffe/dev/catkin_ws/src/robotino/src/yolov7/statedict_tiny10_hocker.pt",
)


# # as test and demo (the same code has been placed in detect online):
# # load state dict
loaded_state_dict = torch.load(
    "/home/nachtaktiverhalbaffe/dev/catkin_ws/src/robotino/src/yolov7/weights/tiny10_hocker.pt"
)
# get hyperparamters, these are the same for both models
hyp_loc = "./yolov7/data/hyp.scratch.custom.yaml"
with open(hyp_loc) as f:
    hyp = yaml.load(f, Loader=yaml.SafeLoader)  # load hyps
# load cfg, this is different for each
### this is for workstations###
# cfg = "./yolov7/cfg/training/yolov7-tiny_robo_ws.yaml"
# nc = 2  # amount of classes, the same as in the cfg file
### this is for movable ###
cfg = "./yolov7/cfg/training/yolov7-tiny_robo_movable.yaml"
nc = 10
model = Model(cfg, ch=3, nc=nc, anchors=hyp.get("anchors")).to("cpu")
model.load_state_dict(loaded_state_dict)
print(model)
