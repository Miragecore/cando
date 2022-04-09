import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
import easydict
import numpy as np
import easydict
import torch.nn as nn
import torchvision.transforms as transforms

import cv2
from cv_bridge import CvBridge, CvBridgeError

from .AnyNet.models.anynet import AnyNet

class AnyNetDisparity(Node):

  def __init__(self):
    super().__init__('anynet_disparity')

    self.logger = self.get_logger()

    args = easydict.EasyDict({"init_channels": 1 , 
                      "maxdisplist": [12,3,3],  
                      "spn_init_channels" : 8,
                      "nblocks" : 2,
                      "layers_3d" : 4,
                      "channels_3d" : 4,
                      "growth_rate" : [4,1,1],
                      "with_spn" : False})

    self.declare_parameter('pretrained', 'config/checkpoint/kitti2015/checkpoint.tar')
    self.pretrained = self.get_parameter('pretrained').value
    self.logger.info("pretrained model file : " + self.pretrained);
    self.add_on_set_parameters_callback(self.parameters_callback)

    self.tf = transforms.ToTensor()

    self.model = AnyNet(args)
    checkpoint = torch.load(self.pretrained, map_location=torch.device('cpu'))
    self.model.load_state_dict(checkpoint['state_dict'], strict=False)

    self.bridge = CvBridge()

    self.publisher_ = self.create_publisher(Image, "/disparity" ,10)

    left_rect_sub = message_filters.Subscriber(self, Image, "/left/image_rect")
    right_rect_sub = message_filters.Subscriber(self, Image, "/right/image_rect")

    ts = message_filters.ApproximateTimeSynchronizer([left_rect_sub, right_rect_sub], 10, 0.1, allow_headerless=True)

    ts.registerCallback(self.callback)

  def normalize8(I):
    mn = I.min()
    mx = I.max()

    mx -= mn

    I = ((I - mn)/mx) * 255
    return I.astype(np.uint8)

  def callback(self, left, right):
    try:
      limg = self.bridge.imgmsg_to_cv2(left, "bgr8")
      rimg = self.bridge.imgmsg_to_cv2(right, "bgr8")
    except CvBridgeError:
      self.logger.error("cv bridge error")

    # H x W x C
    limg = cv2.cvtColor(limg, cv2.COLOR_BGR2RGB)
    rimg = cv2.cvtColor(rimg, cv2.COLOR_BGR2RGB)

    ltensor = self.tf(limg).unsqueeze(0)
    rtensor = self.tf(rimg).unsqueeze(0)

    # 1 x C x H x W
    result = self.model(ltensor, rtensor)
    
    stage_indx = 2
   
    # 1 x C x H x W 
    result = result[stage_indx].detach().numpy()
    result = result.squeeze(0)
    # H x W , C = 1, so removed
    #result = result.transpose(1,2,0)
    result = self.normalize8(result)
    # H x W x C
    result = result.astype(np.uint8).copy()
    result = self.bridge.cv2_to_imgmsg(result, encoding="mono8") 

    self.publisher_.publish(result)

  def parameters_callback(self, params):
    success = False
    for param in params:
      if param.name == "pretrained":
        if param.type_ == Parameter.Type.STRING:
          self.pretrained = param.value
          self.logger.info("pretrained file changed : " + self.pretrained)
          checkpoint = torch.load(self.pretrained, map_location=torch.device('cpu'))
          self.model.load_state_dict(checkpoint['state_dict'], strict=False)
    return SetParametersResult(successful=success)
'''
        if param.name == "battery_percentage_warning":
            if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                if param.value >= 0.0 and param.value < 100.0:
                    success = True
                    self.battery_percentage_warning_ = param.value
        if param.name == "simulation_mode":
            if param.type_ == Parameter.Type.BOOL:
                success = True
                self.simulation_mode_ = param.value
'''

def main(args=None):
  rclpy.init(args=args)

  anetDisparity = AnyNetDisparity()

  rclpy.spin(anetDisparity)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  anetDisparity.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main() 
