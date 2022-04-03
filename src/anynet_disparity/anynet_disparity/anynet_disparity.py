import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String

import easydict
#import torchvision.transforms as transforms
import numpy as np
import easydict
import torch.nn as nn
import torch.nn.parallel

import cv2
from cv_bridge import CvBridge, CvBridgeError

from .Anynet.models.anynet import AnyNet

class AnyNetDisparity(Node):

  def __init__(self):
    super().__init__('anynet_disparity')

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
    self.add_on_set_parameters_callback(self.parameters_callback)

    self.model = AnyNet(args)
    checkpoint = torch.load(self.pretrained, map_location=torch.device('cpu'))
    self.model.load_state_dict(checkpoint['state_dict'], strict=False)

    self.bridge = CvBridge()

    self.publisher_ = self.create_publisher(Image, "/disparity" ,10)

    left_rect_sub = message_filters.Subscriber(self, Image, "/left/image_rect")
    right_rect_sub = message_filters.Subscriber(self, Image, "/right/image_rect")

    ts = message_filters.ApproximateTimeSynchronizer([left_rect_sub, right_rect_sub], 10, 0.1, allow_headerless=True)

    ts.registerCallback(self.callback)

  def callback(left, right):
    try:
      limg = self.bridge.imgmsg_to_cv2(left, "rgb8");
      rimg = self.bridge.imgmsg_to_cv2(right, "rgb8");
    except CvBridgeError:
      print(e)

    ltensor = torch.from_numpy(limg.astype(np.float32))
    rtensor = torch.from_numpy(rimg.astype(np.float32))
    
    result = self.model(ltensor, rtensor)
    result = result.cpu().numpy().astype(np.float32)
    result = bridge.cv2_to_imgmsg(result, encoding="passthrough") 

    self.publisher_.publish(result)

  def parameters_callback(self, params):
    success = False
    for param in params:
      if param.name == "pretrained":
        if param.type_ == Parameter.Type.STRING:
          self.pretrained = param.value
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
