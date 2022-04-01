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

from cv_bridge import CvBridge, CvBridgeError

from .Anynet.models.anynet import AnyNet

class AnyNetDisparity(Node):

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

  def __init__(self):
    super().__init__('anynet_disparity')

    args = easydict.EasyDict({"init_channels": 1 , 
                      "maxdisplist": [12,3,3],  
                      "spn_init_channels" : 8,
                      "nblocks" : 2,
                      "layers_3d" : 4,
                      "channels_3d" : 4,
                      "growth_rate" : [4,1,1],
                      "with_spn" : False,
                      "pretrained" : "config/checkpoint/kitti2015_ck/checkpoint.tar"})

    self.model = AnyNet(args)
    checkpoint = torch.load(args.pretrained, map_location=torch.device('cpu'))
    self.model.load_state_dict(checkpoint['state_dict'], strict=False)

    self.bridge = CvBridge()

    self.publisher_ = self.create_publisher(Image, "/disparity" ,10)

    left_rect_sub = message_filters.Subscriber(self, Image, "/left_rect")
    right_rect_sub = message_filters.Subscriber(self, Image, "/right_rect")

    ts = message_filters.ApproximateTimeSynchronizer([left_rect_sub, right_rect_sub], 10, 0.1, allow_headerless=True)

    ts.registerCallback(self.callback)


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
