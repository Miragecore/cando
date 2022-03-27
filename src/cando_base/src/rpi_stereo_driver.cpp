
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "cando_base/rpi_stereo_driver.hpp"
 
namespace rpi_stereo_cam
{

  RpiStereoCamDriver::RpiStereoCamDriver()
  {
  }

  RpiStereoCamDriver::~RpiStereoCamDriver() 
  {
  }

  int RpiStereoCamDriver::init()
  {
    int fd = open("/dev/i2c-1", O_RDWR);
    if(!fd){
      return -1;
    }
    i2c_state.device_handle = fd;

    initGPIO();

    init_camera(capture_);

    return 0;
  }

  void RpiStereoCamDriver::set_width(int width_)
  {
    //cature_->set();
    width = width_;
  }

  void RpiStereoCamDriver::set_height(int height_)
  {
    //capture_->set();
    height = height_;
  }

  int RpiStereoCamDriver::get_width()
  {
    return width;
  }

  int RpiStereoCamDriver::get_height()
  {
    return height;
  }

  void RpiStereoCamDriver::read_left(cv::OutputArray left_)
  {
    choose_channel(LEFT_CAM);
    capture_.grab();
    capture_.grab();
    capture_.grab();
    capture_.grab();
    capture_.read(left_);
  }

  void RpiStereoCamDriver::read_right(cv::OutputArray right_)
  {
    choose_channel(RIGHT_CAM);
    capture_.grab();
    capture_.grab();
    capture_.grab();
    capture_.grab();
    capture_.read(right_);
  }

  int RpiStereoCamDriver::initGPIO()
  {
    wiringPiSetup() ;
    int io_count = sizeof(arducam_adapter_board[0].gpios)
                    /sizeof(arducam_adapter_board[0].gpios[0]);
    for(int i = 0 ; i < io_count ; i++){
      pinMode (arducam_adapter_board[0].gpios[i].gpio, OUTPUT);
    }
    return 0;
  }

  int  RpiStereoCamDriver::camera_detect()
  {
     int i = 0;
     uint16_t sensorID = 0;
     
     for(i = 0; i< (int)(sizeof(arducam_sensor_list)/sizeof(arducam_sensor_list[0])); i++)
     {
         sensorID = 0;
         i2c_state.chip_address =arducam_sensor_list[i]. slave_id;
         i2c_state.register_address =arducam_sensor_list[i].sensor_id_addr; //read hight id
         i2c_read_reg(&i2c_state); sensorID = i2c_state.register_value;
         i2c_state.register_address =arducam_sensor_list[i].sensor_id_addr+1; //read low id
         i2c_read_reg(&i2c_state); sensorID = sensorID<<8 | i2c_state.register_value;
         if(sensorID == arducam_sensor_list[i].sensor_id_val){
             return sensorID;
         }
     }
      return 0;
  }

  int RpiStereoCamDriver::init_camera(cv::VideoCapture &cap)
  {
    //int access(const char *filename, int mode);
    char *i2c;
    int ret = 0;
    system("sudo modprobe bcm2835_v4l2");
    if(access("/dev/video0",0)){
        //printf("Please check your camera connection,then try again.\r\n");
        //exit(0);
      return -1;
    }

    sleep(0.1);

    int ch_count = sizeof(arducam_adapter_board)/sizeof(arducam_adapter_board[0]);

    for(int ch_id = 0; ch_id < ch_count;ch_id++)
    {  
      i2c = arducam_adapter_board[ch_id].i2c_command;
      system(i2c);

      int io_count = sizeof(arducam_adapter_board[ch_id].gpios)
                      /sizeof(arducam_adapter_board[ch_id].gpios[0]);

      for(int i = 0; i< io_count;i++ )
      {
        digitalWrite(arducam_adapter_board[ch_id].gpios[i].gpio,
                     arducam_adapter_board[ch_id].gpios[i].value);  
      }
      
      if(camera_detect())
      {
         //printf("Detected camera on channal:%s\r\n",arducam_adapter_board[ch_id].camera_id);
        cap.open(cv::CAP_V4L2);
        if (!cap.isOpened())
        {
            //printf("ERROR! Unable to open camera\r\n") ;
            return -1;
        } 

				cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
				cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
				

        cap.grab();
        system("v4l2-ctl -c exposure_time_absolute=500");
        //arducam_adapter_board[ch_id].camera_exist = 1;
        ret ++;
        sleep(1);

      } 
      //{
        //arducam_adapter_board[ch_id].camera_exist = 0;
          // printf("Camera is missing on channal:%s\r\n",arducam_adapter_board[ch_id].camera_id); 
      //} 
    }
    //printf("Start preview...\r\n");
    return ret;
  }

  int RpiStereoCamDriver::i2c_read_reg(I2C_RW_STATE *state)
  {
    //uint16_t val = 0;
    int err;
    uint8_t buf[2];
    uint8_t data[1];
    
    if (ioctl(state->device_handle, I2C_SLAVE_FORCE, state->chip_address) < 0)
    {
      //printf("Failed to set I2C address\n");
        return -1;
    }

    buf[0] = state->register_address >> 8;
    buf[1] = state->register_address;

    struct i2c_rdwr_ioctl_data msgset;
    struct i2c_msg msgs[2] = {
      {
         .addr = state->chip_address,
         .flags = 0,
         .len = 2,
         .buf = buf,
      },
      {
        .addr = state->chip_address,
        .flags = I2C_M_RD,
        .len = 1,
        .buf = data,
      },
    };
    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    err = ioctl(state->device_handle, I2C_RDWR, &msgset);

    if (err != (int)msgset.nmsgs){
          return -1;  
      }		
      state->register_value = data[0];
    return 0;
  }
  
  void RpiStereoCamDriver::choose_channel(int ch_id)
  {
    int io_count = sizeof(arducam_adapter_board[ch_id].gpios)
                    /sizeof(arducam_adapter_board[ch_id].gpios[0]);
    for(int i = 0; i< io_count; i++ )
    {
      digitalWrite(arducam_adapter_board[ch_id].gpios[i].gpio,
          arducam_adapter_board[ch_id].gpios[i].value);  
    }
  }
} //rpi_stereo_cam

