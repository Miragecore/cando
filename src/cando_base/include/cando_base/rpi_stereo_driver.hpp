#ifndef RPI_STEREO_CAM_HPP
#define RPI_STEREO_CAM_HPP

#include <opencv2/opencv.hpp>

namespace rpi_stereo_cam
{
  const int LEFT_CAM = 0;
  const int RIGHT_CAM = 1;

  typedef struct{
      int device_handle;
      uint16_t chip_address;
      uint16_t register_address;
      uint16_t register_value;
  }I2C_RW_STATE;

  typedef struct{
   int slave_id;
   uint16_t sensor_id_addr; 
   uint16_t sensor_id_val;
   char* sensor_name;
  }SENSOR_TYPE;
  
  typedef struct{
      int gpio;
      int value;
  }GPIO_TYPE;

  typedef struct{
      //bool camera_exist;
      char channel_id;
      char *camera_id;
      char *i2c_command; 
      GPIO_TYPE gpios[3];
  }MULTI_ADAPTER_T;

  const MULTI_ADAPTER_T arducam_adapter_board[]={
      {
          .channel_id = 0,
          .camera_id = (char *)"A",
          .i2c_command = (char *)"i2cset -y 1 0x70 0x00 0x04",
          .gpios = {
              {   .gpio =  7,
                  .value = 0,
              },
              {   .gpio =  0,
                  .value = 0,
              },
              {   .gpio =  1,
                  .value = 1,
              }
          }
      },
      {
          .channel_id = 1,
          .camera_id =(char *)"B",
          .i2c_command = (char *)"i2cset -y 1 0x70 0x00 0x05",
          .gpios = {
              {   .gpio  = 7,
                  .value = 1,
              },
              {   .gpio  = 0,
                  .value = 0,
              },
              {   .gpio  = 1,
                  .value = 1,
              }
          }
      },
      {
          .channel_id = 2,
          .camera_id = (char *)"C",
          .i2c_command = (char *)"i2cset -y 1 0x70 0x00 0x06",
          .gpios = {
              {   .gpio  = 7,
                  .value = 0,
              },
              {   .gpio  = 0,
                  .value = 1,
              },
              {   .gpio  = 1,
                  .value = 0,
              }
          }
      },
      {
          .channel_id = 3,
          .camera_id = (char *)"D",
          .i2c_command = (char *)"i2cset -y 1 0x70 0x00 0x07",
          .gpios = {
              {   .gpio  = 7,
                  .value = 1,
              },
              {   .gpio  = 0,
                  .value = 1,
              },
              {   .gpio  = 1,
                  .value = 0,
              }
          }
      },
  };

  const SENSOR_TYPE arducam_sensor_list[] = {
      {   .slave_id = 0x10, 
          .sensor_id_addr = 0x0000,
          .sensor_id_val = 0x0219,
          .sensor_name =(char *)"imx219"
      },
      {
          .slave_id = 0x36, 
          .sensor_id_addr = 0x300A,
          .sensor_id_val = 0x5647,
          .sensor_name =(char *)"ov5647" 
      },
      {
          .slave_id = 0x1A, 
          .sensor_id_addr = 0x0016,
          .sensor_id_val = 0x0477,
          .sensor_name =(char *)"imx477" 
      }
  };
  //use 2 Channels (A, C) on 4 Channel adapter
  class RpiStereoCamDriver
  {
    public:

      RpiStereoCamDriver();

      ~RpiStereoCamDriver();

      int init();

      void set_width(int width_);
      void set_height(int height_);
      int get_width();
      int get_height();

      void read_left(cv::OutputArray left_);
      void read_right(cv::OutputArray right_);

    private:

      cv::VideoCapture capture_;

      int initGPIO();

      int camera_detect();

      int init_camera(cv::VideoCapture &cap);

      int i2c_read_reg(I2C_RW_STATE *state);

      void choose_channel(int ch_id);

      I2C_RW_STATE i2c_state;
      int i2c_handle;
      //assume same image size both camera
      int width;
      int height;

  };// class RpiStereoCamDrvier
}// namespace rpi_stereo_cam

#endif //RPI_STEREO_CAM_HPP

