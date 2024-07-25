#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "robot_msgs/sc_rc_msg.h"
#include "robot_msgs/robot_ctrl.h"
#include "robot_msgs/vision.h"
#include "robot_msgs/competition_info.h"
#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/**
 * @brief Robot Base Node
 *        Main Process is
 *        1. RECEIVING:
 *           Virtual Serial Comm -> Unpack and Get Protocol Data
 *           -> Convert to ROS Data -> ROS Publish
 *        2. SENDING:
 *           ROS Subscribe -> Get ROS Data -> Convert to Protocol Data
 *           -> Convert to Protocol Data and Pack -> Virtual Serial Comm
 */
double x = 0.0;
double y = 0.0;
double th = 0.0;
ros::Time current_time, last_time;

namespace robomaster
{
  class Robot
  {
  public:
    Robot(std::string device_path = "/dev/robomaster") : device_path_(device_path)
    {

      if (!(ROSInit() && CommInit()))
      {
        ros::shutdown();
      };
    }
    ~Robot()
    {
      if (recv_thread_.joinable())
      {
        recv_thread_.join();
      }
    }

    void navgation_ctrl_callback(const geometry_msgs::Twist &cmd_vel)
    {
      robot_ctrl.vx = cmd_vel.linear.x;
      robot_ctrl.vy = cmd_vel.linear.y;
      robot_ctrl.vw = cmd_vel.angular.z;
      uint16_t send_length = SenderPackSolve((uint8_t *)&robot_ctrl, sizeof(robot_ctrl_info_t),
                                             CHASSIS_CTRL_CMD_ID, send_buff_.get());
      device_ptr_->Write(send_buff_.get(), send_length);
      // ROS_INFO("Sending nav_ctrl msg");
    }

  private:
    bool ROSInit()
    {
      ros::NodeHandle nh;

      cmd_vel_sub_ = nh.subscribe("cmd_vel", 10, &Robot::navgation_ctrl_callback, this);
      chassis_info_pub_ = nh.advertise<std_msgs::Float32>("battery_voltage",1);
      current_time = ros::Time::now();
      last_time = ros::Time::now();

      return true;
    }
    bool CommInit()
    {

      device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 比特率115200

      if (!device_ptr_->Init())
      {
        return false;
      }

      recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
      send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);

      memset(&frame_receive_header_, 0, sizeof(frame_header_struct_t));
      memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));

      /** Specific Protocol Data Initialize here**/
      // memset(&summer_camp_info_, 0, sizeof(summer_camp_info_t));

      // Start recv thread
      recv_thread_ = std::thread(&Robot::RecvThread, this);
      return true;
    }

    void RecvThread()
    {

      int a = 0;
      int flag = 0;

      uint8_t last_len = 0;

      while (ros::ok())
      {
        last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, 128);

        while (flag == 0 && last_len == 1)
        {
          if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF))
          {
            flag = 1;
            SearchFrameSOF(recv_buff_.get(), a);
          }
          a++;
        }
        flag = 0;
        a = 0;
        ros::spinOnce(); 
        usleep(1);
      }
    }

    void SearchFrameSOF(uint8_t *frame, uint16_t total_len)
    {
      uint16_t i;
      uint16_t index = 0;
      int a = 0;

      for (i = 0; i < total_len;)
      {
        if (*frame == HEADER_SOF)
        {
          ReceiveDataSolve(frame);
          i = total_len;
        }
        else
        {
          frame++;
          i++;
        }
      }
    }

    uint16_t ReceiveDataSolve(uint8_t *frame)
    {
      uint8_t index = 0;
      uint16_t cmd_id = 0;

      if (*frame != HEADER_SOF)
      {
        return 0;
      }

      memcpy(&frame_receive_header_, frame, sizeof(frame_header_struct_t));
      index += sizeof(frame_header_struct_t);
      if ((!Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t))) || (!Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9)))
      {
        ROS_ERROR("CRC  EEROR!");
        return 0;
      }
      else
      {
        memcpy(&cmd_id, frame + index, sizeof(uint16_t));
        index += sizeof(uint16_t);
        // printf("id:%x\n", cmd_id);
        switch (cmd_id)
        {

        /** Write Your code here to get different types of data and publish them using ROS interface
         *
         *  Example:
         *
         *   case XXXX_CMD_ID:{
         *
         *    memcpy(&xxxx_info, frame + index, sizeof(xxxx_info_t))
         *    break;
         *
         *   }
         *
         */
        case CHASSIS_ODOM_CMD_ID:
        {
          ROS_INFO("ODOM info");
          // printf("get chassis_odom_msg");
          memcpy(&chassis_odom_info_, frame + index, sizeof(chassis_odom_info_t));

          voltage.data = chassis_odom_info_.voltage;

          chassis_info_pub_.publish(voltage);
        }
        break;

    

        default:
          break;
        }
        index += frame_receive_header_.data_length + 2;
        return index;
      }
    }

    uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
                             uint16_t cmd_id, uint8_t *send_buf)
    {

      uint8_t index = 0;
      frame_send_header_.SOF = HEADER_SOF;
      frame_send_header_.data_length = data_length;
      frame_send_header_.seq++;

      Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(frame_header_struct_t));

      memcpy(send_buf, &frame_send_header_, sizeof(frame_header_struct_t));

      index += sizeof(frame_header_struct_t);

      memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));

      index += sizeof(uint16_t);

      memcpy(send_buf + index, data, data_length);

      Append_CRC16_Check_Sum(send_buf, data_length + 9);

      return data_length + 9;
    }

  private:
    //! VCOM Data Receiving Thread (Tips: VCOM Sending part is in Each ROS Data Callback)
    std::thread recv_thread_;

    //! Device Information and Buffer Allocation
    std::string device_path_;
    std::shared_ptr<SerialDevice> device_ptr_;
    std::unique_ptr<uint8_t[]> recv_buff_;
    std::unique_ptr<uint8_t[]> send_buff_;
    const unsigned int BUFF_LENGTH = 512;

    //! Frame Information
    frame_header_struct_t frame_receive_header_;
    frame_header_struct_t frame_send_header_;

    /** @brief specific protocol data are defined here
     *         xxxx_info_t is defined in protocol.h
     */

    //! Receive from VCOM
    chassis_odom_info_t chassis_odom_info_;
    robot_ctrl_info_t robot_ctrl;
    std_msgs::Float32 voltage;

    // geometry_msgs::TransformStamped odom_tf_;//! ros chassis odometry tf
    geometry_msgs::TransformStamped odom_trans;
    bool start_point=false;

    nav_msgs::Odometry odom_;

    //! Send to VCOM

    /** @brief ROS data corresponding to specific protocol data are defined here
     *         You can use ROS provided message data type or create your own one
     *         More information please refer to
     *               http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
     */

    /** @brief ROS Subscription and Publication
     */


    ros::Subscriber cmd_vel_sub_;
    ros::Publisher chassis_info_pub_;
  };
}

#endif // ROBOMASTER_ROBOT_H
