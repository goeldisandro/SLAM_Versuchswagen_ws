#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <log4cxx/logger.h>

bool zero_orientation_set = false;

bool set_odom_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;
  uint8_t st = 1;
  uint8_t *start = &st;
  bool handshake = false;
  
// x and y translation and angle of the robot
  double x = 0.0;
  double y = 0.0;
  double theta = 0;

// x and y speed and angle of the robot
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

// old eoncoder values
  long oldEr  = 0;
  long oldEl  = 0;
  
  int count = 0;

  unsigned long time_o;

  double U = 0.633;   // circumference of a wheel
  double D = 0.561;    // distance between wheels
//double k = 0.999  ;       //>1 nach links korrigieren <1 nach rechts korrigieren

  tf::Quaternion orientation;
  tf::Quaternion zero_orientation;

  ros::init(argc, argv, "odom_node");

  log4cxx::LoggerPtr my_logger =
           log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(
           ros::console::g_level_lookup[ros::console::levels::Debug]);

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM1");
  //private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "odom");
  //private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "base_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "odom");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  //private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::ServiceServer service = nh.advertiseService("set_odom_zero_orientation", set_odom_zero_orientation);

  ros::Rate r(200); // 200 hz

  nav_msgs::Odometry odom_msg;


//  static tf::TransformBroadcaster tf_br;
//  tf::Transform transform;
//  transform.setOrigin(tf::Vector3(0,0,0));

  std::string input;
  std::string read;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
	
        if(ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
          input += read;
	  if(!handshake)
	  {
		ser.write(start,1);
		ROS_DEBUG("Start written odom");
		while (input.length() >= 6) // while there might be a complete package in input
		  {
		    //parse for data packets
		    data_packet_start = input.find("$\x03");
		    if (data_packet_start != std::string::npos)
		    {
		      ROS_DEBUG("found possible start of handshake odom packet at position %d", data_packet_start);
		      if ((input.length() >= data_packet_start + 6) && (input.compare(data_packet_start + 3, 3, "!\r\n") == 0))
		      {		
				ROS_DEBUG("odom debug Package found");
				int8_t deb_num = 0xff &(char)input[data_packet_start + 2];
				ROS_DEBUG("odom debug State: %i",deb_num);
				input.erase(0, data_packet_start + 6); // delete everything up to and including the processed packet
				if(deb_num == 10){
					handshake = true;
					ROS_DEBUG("odom handshake successfull");
					input.clear();				
				}
		      }
		      else
		      {
		        if (input.length() >= data_packet_start + 6)
		        {
		          input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
			  ROS_DEBUG("input lenght:%i  deleted up to %d", (int)input.size() ,data_packet_start);
		        }
		        else
		        {
		          // do not delete start character, maybe complete package has not arrived yet
		          input.erase(0, data_packet_start);
		        }
		      }		    
		    }
		    else
		    {
		    	input.clear();
		    }
		   }
	  }
	  else
	  {
		  while (input.length() >= 14) // while there might be a complete package in input
		  {
		    //parse for data packets
		    data_packet_start = input.find("$\x03");
		    if (data_packet_start != std::string::npos)
		    {
		      ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
		      if ((input.length() >= data_packet_start + 14) && (input.compare(data_packet_start + 12, 2, "\r\n") == 0))  //check if positions 12,13 exist, then test values
		      {
		        ROS_DEBUG("seems to be a real data package: long enough and found end characters");
		        // get quaternion values
		        long newEr = ((long)((input[data_packet_start + 2]&0xff)<<24) | (long)((input[data_packet_start + 3]&0xff)<<16) | (long)((input[data_packet_start + 4]&0xff)<<8) | (long)(input[data_packet_start + 5]&0xff));
		        long newEl = ((long)((input[data_packet_start + 6]&0xff)<<24) | (long)((input[data_packet_start + 7]&0xff)<<16) | (long)((input[data_packet_start + 8]&0xff)<<8) | (long)(input[data_packet_start + 9]&0xff));

		        if (!zero_orientation_set)
		        {
		          oldEr = newEr;
			  oldEl = oldEl;
			  x = 0.0;
	  		  y = 0.0;
			  theta = 0;
			  if(count >= 0){
		          	zero_orientation_set = true;
				ROS_INFO("Odom Zero Orientation Set.");
			  }
			  count++;                
			}

		        // get Odom Transform
			// calculates the difference to the old values
			long dEr = (newEr -oldEr);
			long dEl = (newEl -oldEl);
			//unsigned long dt = time_n -time_o;

			// if there's a difference it calculates the new position
	    
		  	theta += U*(dEr-dEl)/(D*2000);      // angle of the robot, 2000 are the ticks per wheel rotation
			if(theta > 6.283) theta=0;
			if(theta < 0) theta=6.283;
			x += U/2000*(dEr+dEl)/2*cos(theta); //calculate x value
			y += U/2000*(dEr+dEl)/2*sin(theta); //calculate y value	    
		
			ROS_DEBUG("x: %f", x);
	    
			// Write Encoder values for the next time
			oldEr = newEr;
			oldEl = newEl; 

		        uint8_t received_message_number = input[data_packet_start + 10];
		        ROS_DEBUG("received message number: %i", received_message_number);

		        if (received_message) // can only check for continuous numbers if already received at least one packet
		        {
		          uint8_t message_distance = received_message_number - last_received_message_number;
		          if ( message_distance > 1 )
		          {
		          }
		        }
		        else
		        {
		          received_message = true;
		        }
		        last_received_message_number = received_message_number;

		        // calculate measurement time
		        ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

		        // publish imu message
		        odom_msg.header.stamp = measurement_time;
		        odom_msg.header.frame_id = "odom";
		        odom_msg.child_frame_id = "base_link";

		        odom_msg.pose.pose.position.x = x;
	    		odom_msg.pose.pose.position.y = y;
	   		odom_msg.pose.pose.position.z = 0.0;
			odom_msg.pose.pose.orientation = odom_quat;

	    		odom_msg.twist.twist.linear.x = 0;
	    		odom_msg.twist.twist.linear.y = 0;
	    		odom_msg.twist.twist.angular.z = 0;
	    
	    		odom_pub.publish(odom_msg);


		        // publish tf transform
	//                if (broadcast_tf)
	//                {
	//                  transform.setRotation(differential_rotation);
	//                  tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
	//                }
		        input.erase(0, data_packet_start + 28);  //delete everything up to and including the processed packet
		      }
		      else
		      {
		        if (input.length() >= data_packet_start + 14)
		        {
		          input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
		        }
		        else
		        {
		          // do not delete start character, maybe complete package has not arrived yet
		          input.erase(0, data_packet_start);
		        }
		      }
		    }
		    else
		    {
		      // no start character found in input, so delete everything
		      input.clear();
		    }
		  }
		}
        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(57600);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
}
