#include "ros/ros.h"

#include "utils/media.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

int main(int argc, char **argv)
{
    av_register_all();

    ros::init(argc, argv, "gopro");

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;

    ros::NodeHandle nPriv("~");

    ros::Publisher imu_pub = nPriv.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher image_pub = nPriv.advertise<sensor_msgs::Image>("image", 1000);

    std::string path;
    nPriv.param("path", path, std::string());

    std::vector<std::string> path_list;
    nPriv.param("path_list", path_list, std::vector<std::string>());

    bool only_imu;
    nPriv.param("only_imu", only_imu, false);

    bool loop;
    nPriv.param("loop", loop, false);

    int frame_width = -1;
    int frame_height = -1;

    frame_width = 768;
    frame_height = 432;
    ros::Rate loop_rate(20);

    if (path_list.empty() && path.empty())
    {
        ROS_ERROR("You must supply either a path or path_list parameter");
        exit(1);
    }
    else if (path_list.empty())
    {
        path_list.push_back(path);
    }

    int media_index = 0;
    Media media(path_list[media_index]);
    media.set_frame_width(frame_width);
    media.set_frame_height(frame_height);
    media.open();

    enum Media::PacketType packet_type = media.next_packet();
    double start_pts = 0.0;
    double relative_pts = 0.0;
    double duration = 0.0;

    std::vector<std::pair<double, sensor_msgs::Imu>> imu_data;

    ROS_INFO("Now reading IMU");
    while (ros::ok())
    {
        // Skip to next IMU packet or end
        while (packet_type != Media::PacketType::IMU && packet_type != Media::PacketType::END)
        {
            packet_type = media.next_packet();
        }

        if (media.get_packet_type() == Media::PacketType::IMU)
        {
            relative_pts = media.get_pts();
            duration = media.get_duration();
            ROS_INFO("imu ts = %f", start_pts + relative_pts);

            std::vector<Eigen::Vector3f> accl_data;
            std::vector<Eigen::Vector3f> gyro_data;
            media.read_imu(accl_data, gyro_data);
            // ROS_INFO("pts = %f duration = %f", start_pts + relative_pts, duration);

            for (int gyro_idx = 0; gyro_idx < gyro_data.size(); gyro_idx++)
            {
                int accl_idx = gyro_idx * accl_data.size() / gyro_data.size();

                double delta_ts = ((double)gyro_idx) * duration / (double)gyro_data.size();
                double imu_ts = start_pts + relative_pts + delta_ts;

                // ROS_INFO("imu ts partial = %f", imu_ts);

                Eigen::Vector3f accl = accl_data[accl_idx];
                Eigen::Vector3f gyro = gyro_data[gyro_idx];

                sensor_msgs::Imu imu;

                imu.header.stamp.fromSec(imu_ts);

                // imu.linear_acceleration.x = -accl[1];
                // imu.linear_acceleration.y = accl[0];
                // imu.linear_acceleration.z = accl[2];
                imu.linear_acceleration.x = accl[0];
                imu.linear_acceleration.y = accl[1];
                imu.linear_acceleration.z = accl[2];

                // imu.angular_velocity.x = -gyro[1];
                // imu.angular_velocity.y = gyro[0];
                // imu.angular_velocity.z = gyro[2];
                imu.angular_velocity.x = gyro[0];
                imu.angular_velocity.y = gyro[1];
                imu.angular_velocity.z = gyro[2];

                // ROS_INFO("imu accl %f %f %f", accl[0], accl[1], accl[2]);
                // ROS_INFO("imu gyro %f %f %f", gyro[0], gyro[1], gyro[2]);

                if (only_imu)
                {
                    imu_pub.publish(imu);
                }
                else
                {
                    imu_data.push_back(std::make_pair(imu_ts, imu));
                }
            }
        }
        else if (packet_type == Media::PacketType::END)
        {
            media_index++;
            if (media_index < path_list.size())
            {
                start_pts = start_pts + relative_pts + duration;
                media = Media(path_list[media_index]);
                media.set_frame_width(frame_width);
                media.set_frame_height(frame_height);
                media.open();
                packet_type = media.next_packet();
            }
            else
            {
                break;
            }
        }

        packet_type = media.next_packet();

        ros::spinOnce();
    }

    if (only_imu)
    {
        exit(0);
    }

    media_index = 0;
    media = Media(path_list[media_index]);
    media.set_frame_width(frame_width);
    media.set_frame_height(frame_height);
    media.open();
    packet_type = media.next_packet();
    start_pts = 0.0;
    relative_pts = 0.0;
    duration = 0.0;

    std::size_t imu_idx = 0;

    ROS_INFO("Now reading video");

    while (ros::ok())
    {
        // Skip to next VIDEO packet or end
        while (packet_type != Media::PacketType::VIDEO && packet_type != Media::PacketType::END)
        {
            packet_type = media.next_packet();
        }

        if (packet_type == Media::PacketType::VIDEO)
        {
            relative_pts = media.get_pts();
            duration = media.get_duration();

            if (media.read_frame())
            {
                int size = media.get_frame_linesize() * media.get_frame_height();
                double video_ts = start_pts + relative_pts;

                sensor_msgs::Image img;

                img.header.stamp.fromSec(video_ts);

                img.data = std::vector<uint8_t>(size);
                memcpy(img.data.data(), media.get_frame_data(), size);
                // img.encoding = sensor_msgs::image_encodings::RGB8;
                img.encoding = sensor_msgs::image_encodings::MONO8;
                img.width = media.get_frame_width();
                img.height = media.get_frame_height();
                img.step = media.get_frame_linesize();

                while (imu_idx < imu_data.size())
                {
                    std::pair<double, sensor_msgs::Imu> pair = imu_data[imu_idx];
                    if (pair.first < video_ts)
                    {
                        // ROS_INFO("Publish imu ts = %f", pair.first);
                        imu_pub.publish(pair.second);
                        imu_idx++;
                    }
                    else
                    {
                        break;
                    }
                }

                // ROS_INFO("Publish image ts = %f", video_ts);

                // loop_rate.sleep();
                image_pub.publish(img);
            }
            else
            {
                ROS_ERROR("Problem decoding frame, pts = %f", start_pts + relative_pts);
            }
        }
        else if (packet_type == Media::PacketType::END)
        {
            media_index++;
            if (loop && media_index >= path_list.size())
            {
                imu_idx = 0;
                media_index = 0;
                start_pts = 0.0;
                media = Media(path_list[media_index]);
                media.set_frame_width(frame_width);
                media.set_frame_height(frame_height);
                media.open();
                packet_type = media.next_packet();
            }
            else if (media_index < path_list.size())
            {
                start_pts = start_pts + relative_pts + duration;
                media = Media(path_list[media_index]);
                media.set_frame_width(frame_width);
                media.set_frame_height(frame_height);
                media.open();
                packet_type = media.next_packet();
            }
            else
            {
                break;
            }
        }

        packet_type = media.next_packet();

        ros::spinOnce();
    }

    return 0;
}
