#include <gtest/gtest.h>
#include <ros/ros.h>
#include <threatGen.h>
class ThreatGenTest
{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_pos_;
    ros::Subscriber sub_target_;

public:
    geometry_msgs::Pose2D set_point;
    int count = 0;
    ThreatGenTest(const ros::NodeHandle &n)
        : nh_(n)
    {
        sub_target_ = nh_.subscribe("/threat",
                                  1, &ThreatGenTest::ThreatCallback, this);
        pub_pos_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
        
    }

    void ThreatCallback(const geometry_msgs::Pose2D::ConstPtr &data)
    {
        count++;
        set_point.x = data->x;
        set_point.y = data->y;
    }

    void publish(nav_msgs::Odometry odom_msg, int max_count)
    {
        ros::Rate loop_rate(10);
        int counter = 1;
        while (ros::ok())
        {
            pub_pos_.publish(odom_msg);
            ros::spinOnce();
            loop_rate.sleep();
            counter += 1;
            if (counter > max_count)
                break;
        }
    }

};

TEST(ThreatGenTests, checkGenerate)
{
    ros::NodeHandle nh_;
    
    ThreatGenTest threatTest(nh_);
    ThreatGen threat(nh_, 3, 10, true);

    threat.generateTargets(true);
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;
    threatTest.publish(odom,3);
    ros::spinOnce();
    ros::Duration(5).sleep();
    EXPECT_GT(threatTest.count,0);
}