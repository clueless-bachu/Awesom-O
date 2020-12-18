/**
 * @file Planner.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "Planner.h"
/**
* @brief Constructor
* @param NodeHandle function
* @return None
*/
Planner::Planner(const ros::NodeHandle& n, bool useVision):
    nh_(n) {
    pub_goal_ = nh_.advertise<geometry_msgs::Pose2D>
                                ("/move_base_simple/goal", 1);
    sub_reached_flag_ = nh_.subscribe("/odom", 1,
                                &Planner::flagCallBack, this);
    this->useVision = useVision;
    std::cout<< this->useVision << std::endl;
    if (!this->useVision) {
        sub_ar_pos_ = nh_.subscribe("/threat", 1,
                                &Planner::threatCallback, this);
    } else {
        sub_ar_pos_ = nh_.subscribe("/ar_pose_marker", 1,
                                &Planner::aRCallback, this);
    }
    this->mission_complete_ = false;
    this->num_targets_ = 3;

    std::vector<std::vector<double>> pwaypoints = {
        {-3, 3},
        {-3, 2},
        {-3, 1},
        {-3, 0},
        {-3, -1},
        {-3, -2},
        {-3, -3},
        {-2, -3},
        {-2, -2},
        {-2, -1},
        {-2, 0},
        {-2, 1},
        {-2, 2},
        {-2, 3},
        {-1, 3},
        {-1, 2},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {-1, -2},
        {-1, -3},
        {0, -3},
        {0, -2},
        {0, -1},
        {0, 0},
        {0, 1},
        {0, 2},
        {0, 3},
        {1, 3},
        {1, 2},
        {1, 1},
        {1, 0},
        {1, -1},
        {1, -2},
        {1, -3},
        {2, -3},
        {2, -2},
        {2, -1},
        {2, 0},
        {2, 1},
        {2, 2},
        {2, 3},
        {3, 3},
        {3, 2},
        {3, 1},
        {3, 0},
        {3, -1},
        {3, -2},
        {3, -3},
    };
    for (auto i : pwaypoints) {
        geometry_msgs::Pose2D temp;
        temp.x = i[0];
        temp.y = i[1];
        this->waypoints_.push(temp);
    }
    ROS_INFO("Planner node initilized!");
    // this->run();
}

/**
* @brief Destructor
* @param None
* @return None
*/
Planner::~Planner() {  // done
}

void Planner::threatCallback(const geometry_msgs::Pose2D::ConstPtr &data) {
    if (detected_targets_.find(data->theta) == detected_targets_.end()) {
        this->detected_targets_.insert(data->theta);
        this->targets_queue_.push(*data);
    }
}
/**
* @brief ARCallback method
* @param Artag custom msg
* @return None
*/
void Planner::aRCallback
(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {  // done
    // if message not empty,
    if (msg == NULL) {
        return;
    }
    // run for loop over every artag marker
    for (int i = 0; i < msg->markers.size(); i++) {
    // if id not in set, push world coordinate of tag to queue
        if (!detected_targets_.count(static_cast<int>(msg->markers[i].id))
                                    && msg->markers[i].id != 0) {
            tf::StampedTransform transform;
            try {
                ros::Time now = ros::Time::now();
                std::string x = "ar_marker_" +
                 std::to_string(msg->markers[i].id);
                // std::cout<<(x);
                // ROS_INFO_STREAM(x);
                listener_.waitForTransform("odom", x,
                ros::Time(0), ros::Duration(10.0));
                listener_.lookupTransform("/odom", "/" + x,
                ros::Time(0), transform);
                geometry_msgs::Pose2D global_position;
                global_position.x = transform.getOrigin().x();
                global_position.y = transform.getOrigin().y();
                // ROS_INFO_STREAM(global_position.x<<","<<global_position.y);
                targets_queue_.push(global_position);
                detected_targets_.insert(msg->markers[i].id);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
    }
}

/**
* @brief run method which runs an infinite while loop 
* @param vector<vector<float>>
* @return None
*/
void Planner::run(bool TEST) {
    ROS_INFO("Planner node initilized!");
    int counter = 0;
    geometry_msgs::Pose2D waypoint;
    std::vector<geometry_msgs::Pose2D> local_waypoints;
    while (nh_.ok() && !mission_complete_ &&
     (!waypoints_.empty() || local_waypoints.size()!= 0)) {
        counter++;
        if (TEST == true && counter < 10)
            return;

        if (local_waypoints.size() > 0
                && sqrt(pow((x_pos - local_waypoints[0].x), 2) +
                pow((y_pos - local_waypoints[0].y), 2)) <0.5) {
            ROS_INFO("Waypoint reached:");
            std::cout<< local_waypoints[0].x << ", " << local_waypoints[0].y
                        <<std::endl;
            ROS_INFO("waypoint removed");
            local_waypoints.erase(local_waypoints.begin());
        }
        if (!targets_queue_.empty()) {
            local_waypoints.insert(local_waypoints.begin(),
                     targets_queue_.front());
            targets_queue_.pop();
            ROS_INFO("Added threat to local list of waypoints");
        }
        pub_goal_.publish(local_waypoints[0]);
        ros::Duration(0.5).sleep();
        if (local_waypoints.size() == 0 && !waypoints_.empty()) {
            ROS_INFO("waypoint added to local list");
            local_waypoints.push_back(waypoints_.front());
            waypoints_.pop();
        }
        ros::spinOnce();
        if (detected_targets_.size() == num_targets_
                         && local_waypoints.size()== 0) {
                    mission_complete_ = true;
        }
    }
    if (mission_complete_) {
        ROS_INFO("MISSION COMPLETE! : Found all targets.");
    } else {
        ROS_INFO("MISSION FAILED! : All targets were not found during search.");
    }
}

/**
* @brief run method which runs an infinite while loop 
* @param vector<vector<float>>
* @return None
*/
void Planner::flagCallBack(const nav_msgs::Odometry::ConstPtr &data) {
    x_pos = data->pose.pose.position.x;
    y_pos = data->pose.pose.position.y;
}
