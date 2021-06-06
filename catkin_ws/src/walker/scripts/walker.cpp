/**
 * @file walker.cpp : This files initialises and executes the ros node named
 * walker
 * @author Divyam Garg (dgarg1@umd.edu)
 * @brief
 * @version 0.1
 * @date 2020-11-22
 * @copyright Copyright (c) 2020
 * @license MIT License
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 */

#include "../include/walker.hpp"

/**
 * @brief Construct a new Walker:: Walker object initialises twist message and
 * publish and subscribe topics
 *
 * @param n
 */
Walker::Walker(ros::NodeHandle n)
    : move_pub(n.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
      laser_sub(n.subscribe("/scan", 10, &Walker::laserCallBack, this)) {
  twist.linear.x = 0;
  twist.linear.z = 0;
  move_pub.publish(twist);
  ROS_INFO("Walker node initialized");
}
/**
 * @brief Destroy the Walker:: Walker object
 *
 */
Walker::~Walker() { ROS_INFO("Walker_node is shutting down"); }
/**
 * @brief callback function that waits for scan message and accordingly
 * publishes the velocity
 *
 * @param scan
 */
void Walker::laserCallBack(const sensor_msgs::LaserScan::ConstPtr &scan) {
  bool flag_check = false;
  // checking at -30, 0 and 30 degrees for object at a certain threshold value
  if (scan->ranges[0] <= 0.6 || scan->ranges[30] <= 0.6 ||
      scan->ranges[330] <= 0.6) {
    flag_check = true;
  }

  // publish angular velocity if the obstacle is within threshold value
  if (flag_check) {
    twist.linear.x = 0;
    twist.angular.z = 0.8;
    move_pub.publish(twist);
    ROS_WARN("Obstacle ahead, taking a turn");
  }
  // publish linear velocity if the obstacle is not within threshold value
  if (flag_check == false) {
    twist.linear.x = 0.5;
    twist.angular.z = 0.0;
    move_pub.publish(twist);
    ROS_INFO("No Obstacle ahead, moving forward");
  }
}
