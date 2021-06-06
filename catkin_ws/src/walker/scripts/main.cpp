/**
 * @file main.cpp : This files initialises and executes the ros node named
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
 */

#include "../include/walker.hpp"
#include <ros/ros.h>

/**
 * @brief This main file executes ros node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  //> Initializing the node.
  ros::init(argc, argv, "walker");

  //> Creating Node handle.
  ros::NodeHandle n;

  //> Creating Walker walk object.
  Walker walk(n);

  //> Waiting for scan msg call back.
  ros::spin();

  return 0;
}
