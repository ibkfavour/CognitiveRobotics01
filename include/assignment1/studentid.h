/*******************************************************************************************************************
*   Assignment 1: Determine the 2D position of a mobile robot using three-point triangulation
*
*   This is the interface file.
*   For documentation, please see the application file
*
*   David Vernon
*   16 February 2021
*
*   Audit Trail
*   -----------
*
*
*   ===============================================================================================================
*   NOTE: this sample program only implements the input and output functionality.
*         For convenience, it also has a function to compute the intersection of two circles.
*         It remains to write the code to determine the 2D position using three-point triangulation.
*         David Vernon, 16 February 2021
*   ===============================================================================================================
*
*******************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

#define ROS_PACKAGE_NAME    "assignment1"
#define MAX_FILENAME_LENGTH 200

int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime);

void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);
