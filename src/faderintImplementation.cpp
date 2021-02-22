/*******************************************************************************************************************
*   
*   Assignment 1: Determine the 2D position of a mobile robot using three-point triangulation
*
*   This is the implementation file.
*   For documentation, please see the application file
*
*   David Vernon
*   16 February 2021
*
*   Audit Trail
*   -----------
*
*
*
*   -----------
*
*   NOTE: this sample program only implements the input and output functionality.
*         For convenience, it also has a function to compute the intersection of two circles.
*         It remains to write the code to determine the 2D position using three-point triangulation.
*
*         Make sure to customize this file by replacing studentid with your own student ID in the #include directive
*       
*         Make sure you have done the following
*
*         - created a package named assignment1
*
*         - created a subdirectory named data in the package
*           and that you have created the input file assignment1Input.txt there
*
*         - created the interface file studentid.h in the include/assignment1 subdirectory
*
*         - created the application source file studentidApplication.cpp
*           and the implementation source file  studentidImplementation.cpp
*           in the src subdirectory
*    
*   David Vernon, 16 February 2021
*
*******************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>

#include <assignment1/faderint.h> // replace studentid with your own student ID

/*******************************************************************************************************************
*   Triangulation of robot position (x, y) given locations of three landmarks, (x1, y1), (x2, y2), and (x3, y3)
*   and angle subtended by landmarks 1 and 2 and angle subtended by landmarks 2 and 3 
*******************************************************************************************************************/

/* circle_circle_intersection() 
 * Determine the points where 2 circles in a common plane intersect.
 *
 * int circle_circle_intersection(
 *                                // center and radius of 1st circle
 *                                double x0, double y0, double r0,
 *                                // center and radius of 2nd circle
 *                                double x1, double y1, double r1,
 *                                // 1st intersection point
 *                                double *xi, double *yi,              
 *                                // 2nd intersection point
 *                                double *xi_prime, double *yi_prime)
 *
 * This is a public domain work. 3/26/2005 Tim Voght
 *
 * http://paulbourke.net/geometry/circlesphere/tvoght.c
 */


int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double *xi, double *yi,
                               double *xi_prime, double *yi_prime)
{
  double a, dx, dy, d, h, rx, ry;
  double x2, y2;

  /* dx and dy are the vertical and horizontal distances between
   * the circle centers.
   */
  dx = x1 - x0;
  dy = y1 - y0;

  /* Determine the straight-line distance between the centers. */
  //d = sqrt((dy*dy) + (dx*dx));
  d = hypot(dx,dy); // Suggested by Keith Briggs

  /* Check for solvability. */
  if (d > (r0 + r1))
  {
    /* no solution. circles do not intersect. */
    return 0;
  }
  if (d < fabs(r0 - r1))
  {
    /* no solution. one circle is contained in the other */
    return 0;
  }

  /* 'point 2' is the point where the line through the circle
   * intersection points crosses the line between the circle
   * centers.  
   */

  /* Determine the distance from point 0 to point 2. */
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  /* Determine the coordinates of point 2. */
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  /* Determine the distance from point 2 to either of the
   * intersection points.
   */
  h = sqrt((r0*r0) - (a*a));

  /* Now determine the offsets of the intersection points from
   * point 2.
   */
  rx = -dy * (h/d);
  ry = dx * (h/d);

  /* Determine the absolute intersection points. */
  *xi = x2 + rx;
  *xi_prime = x2 - rx;
  *yi = y2 + ry;
  *yi_prime = y2 - ry;

  return 1;
}


/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


void display_error_and_exit(char error_message[]) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void prompt_and_exit(int status) {
   printf("Press any key to terminate the program ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}

void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

