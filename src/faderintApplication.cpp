/*******************************************************************************************************************
*  
*   Assignment 1: Determine the 2D position of a mobile robot using three-point triangulation, given:
*
*   - the locations of three landmarks: (x1, y1), (x2, y2), and (x3, y3)
*   - the angle alpha1 subtended by landmarks 1 and 2 
*   - the angle alpha2 subtended by landmarks 2 and 3
*
*   Angles are given in degrees.
*
*   The coordinates of the three landmarks and two angles are read from assignment1Input.txt
*
*   This file contains a sequence of input datasets, one set per line.  
*
*   For each input dataset, the robot coordinates  (xr, yr) are written to the output file assignment1Output.txt
*
*   Sample Input
*   1.0 0.0  0.5 0.5  0.0 1.0  45.0 45.0
*   1.5 5.0  6.0 5.5  8.0 1.0  60.0 50.0
*
*   Sample Output
*   0.000, 0.000 
*   1.901, 2.259 
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
*
*******************************************************************************************************************/

#include <assignment1/faderint.h> // replace studentid with your own student ID

main(int argc, char **argv) {

   bool debug = true;

   FILE *fp_in;                    
   FILE *fp_out;                    
   int end_of_file; 
   std::string          packagedir;
   char                 path[MAX_FILENAME_LENGTH];
   char                 input_filename[MAX_FILENAME_LENGTH]            = "assignment1Input.txt";
   char                 output_filename[MAX_FILENAME_LENGTH]           = "assignment1Output.txt";
   char                 path_and_input_filename[MAX_FILENAME_LENGTH]   = "";
   char                 path_and_output_filename[MAX_FILENAME_LENGTH]  = "";
      
   /******************************************************************************

   Triangulation variables

   *******************************************************************************/

   /* initial position for demo */

   double x1, y1;                  // initialized from input file
   double x2, y2;                  // initialized from input file
   double x3, y3;                  // initialized from input file
   double alpha1, alpha2;          // initialized from input file
   double xr, yr;                  // robot position; computed by this program


   /*******************************************************************************

   Read the landmark coordinates and the angles from the input file assignment1Input.txt

   ********************************************************************************/

   /* construct the full path and filename */
   
   packagedir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
 
   if (debug) cout << "Package directory: " << packagedir << endl;

   strcat(path_and_input_filename, packagedir.c_str());  
   strcat(path_and_input_filename, "/data/"); 
   strcat(path_and_input_filename, input_filename);

   if (debug) printf("Input file is  %s\n",path_and_input_filename);

   /* open the input file */
   
   if ((fp_in = fopen(path_and_input_filename,"r")) == 0) {
      printf("Error: can't open %s\n",path_and_input_filename);
      prompt_and_exit(1);
   }

   /* open the output file */

   strcat(path_and_output_filename, packagedir.c_str());   // construct the full path and filename
   strcat(path_and_output_filename, "/data/"); 
   strcat(path_and_output_filename, output_filename);

   if (true || debug) printf("Output file is %s\n",path_and_output_filename);

   if ((fp_out = fopen(path_and_output_filename,"w")) == 0) {
      printf("Error can't open %s\n",path_and_output_filename);
      prompt_and_exit(0);
   }

   /* read data */
   
   end_of_file = fscanf(fp_in, "%lf %lf %lf %lf %lf %lf %lf %lf", &x1, &y1, &x2, &y2, &x3, &y3, &alpha1, &alpha2); // read the configuration filename
      
   while (end_of_file != EOF) {

      if (debug) {
	 printf("Triangulation: landmark coordinates and angles \n");
	 printf("%5.3f %5.3f\n",x1, y1);
         printf("%5.3f %5.3f\n",x2, y2);
         printf("%5.3f %5.3f\n",x3, y3);
	 printf("%5.3f\n",alpha1);
	 printf("%5.3f\n\n",alpha2);
      }

      /* compute the robot location by 

         1. identifying the centre and radius of two circles, each containing a pair of landmarks
         2. computing the intersection of the two circles, one of which gives the location of the robot

      */

      alpha1 = M_PI*alpha1/180;
      alpha2 = M_PI*alpha2/180;

      double beta1 = M_PI/2 - atan((y2-y1)/(x1-x2));
      double beta2 = M_PI/2 - atan((y3-y2)/(x2-x3));

      double d_1 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      double r_1 = 0.5 * d_1/sin(alpha1);
      double h_1 = 0.5 * d_1/tan(alpha1);
      double x_m1 = 0.5 * (x1 + x2);
      double y_m1 = 0.5 * (y1 + y2);
      double dx_1 = h_1 * cos(beta1);
      double dy_1 = h_1 * sin(beta1);
      double x_c1 = x_m1 - dx_1;
      double y_c1 = y_m1 - dy_1;

      double d_2 = sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
      double r_2 = 0.5 * d_2/sin(alpha2);
      double h_2 = 0.5 * d_2/tan(alpha2);
      double x_m2 = 0.5 * (x2 + x3);
      double y_m2 = 0.5 * (y2 + y3);
      double dx_2 = h_2 * cos(beta2);
      double dy_2 = h_2 * sin(beta2);
      double x_c2 = x_m2 - dx_2;
      double y_c2 = y_m2 - dy_2;

      double x_i1 = 0, y_i1 = 0;
      double x_i2 = 0, y_i2 = 0;

      circle_circle_intersection(x_c1, y_c1, r_1, x_c2, y_c2, r_2, &x_i1, &y_i1, &x_i2, &y_i2);


      xr  = x_i1;
      yr  = y_i1;

if ((x_i1 == x1 && y_i1 == y1) ||
    (x_i1 == x2 && y_i1 == y2) ||
    (x_i1 == x3 && y_i1 == y3)) {
     xr = x_i2;
     yr = y_i2;
    }
      

      /* ... */


      
      fprintf(fp_out, "robot location: %5.3f, %5.3f \n",xr, yr);

      /* read next dataset */

      end_of_file = fscanf(fp_in, "%lf %lf %lf %lf %lf %lf %lf %lf", &x1, &y1, &x2, &y2, &x3, &y3, &alpha1, &alpha2); // read the configuration filename

   } while (end_of_file != EOF);

   fclose(fp_out);

   if (debug) prompt_and_exit(0);
}
