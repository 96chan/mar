#define DEBUG 1
/*****************************
  Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#ifdef __APPLE__
#include <gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glui.h>
#endif
#include "aruco.h"
#include "functions.h"

#define PI 3.1415926535897932384626433832795
using namespace cv;
using namespace aruco;

//Enumeration for modes
enum Mode{
  Free,
  Grid,
};

// Global Variables 
std::map<int, std::string> lettermap;
string TheInputVideo;
string TheIntrinsicFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
MarkerDetector PPDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheUndInputImage,TheResizedImage;
CameraParameters TheCameraParams;
Size TheGlWindowSize;
bool TheCaptureFlag=true;
bool readIntrinsicFile(string TheIntrinsicFile,Mat & TheIntriscCameraMatrix,Mat &TheDistorsionCameraParams,Size size);
Mat mA, mB, mC, mD, mE, mF, mG, mH, mI; // Marker Types
Mat eA,eB,eD,eE,eF,eG,eAD,eDE,eEF,eFG,eGB,eBA; // Extended Marker for simulation 
Mat hAC,hDC,hBC,hEC,hFC,hGC,hAD,hDE,hEF,hFG,hGB,hBA;
int MarkerID[9]; // marker id such as A,B,C.. upto I
int file_id = 0; //screen capture numbering
int window_id;

// Pointer to the controls
GLUI *glui_button, *glui_subwin; 
GLUI_RadioGroup *linear_rg;
GLUI_RadioGroup *unit_rg;
GLUI_RadioGroup *simulate_rg;
GLUI_Checkbox *outline_cb;
GLUI_Checkbox *grid_cb;
GLUI_Checkbox *linear_cb;

int mode_grid=0;
int mode_linear=0;
int linear_type=3;
int outline_type=0;
int unit_type=0;
int simulate_type=3;
GLUI_Button *show_btn;
GLUI_Button *hide_btn;
GLUI_Button *capture_btn;
GLUI_Button *quit_btn;


// flags
bool imperialUnitFlag = false;
bool xflag = false;
bool yflag = false;
bool zflag = false;
bool aflag = false;
bool bflag = false;
bool cflag = false;
bool coin_flag = false;
bool outline_flag = false;
bool capture_flag = false;
bool line_flag =false;
bool grid_flag =false;
bool simulate_flag = false;
bool alias_flag = true;// true;
float M2CM = 100.0f;
float M2IN = 39.3700787f;

Mode mode;

void init(){
  mode = Free;
}
/************************************
 *
 *
 ************************************/
bool readArguments ( int argc,char **argv )
{
    if (argc!=4) {
        cerr<<"Invalid number of arguments"<<endl;
        cerr<<"Usage: (in.avi|live)  intrinsics.yml   size "<<endl;
        return false;
    }
    TheInputVideo=argv[1];
    TheIntrinsicFile=argv[2];
    TheMarkerSize=atof(argv[3]);
    return true;
}

void vButton(GLUI_Control* control){
    cout<<"control : " << control->get_id() <<endl;
  
    if(control->get_id() == 1){
            capture_flag = true;
            ScreenCapture();
            cout << "Screen Capture" <<endl;
    }
    else if(control->get_id() == 2){
            glui_button->hide();
            glui_subwin->show();
    }
    else if(control->get_id() == 3){
            glui_button->show();
            glui_subwin->hide(); 
    }
}
/************************************
 * Mode Selection using Touch
*************************************/
void vMenu(int value){
    cout << "mode_grid : " << mode_grid <<endl;
    cout << "mode_linear : " << mode_linear <<endl;
    cout << "simulate_type " << simulate_type <<endl;
    cout << "outline_type " << outline_type <<endl;
    cout << "unit_type " << unit_type <<endl;
    if(mode_grid==0){value =1;grid_flag=false;}
    else if(mode_grid==1){value =2;grid_flag=true;}
    if(outline_type==0) outline_flag =false;
    else if(outline_type==1) outline_flag =true;
    if(unit_type==0) imperialUnitFlag=false; 
    else if(unit_type==1) imperialUnitFlag =true;
    if(mode_linear==0) line_flag=false;
    else if(mode_linear==1) line_flag=true;

    if(simulate_type==0) {simulate_flag=true;aflag=true;bflag=false;cflag=false;}
    else if(simulate_type==1) {simulate_flag=true;aflag=false;bflag=true;cflag=false;}
    else if(simulate_type==2) {simulate_flag=true;aflag=false;bflag=false;cflag=true;}
    else if(simulate_type==3) {simulate_flag=false;aflag=false;bflag=false;cflag=false;}
     
    switch(value){
        case 1:
            mode = Free;
            if(line_flag){
            if(linear_type==0){xflag =true;yflag=false;zflag=false;}
            else if(linear_type==1){xflag =false;yflag=true;zflag=false;}
            else if(linear_type==2){xflag =false;yflag=false;zflag=true;}
            else if(linear_type==3){xflag =false;yflag=false;zflag=false;}
            }
           cout << "I am in the default free mode" << endl;
           break;
        case 2:
            mode = Grid;
            cout << "I am in the grid mode" << endl;
            break;
        default: break;
        }
    glutPostRedisplay();
}

/************************************
 *Mode Selection using Keyboard
 ************************************/
void vKeyboard(unsigned char key,int x,int y){
    if (key == 'g'){
    mode = Grid;
    cout << "I am in the grid mode" << endl;
  } else if (key == 'x'){
    mode = Free;
    xflag = !xflag;
    cout << "I am in the line mode2" << endl;
  }
  else if (key == 'y') {
     mode = Free;
     yflag = !yflag;
  }
  else if (key == 'z') {
     mode = Free;
     zflag = !zflag;
  } 
  else if (key == 'u'){
    imperialUnitFlag = !imperialUnitFlag;
  } 
  else {
    mode = Free;
    cout << "I am in the default free mode" << endl;
  }
}
void ScreenCapture(){
 time_t now = time(0);
 char* dt = ctime(&now);
 std::string filename = "/home/ischool/Documents/MathMAR/image/"+string(dt)+".png";
 
 cv::Mat img(720,1280,CV_8UC3);
 glPixelStorei(GL_PACK_ALIGNMENT,(img.step &3)?1:4);
 glPixelStorei(GL_PACK_ROW_LENGTH,img.step/img.elemSize());
 glReadPixels(0,0,img.cols,img.rows,GL_BGR_EXT,GL_UNSIGNED_BYTE,img.data);
 cv::Mat flipped(img);
 cv::flip(img,flipped,0);
 file_id ++;
 cv::imwrite(filename,img);
/*
 IplImage *srcimg_R = new IplImage(TheInputImage);
 char file_name[20];
 sprintf(file_name,"capture.bmp");
 cvSaveImage(file_name,srcimg_R);
*/
}

/************************************
 *
 ************************************/

void vMouse(int b,int s,int x,int y)
{
}

/************************************
 *
 ************************************/
float convertD(float distance){
   return imperialUnitFlag?(M2IN*distance):(M2CM*distance);
}
void convertDistance(float *distance){
  *distance = imperialUnitFlag?(M2IN**distance):(M2CM**distance);
}
string convertInt(int number){
  stringstream ss;
  ss << number;
  return ss.str();
}

/**********************************************************
 *Calculate the distance between two points expressed as cv matrices
 *Finished by deepak
 **************************************************************/
float calculateDistance(cv::Mat t0,cv::Mat t1,bool convert=true){
  
  float d[3];
  float distsquared = 0;
  float distance = 0;
  for (int i = 0; i < 3; i++){
    d[i] = (t0.at<float>(i,0) - t1.at<float>(i,0)); 
    distsquared = distsquared + d[i] * d[i];   
  }
  distance = sqrt(distsquared);
  //metric or imperial
  if (convert) {
  convertDistance(&distance);
  }
  return distance;
}
int calculatePerimeter(vector<cv::Point2f> centers) {
  int perimeter = 0;
  perimeter += floor(calculateDistance(mA,mB)+0.5);
  switch(centers.size()){
    default:
         break;
    case 3:
         perimeter += floor(calculateDistance(mB,mC)+0.5) + floor(calculateDistance(mA,mC)+0.5);
         break;
    case 4: 
         perimeter += floor(calculateDistance(mB,mC)+0.5) + floor(calculateDistance(mD,mA)+0.5) +
         floor(calculateDistance(mC,mD)+0.5);
         break;
    case 5: 
         perimeter += floor(calculateDistance(mB,mC)+0.5) + floor(calculateDistance(mC,mE)+0.5) +
         floor(calculateDistance(mE,mD)+0.5) + floor(calculateDistance(mD,mA)+0.5);
         break;
    case 6:
         perimeter +=  floor(calculateDistance(mB,mC)+0.5) +  floor(calculateDistance(mC,mF)+0.5) +  floor(calculateDistance(mF,mE)+0.5) +  floor(calculateDistance(mE,mD)+0.5) +  floor(calculateDistance(mD,mA)+0.5);
         break;
    case 7:
         perimeter +=  floor(calculateDistance(mB,mC)+0.5) +  floor(calculateDistance(mC,mG)+0.5) + floor(calculateDistance(mG,mF)+0.5) +  floor(calculateDistance(mF,mE)+0.5) +  floor(calculateDistance(mE,mD)+0.5) +  floor(calculateDistance(mD,mA)+0.5);
         break;
    case 8:
         perimeter +=  floor(calculateDistance(mB,mC)+0.5) +  floor(calculateDistance(mC,mH)+0.5) +   floor(calculateDistance(mH,mG)+0.5) +  floor(calculateDistance(mG,mF)+0.5) + floor(calculateDistance(mF,mE)+0.5) +  floor(calculateDistance(mE,mD)+0.5) +  floor(calculateDistance(mD,mA)+0.5);
         break;
  case 9:
         perimeter +=  floor(calculateDistance(mB,mI)+0.5) +  floor(calculateDistance(mI,mH)+0.5) +  floor(calculateDistance(mH,mG)+0.5) +  floor(calculateDistance(mG,mF)+0.5)  +  floor(calculateDistance(mF,mE)+0.5)  +  floor(calculateDistance(mE,mD)+0.5) +  floor(calculateDistance(mD,mA)+0.5);
         break;
  }
  return perimeter;
}
// only needed for calculating cols in grid mode 
float calTArea(cv::Mat t0, cv::Mat t1, cv::Mat t2,bool flag){
    float side1 = calculateDistance(t0,t1,flag);
    float side2 = calculateDistance(t1,t2,flag);
    float side3 = calculateDistance(t0,t2,flag);
    float perimeter = side1 + side2 + side3;
    float s = perimeter / 2;
    float area = sqrt(s * (s - side1) * (s - side2) * (s - side3));
    return area;
}
float calculateTriangleArea(cv::Mat t0, cv::Mat t1, cv::Mat t2, bool flag){
    float side1 = floor(calculateDistance(t0,t1,flag)+0.5);
    float side2 = floor(calculateDistance(t1,t2,flag)+0.5);
    float side3 = floor(calculateDistance(t0,t2,flag)+0.5);
    float perimeter = side1 + side2 + side3;
    float s = perimeter / 2;
    float area = sqrt(s * (s - side1) * (s - side2) * (s - side3));
    return area;
}
float calculateArea(vector<cv::Point2f> centers){   
  float area = 0;
  
  switch(centers.size()){
    case 3:
      area = calculateTriangleArea(mA,mB,mC,true);
      break;
    case 4:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true);
      break;
    case 5:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+calculateTriangleArea(mD,mC,mE,true);
      break;
    case 6:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+calculateTriangleArea(mD,mC,mE,true)+calculateTriangleArea(mC,mF,mE,true);
      break;
    case 7:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+calculateTriangleArea(mD,mC,mE,true)+calculateTriangleArea(mC,mF,mE,true)+calculateTriangleArea(mC,mG,mF,true);
      break;
    case 8:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+calculateTriangleArea(mD,mC,mE,true)+calculateTriangleArea(mC,mF,mE,true)+calculateTriangleArea(mC,mG,mF,true)+calculateTriangleArea(mC,mH,mG,true);
      break;
    case 9:
      area = calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+calculateTriangleArea(mD,mC,mE,true)+calculateTriangleArea(mC,mF,mE,true)+calculateTriangleArea(mC,mG,mF,true)+calculateTriangleArea(mC,mH,mG,true)+calculateTriangleArea(mB,mI,mH,true)+calculateTriangleArea(mC,mB,mH,true);
      break;
    default: break;
  }
  return area;
}
void drawString(char* string, int sub=0){

  char *c;
  for (c=string; *c != '\0'; c++) 
    {
      if (!sub) {
      glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *c);
//HELVETICA_18, *c);
      }
      else {
      glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, *c);
//HELVETICA_12, *c);
      }
    }
 
}

void drawStringLetter(std::string sstring){
  
  char *c;
  char *string =  (char*) sstring.c_str();
  for (c=string; *c != '\0'; c++)
    {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

void drawX(cv::Mat t3,cv::Mat t2, char xchar){

  float s = calculateDistance(t3,t2);
  char buffer[50];
  if (xchar == 'X') {
  int n = sprintf(buffer,"X\n");
  }
  else if (xchar == 'Y') {
  int n = sprintf(buffer,"Y\n");
  }
  else if (xchar == 'Z') {
  int n = sprintf(buffer, "Z\n");
  }

  float x_bmid = (t3.at<float>(0,0) + t2.at<float>(0,0))/2;
  float y_bmid = (t3.at<float>(1,0) + t2.at<float>(1,0))/2;
  float z_bmid = (-t3.at<float>(2,0) - t2.at<float>(2,0))/2;

  glPushMatrix();
  glColor3f(0,0,1);


  glLoadIdentity();
  glTranslatef(x_bmid,y_bmid,z_bmid);
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawString(buffer);
  glPopMatrix();
  cout << "I am in the X of line mode2"<<endl;

}

/**
@brief draws a letter, used for letters on Markers
@param currentMat gives us information about placement
@param letter the representative character
*/
void drawLetter(cv::Mat currentMat, std::string letter, int black=0,float dist=0.0){
   
  float xcoordinate = currentMat.at<float>(0,0);// + t2.at<float>(0,0))/2;
  float ycoordinate = currentMat.at<float>(1,0);// + t2.at<float>(1,0))/2;
  float zcoordinate = -1*currentMat.at<float>(2,0);// - t2.at<float>(2,0))/2;
  glPushMatrix();
  if (black==0) {
  glColor4ub(28,248,255,128);
  }
  else if(black==2){
  glColor4ub(255,255,255,255);
  xcoordinate -= 0.01;
  }

  glLoadIdentity();
glTranslatef(xcoordinate, ycoordinate, zcoordinate);//x_bmid,y_bmid,z_bmid);
if (dist !=  0.0) {
glTranslatef(0.0, dist, 0.0);
}
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawStringLetter(letter);
  
glPopMatrix();
}


void drawSideText(cv::Mat t3,cv::Mat t2){
  float sl = calcSlope(t3,t2); 
  float s = calculateDistance(t3,t2);
  char buffer[50];
  char buffer2[50];
  if (imperialUnitFlag){
  int n = sprintf(buffer,"%.0f in\n",floor(s+0.5));
  int m = sprintf(buffer2, "%.3f slope\n", sl);
  } else{
    int n = sprintf(buffer,"%.0f cm\n",floor(s+0.5));
    int m = sprintf(buffer2, "%.3f slope\n", sl);
  }
  
  float x_bmid = (t3.at<float>(0,0) + t2.at<float>(0,0))/2;
  float y_bmid = (t3.at<float>(1,0) + t2.at<float>(1,0))/2;
  float z_bmid = (-t3.at<float>(2,0) - t2.at<float>(2,0))/2;
  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(x_bmid,y_bmid,z_bmid);
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawString(buffer);
 glPopMatrix();
}
void drawSideTextTranslate(cv::Mat t3,cv::Mat t2, float unit){
    
  float s = calculateDistance(t2,t3);//; + calculateDistance(t25, t3);
  char buffer[50];
  if (!zflag) {
  if (imperialUnitFlag){
    int n = sprintf(buffer,"%.0f in\n",floor(s+0.5));
  } else{
    int n = sprintf(buffer,"%.0f cm\n",floor(s+0.5));
  }
  }
  else {
    int n = sprintf(buffer,"Z");
  }
  float x_bmid = (t3.at<float>(0,0) + t2.at<float>(0,0))/2;
  float y_bmid = (t3.at<float>(1,0) + t2.at<float>(1,0))/2;
  float z_bmid = (-t3.at<float>(2,0) - t2.at<float>(2,0))/2;

  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(x_bmid,y_bmid + unit,z_bmid);
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawString(buffer);
  drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
  drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
  glPopMatrix();
}
void drawLetterOnXTranslate(cv::Mat tnaught, std::string letter){
   float xcoordinate = tnaught.at<float>(0,0);
   float ycoordinate = tnaught.at<float>(1,0);
   float zcoordinate = -tnaught.at<float>(2,0);

  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(xcoordinate,ycoordinate,zcoordinate);
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawStringLetter(letter);
  drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
  drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
  glPopMatrix();
}
/**
    @brief Display Characters For Markers
*/
void drawArea(vector<cv::Point2f> centers){
  int perimeter = calculatePerimeter(centers);
  float area = calculateArea(centers);
  #ifdef DEBUG
//  cout << "Perimeter : "<<perimeter<<endl;
//  cout << "Area : " <<area <<endl;
  #endif
  char buffer[50];  
  char smallbuffer[50]; 
  char buffer2[50]; 
  if (imperialUnitFlag){
    int n = sprintf(buffer,"Area = %4.0f in",floor(area+0.5));
    int m = sprintf(smallbuffer, "^2\n");
    int l = sprintf(buffer2,"Perimeter = %i in\n", perimeter);
  }
  else{
    int n = sprintf(buffer,"Area = %4.0f cm\n",floor(area+0.5));
    int m = sprintf(smallbuffer, "^2\n");
    int l = sprintf(buffer2,"Perimeter = %i cm\n", perimeter);
  }

  float x_area = 0; 
  float x_area3 =0;
  float y_area = 0;
  float z_area = 0;
  float y_area2 = 0;
  float y_area3 = 0;
  float xtranslateArea = -4*0.015;
  float ytranslateArea = 2.5*0.015;
  x_area = mA.at<float>(0,0) + xtranslateArea;
  y_area = mA.at<float>(1,0) - 0.45*ytranslateArea;
  z_area = -mA.at<float>(2,0);
  y_area2 = mA.at<float>(1,0) -0.25*ytranslateArea;
  y_area3 = mA.at<float>(1,0) -0.30*ytranslateArea;

  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(x_area,y_area2,z_area);
  glRasterPos3f( 0.0f, 0.0f, 0.0f);
  drawString(buffer);
  drawString(smallbuffer,1);
  glPopMatrix();


  glPushMatrix();
  glLoadIdentity();  
  glTranslatef(x_area,y_area,z_area);
  glRasterPos3f(0.0f,0.0f,0.0f);
  drawString(buffer2);
  glPopMatrix();
}

float calcSlope(cv::Mat t0, cv::Mat t1) {
    float run = t0.at<float>(0,0) - t1.at<float>(0,0);
    float rise = t0.at<float>(1,0) - t1.at<float>(1,0);
    float slope = rise/run;
     
    return slope;
}

void lineMode(vector<cv::Point2f> centers){
  
  float translateDistance = -0.055;
  float lineWidth = 3;
  
  if (centers.size() == 3){
    glPushMatrix();
    glLoadIdentity();
    glColor3f(1,1,0);
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
    glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
    glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
    glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
    //glEnd();
    glTranslatef(0.0f,translateDistance,0.0f);
    
    glBegin(GL_LINES);
    glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
    glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
    glEnd();
    
    glPopMatrix();
    drawLetter(mA,lettermap[MarkerID[0]]);
    drawLetter(mB,lettermap[MarkerID[1]]);
    drawLetter(mC,lettermap[MarkerID[2]]);
    drawSideText(mC,mB);
    drawSideText(mB,mA);
//    drawSideTextTranslate(mC,mB,translateDistance);
  }
}

void lineMode2(vector<cv::Point2f> centers){
  
  float translateDistance = -0.055;
  float translateLetterDifference = -0.1;
  float lineWidth = 3;

 if (centers.size() == 3){
    glPushMatrix();
    glLoadIdentity();
    glColor3f(1,1,0);
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
    glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
    glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
    glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
    glEnd();
    glTranslatef(0.0f,translateDistance,0.0f);
    glBegin(GL_LINES);
    glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
    glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
    glEnd();
    glBegin(GL_LINES);
    glTranslatef(0.0f,translateDistance,0.0f);
    glTranslatef(0.0f,translateDistance,0.0f);
    glEnd();
    glPopMatrix();
    if (xflag){
      drawX(mB,mA, 'X');
    }
    else {
      drawSideText(mB, mA);
    }
    if (yflag) {
      drawX(mC,mB,'Y');
    }
    else {
      drawSideText(mC,mB);
    }
    drawSideTextTranslate(mC,mA,translateDistance);
    drawLetter(mA,lettermap[MarkerID[0]]);
    drawLetter(mB,lettermap[MarkerID[1]]);
    drawLetter(mC,lettermap[MarkerID[2]]);
    drawLetter(mA,lettermap[MarkerID[0]], 1, translateDistance);
    drawLetter(mC,lettermap[MarkerID[2]], 1, translateDistance);
  } 
}

/*
*  getCos, getSin, and cal_bottom_len
*  are used for calculate Grids for the shape
*  completed by  Chan Kim  
* 
*/
float getCos(cv::Mat t0, cv::Mat t1){
    float run = t0.at<float>(0,0) - t1.at<float>(0,0);
    float hypoten = calculateDistance(t0,t1,false);
    return run/hypoten;
}

float getSin(cv::Mat t0, cv::Mat t1){
    float rise = t0.at<float>(1,0) - t1.at<float>(1,0);
    float hypoten = calculateDistance(t0,t1,false);
    return rise/hypoten;
}
float cal_bottom_len(float hypotenuse, float height){
     return sqrt((hypotenuse*hypotenuse)-(height*height));
}
void simulateMode(vector<cv::Point2f> centers){
  switch(centers.size()){
    case 7:
        eA = 2*mA-mC;
        eB = 2*mB-mC;
        eD = 2*mD-mC;
        eE = 2*mE-mC;
        eF = 2*mF-mC;
        eG = 2*mG-mC;
        eAD = (eA+eD)/2;
        eDE = (eD+eE)/2;
        eEF = (eE+eF)/2; 
        eFG = (eF+eG)/2;
        eGB = (eG+eB)/2;
        eBA = (eB+eA)/2;
        hAC = (mA+mC)/2;
        hBC = (mB+mC)/2;
        hDC = (mD+mC)/2;
        hEC = (mE+mC)/2;
        hFC = (mF+mC)/2;
        hGC = (mG+mC)/2;
        hAD = (mA+mD)/2;
        hDE = (mD+mE)/2;
        hEF = (mE+mF)/2;
        hFG = (mF+mG)/2;
        hGB = (mG+mB)/2;
        hBA = (mB+mA)/2;
        break;
    default: break;
  }
}
// available mode only for triangle and quadranglie
void drawHeight(vector<cv::Point2f> centers){
  float pH[2]; // point for drawing height
  float A[2],B[2],C[2];
  float cos, sin;
  float H1, Area1;
  float AB, AC, BC;
  float v_x; // foot of perpendicular on BC from point A
  //triangle
  switch(centers.size()){
    case 3:
       // Bottom line (B-C)
       cos = getCos(mB,mC);
       sin = getSin(mB,mC);
       // calculate left bottom & right bottom (+-2)
       BC = calculateDistance(mB,mC,false);
       AB = calculateDistance(mA,mB,false);
       AC = calculateDistance(mA,mC,false);
       Area1 = calTArea(mA,mB,mC,false);
       H1 = 2*Area1/BC;
       A[0]=mA.at<float>(0,0);
       A[1]=mA.at<float>(1,0);
       B[0]=mB.at<float>(0,0);
       B[1]=mB.at<float>(1,0);
       C[0]=mC.at<float>(0,0);
       C[1]=mC.at<float>(1,0);
       v_x = cos*cos*A[0]+cos*sin*A[1]-cos*sin*B[1]+sin*sin*B[0];
       // A
       //   B   C
       if (v_x<B[0]){
           pH[0] = B[0]+cal_bottom_len(AB,H1)*cos;
           pH[1] = B[1]+cal_bottom_len(AB,H1)*sin;
       }
       //        A
       //  B   C
       else if (v_x > C[0]){
           pH[0] = C[0]-cal_bottom_len(AC,H1)*cos;
           pH[1] = C[1]-cal_bottom_len(AC,H1)*sin;
       }
       //    A
       //  B    C
       else{
           pH[0] = B[0]-cal_bottom_len(AB,H1)*cos;
           pH[1] = B[1]-cal_bottom_len(AB,H1)*sin;
       } 
    glColor4ub(255,0,0,255);
    glLineWidth(3);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2,0xaaaa);
    glBegin(GL_LINES);
    if(v_x<B[0]){
       glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
       glVertex3f(pH[0],pH[1],-mB.at<float>(2,0));
    }else if(v_x >C[0]){
       glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
       glVertex3f(pH[0],pH[1],-mC.at<float>(2,0));
    }
    glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
   glVertex3f(pH[0],pH[1],-mA.at<float>(2,0));
   glEnd(); 
   glDisable(GL_LINE_STIPPLE);
 
  // for drawing number of the height
  BC = calculateDistance(mB,mC);
  Area1 =calculateArea(centers);
  H1 = floor(2*Area1/BC+0.5);
  char buffer[50];
  if (imperialUnitFlag){
    int n = sprintf(buffer,"%.0f in\n",H1);
  } else{
    int n = sprintf(buffer,"%.0f cm\n",H1);
  }
  cout << "BC : "<< BC << " Area1 : " << Area1 <<" H1 : "<<H1<<endl; 
  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef((pH[0]+mA.at<float>(0,0))/2,(pH[1]+mA.at<float>(1,0))/2,-mA.at<float>(2,0));
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawString(buffer);
  glPopMatrix();
  break;
   }   
}
void gridMode(vector<cv::Point2f> centers){
  float lb[2],lu[2],rb[2],ru[2];
  float A[2],B[2],C[2],D[2],E[2],F[2],G[2],H[2],I[2];
  float col_unit, cols, rows;
  float cos, sin;
  float H1, H2, Area1, Area2;
  float AB, AC, AD, BC, BD, CD;
  float v_x; // foot of perpendicular on BC from point A
  //triangle
  switch(centers.size()){
    case 3:
       // Bottom line (B-C)
       cos = getCos(mB,mC);
       sin = getSin(mB,mC);
       // calculate left bottom & right bottom (+-2)
       BC = calculateDistance(mB,mC,false);
       AB = calculateDistance(mA,mB,false);
       AC = calculateDistance(mA,mC,false);
       Area1 = calTArea(mA,mB,mC,false);
       H1 = 2*Area1/BC;
       A[0]=mA.at<float>(0,0);
       A[1]=mA.at<float>(1,0);
       B[0]=mB.at<float>(0,0);
       B[1]=mB.at<float>(1,0);
       C[0]=mC.at<float>(0,0);
       C[1]=mC.at<float>(1,0);
       v_x = cos*cos*A[0]+cos*sin*A[1]-cos*sin*B[1]+sin*sin*B[0];
       // A
       //   B   C
        if (v_x<B[0]){
           lu[0] = A[0];
           lu[1] = A[1];
           lb[0] = B[0]+cal_bottom_len(AB,H1)*cos;
           lb[1] = B[1]+cal_bottom_len(AB,H1)*sin;
           ru[0] = A[0]+(C[0]-lb[0]);
           ru[1] = A[1]+(C[1]-lb[1]);
           rb[0] = C[0];
           rb[1] = C[1];
       }
       //        A
       //  B   C
       else if(v_x > C[0]){
           lb[0] = B[0];
           lb[1] = B[1];
           ru[0] = A[0];
           ru[1] = A[1];
           rb[0] = C[0]-cal_bottom_len(AC,H1)*cos;
           rb[1] = C[1]-cal_bottom_len(AC,H1)*sin;
           lu[0] = A[0]-(rb[0]-B[0]);
           lu[1] = A[1]-(rb[1]-B[1]);
       }
       //    A
       //  B    C
       else{
           lb[0] = B[0];
           lb[1] = B[1];
           rb[0] = C[0];
           rb[1] = C[1];
           lu[0] = A[0]+cal_bottom_len(AB,H1)*cos;
           lu[1] = A[1]+cal_bottom_len(AB,H1)*sin;
           ru[0] = A[0]-cal_bottom_len(AC,H1)*cos;
           ru[1] = A[1]-cal_bottom_len(AC,H1)*sin;
       } 
       break;
    //quadrangle
    case 4:
       // Bottom line (B-C)
       cos = getCos(mB,mC);
       sin = getSin(mB,mC);
       // length of each side
       BC = calculateDistance(mB,mC,false);
       AB = calculateDistance(mB,mA,false);
       AC = calculateDistance(mA,mC,false);
       CD = calculateDistance(mC,mD,false);
       Area1 = calTArea(mC,mB,mA,false);
       Area2 = calTArea(mC,mD,mB,false);
       H1 = 2*Area1/BC;
       H2 = 2*Area2/BC;
       A[0]=mA.at<float>(0,0);
       A[1]=mA.at<float>(1,0);
       B[0]=mB.at<float>(0,0);
       B[1]=mB.at<float>(1,0);
       C[0]=mC.at<float>(0,0);
       C[1]=mC.at<float>(1,0);
       D[0]=mD.at<float>(0,0);
       D[1]=mD.at<float>(1,0);
       // A.y < D.y
       if (A[1]<D[1]){
           // A
           //  (D)    D 
           //    B      C
           if (A[0]<B[0] && C[0]>D[0]){
               lu[0] = A[0];
               lu[1] = A[1];
               lb[0] = B[0]+cal_bottom_len(AB,H1)*cos;
               lb[1] = B[1]+cal_bottom_len(AB,H1)*sin;
               ru[0] = A[0]+(C[0]-lb[0]);
               ru[1] = A[1]+(C[1]-lb[1]);
               rb[0] = C[0];
               rb[1] = C[1];
           }
           // A
           //            D
           //    B     C
           else if(A[0]<B[0] && C[0]<D[0]){
               lu[0] = A[0];
               lu[1] = A[1];
               lb[0] = B[0]+cal_bottom_len(AB,H1)*cos;
               lb[1] = B[1]+cal_bottom_len(AB,H1)*sin;
               rb[0] = C[0]-cal_bottom_len(CD,H2)*cos;
               rb[1] = C[1]-cal_bottom_len(CD,H2)*sin;
               ru[0] = lu[0]+(rb[0]-lb[0]);
               ru[1] = lu[1]+(rb[1]-lb[1]);
          }
          //      A   D
          //  B           C
          else if (A[0]>B[0] && C[0]>D[0] && A[0] <C[0]){
               rb[0] = C[0];
               rb[1] = C[1];
               lb[0] = B[0];
               lb[1] = B[1];
               lu[0] = A[0]+cal_bottom_len(AB,H1)*cos;
               lu[1] = A[1]+cal_bottom_len(AB,H1)*sin;
               ru[0] = A[0]-cal_bottom_len(AC,H1)*cos;
               ru[1] = A[1]-cal_bottom_len(AC,H1)*sin;
          }
          //     A        D
          //  B     C
          else if (A[0]>B[0] && C[0]<D[0] && A[0] <C[0]){
               lu[0] = A[0]+cal_bottom_len(AB,H1)*cos;
               lu[1] = A[1]+cal_bottom_len(AB,H1)*sin;
               lb[0] = B[0];
               lb[1] = B[1];
               rb[0] = C[0]-cal_bottom_len(CD,H2)*cos;
               rb[1] = C[1]-cal_bottom_len(CD,H2)*sin;
               ru[0] = lu[0]-(lb[0]-rb[0]);
               ru[1] = lu[1]-(lb[1]-rb[1]);
          }
          //             A D
          //  B     C
         else if (A[0] > C[0]){
               lb[0] = B[0];
               lb[1] = B[1];
               lu[0] = A[0]+cal_bottom_len(AB,H1)*cos;
               lu[1] = A[1]+cal_bottom_len(AB,H1)*sin;
               rb[0] = C[0]-cal_bottom_len(CD,H2)*cos;
               rb[1] = C[1]-cal_bottom_len(CD,H2)*sin;
               ru[0] = lu[0]+(rb[0]-lb[0]);
               ru[1] = lu[1]+(rb[1]-lb[1]);
          }
       }
       else if (A[1]>D[1]){
           //  (D)    D 
           // A    
           //    B      C
           if (A[0]<B[0] && C[0]>D[0]){
               rb[0] = C[0];
               rb[1] = C[1];
               lb[0] = B[0]+cal_bottom_len(AB,H1)*cos;
               lb[1] = B[1]+cal_bottom_len(AB,H1)*sin;
               ru[0] = D[0]-cal_bottom_len(CD,H2)*cos;
               ru[1] = D[1]-cal_bottom_len(CD,H2)*sin;
               lu[0] = ru[0]-(C[0]-lb[0]);
               lu[1] = ru[1]-(C[1]-lb[1]);
           }
           //            D
           // A
           //    B     C
           else if(A[0]<B[0] && C[0]<D[0]){
               ru[0] = D[0];
               ru[1] = D[1];
               rb[0] = C[0]-cal_bottom_len(CD,H2)*cos;
               rb[1] = C[1]-cal_bottom_len(CD,H2)*sin;
               lb[0] = B[0]+cal_bottom_len(AB,H1)*cos;
               lb[1] = B[1]+cal_bottom_len(AB,H1)*sin;
               lu[0] = ru[0]-(rb[0]-lb[0]);
               lu[1] = ru[1]-(rb[1]-lb[1]);
         }
          //      A   D
          //  B           C
          else if (A[0]>B[0] && C[0]>D[0] && A[0] <C[0]){
               rb[0] = C[0];
               rb[1] = C[1];
               lb[0] = B[0];
               lb[1] = B[1];
               ru[0] = D[0]-cal_bottom_len(CD,H2)*cos;
               ru[1] = D[1]-cal_bottom_len(CD,H2)*sin;
               lu[0] = ru[0]-(rb[0]-lb[0]);
               lu[1] = ru[1]-(rb[1]-lb[1]);
          }
          //             D
          //    A     (A)
          //  B     C
          else if (A[0]>B[0] && C[0]<D[0]){
               ru[0] = D[0];
               ru[1] = D[1];
               lb[0] = B[0];
               lb[1] = B[1];
               rb[0] = C[0]-cal_bottom_len(CD,H2)*cos;
               rb[1] = C[1]-cal_bottom_len(CD,H2)*sin;
               lu[0] = ru[0]-(rb[0]-lb[0]);
               lu[1] = ru[1]-(rb[1]-lb[1]);
          } 
       }
       break;
       default: break;
   }
   if(centers.size()>2){
   // calculate a number of cols and rows
   cols= floor(convertD(H1)+0.5);
   col_unit = H1/cols;
   rows = floor(convertD(sqrt((rb[0]-lb[0])*(rb[0]-lb[0])+(rb[1]-lb[1])*(rb[1]-lb[1])))+0.5);
   
   // draw yellow shape as the Free Mode
   freeMode(centers,false);
   #ifdef DEBUG
//   std::cout << rows << " rows" << std::endl;
//   std::cout << cols << " columns" << std::endl; 
   #endif
   // draw grids
   GLfloat grid2x2[12] = {rb[0],rb[1],-mC.at<float>(2,0),lb[0],lb[1],-mB.at<float>(2,0),ru[0],ru[1],-mA.at<float>(2,0),lu[0],lu[1],-mA.at<float>(2,0)};
   
   glPushMatrix();
   glLoadIdentity();
   glColor4ub(255,0,0,128);
   glEnable(GL_MAP2_VERTEX_3);
   glMap2f(GL_MAP2_VERTEX_3,
            0.0, 1.0,  // U ranges 0..1 
            3,         // U stride, 3 floats per coord 
            2,         // U is 2nd order, ie. linear 
            0.0, 1.0,  // V ranges 0..1 
           2 * 3,      // V stride, row is 2 coords, 3 floats per coord
            2,         // V is 2nd order, ie linear 
            grid2x2);  // control points 
  glMapGrid2f(ceil(rows), 0.0, 1.0,
            ceil(cols), 0.0, 1.0);
        glLineWidth(2); 
       glEvalMesh2(GL_LINE,
              0, ceil(rows),   // Starting at 0 mesh 5 steps (rows). 
              0, ceil(cols));  // Starting at 0 mesh 6 steps (columns).
  glPopMatrix();
  }
}

struct Points{
    float x, y;
    int idx;
};
bool Sort_x(const Points& a, const Points& b){
    return a.x < b.x;
}
bool Sort_y(const Points& a, const Points& b){
    return a.y < b.y;
}
// assign each marker to specific letter
// assign random located markers to the following
// A  D  E  
// B  C  F
// I  H  G 
void assignMarker(vector<cv::Point2f> centers){
  // init
  mA.empty();
  mB.empty();
  mC.empty();
  mD.empty();
  mE.empty();
  mF.empty();
  mG.empty();
  mH.empty();
  mI.empty();
 cv::Mat tM[centers.size()];
 cv::Mat t[centers.size()];
 vector<Points> vt_x; // x coordinate vectors of every marker
 vector<Points> vt_y; // y coordinate vectors of every marker
 vector<Points> temp;
 vector<int> tMarkerID; // temporate Marker ID

 // save x,y Coordinates and Index of marker
 for(int i=0; i<centers.size(); i++){
   t[i]= TheMarkers[i].Tvec;
   Points p  = {t[i].at<float>(0,0),t[i].at<float>(1,0),i};
   vt_x.push_back(p);
   tMarkerID.push_back(TheMarkers[i].id); 
 } 
 vt_y = vt_x;
 sort(vt_x.begin(),vt_x.end(),Sort_x); //sort by x
 sort(vt_y.begin(),vt_y.end(),Sort_y); //sort by y

 switch(centers.size()){
   case 1: 
      mA = t[0];
      MarkerID[0] = tMarkerID[0];
      break;
   case 2:
      mA = t[vt_x[0].idx];
      mB = t[vt_x[1].idx];
      MarkerID[0]= tMarkerID[vt_x[0].idx];
      MarkerID[1]= tMarkerID[vt_x[1].idx];
      break;
 
   case 3:
       tM[0] = t[vt_x[0].idx];
       MarkerID[0]= tMarkerID[vt_x[0].idx];
       tM[1] = t[vt_x[1].idx];
       MarkerID[1]= tMarkerID[vt_x[1].idx];
       tM[2] = t[vt_x[2].idx];
       MarkerID[2]= tMarkerID[vt_x[2].idx];
          
       if(line_flag){
           mA = tM[0];
           mB = tM[1];
           mC = tM[2];
       }else{
           mA = t[vt_y[0].idx];
           MarkerID[0]= tMarkerID[vt_y[0].idx];
           temp.push_back(vt_y[1]);
           temp.push_back(vt_y[2]);
           sort(temp.begin(),temp.end(),Sort_x);
           mB = t[temp[0].idx];
           MarkerID[1]= tMarkerID[temp[0].idx];
           mC = t[temp[1].idx];
           MarkerID[2]= tMarkerID[temp[1].idx];
       }
       break;
case 4:
      temp.push_back(vt_y[0]);
      temp.push_back(vt_y[1]);
      sort(temp.begin(),temp.end(),Sort_x);
      mA = t[temp[0].idx];
      MarkerID[0]= tMarkerID[temp[0].idx];
      mD = t[temp[1].idx];
      MarkerID[3]= tMarkerID[temp[1].idx];
      temp.clear();
      temp.push_back(vt_y[2]);
      temp.push_back(vt_y[3]);
      sort(temp.begin(),temp.end(),Sort_x);
      mB = t[temp[0].idx];
      MarkerID[1]= tMarkerID[temp[0].idx];
      mC = t[temp[1].idx];
      MarkerID[2]= tMarkerID[temp[1].idx];
      break;
  case 5:
      mE = t[vt_x[4].idx];
      MarkerID[4]= tMarkerID[vt_x[4].idx];
      //exclude E
      for(int i=0;i<centers.size();i++){
        if(vt_y[i].idx == vt_x[4].idx){
            vt_y.erase(vt_y.begin()+i);
            vt_x.erase(vt_x.begin()+4);
            break;
        }
      }
      temp.push_back(vt_y[0]);
      temp.push_back(vt_y[1]);
      sort(temp.begin(),temp.end(),Sort_x);
      mA = t[temp[0].idx];
      MarkerID[0]= tMarkerID[temp[0].idx];
      mD = t[temp[1].idx];
      MarkerID[3]= tMarkerID[temp[1].idx];
      temp.clear();
      temp.push_back(vt_y[2]);
      temp.push_back(vt_y[3]);
      sort(temp.begin(),temp.end(),Sort_x);
      mB = t[temp[0].idx];
      MarkerID[1]= tMarkerID[temp[0].idx];
      mC = t[temp[1].idx];
      MarkerID[2]= tMarkerID[temp[1].idx];
      break;
  case 6:
      temp.push_back(vt_y[0]);
      temp.push_back(vt_y[1]);
      temp.push_back(vt_y[2]);
      sort(temp.begin(),temp.end(),Sort_x);
      mA = t[temp[0].idx];
      MarkerID[0]= tMarkerID[temp[0].idx];
      mD = t[temp[1].idx];
      MarkerID[3]= tMarkerID[temp[1].idx];
      mE = t[temp[2].idx];
      MarkerID[4]= tMarkerID[temp[2].idx];
      temp.clear();
      temp.push_back(vt_y[3]);
      temp.push_back(vt_y[4]);
      temp.push_back(vt_y[5]);
      sort(temp.begin(),temp.end(),Sort_x);
      mB = t[temp[0].idx];
      MarkerID[1]= tMarkerID[temp[0].idx];
      mC = t[temp[1].idx];
      MarkerID[2]= tMarkerID[temp[1].idx];
      mF = t[temp[2].idx];
      MarkerID[5]= tMarkerID[temp[2].idx];
      break;
    case 7:
       if(vt_y[3].idx == vt_x[3].idx){
            tM[2] = t[vt_y[3].idx];
            temp.push_back(vt_y[0]);
            temp.push_back(vt_y[1]);
            temp.push_back(vt_y[2]);
            sort(temp.begin(),temp.end(),Sort_x);
            tM[0] = t[temp[0].idx];
            tM[3] = t[temp[1].idx];
            tM[4] = t[temp[2].idx];
            MarkerID[2] = tMarkerID[vt_y[3].idx];
            MarkerID[0] = tMarkerID[temp[0].idx];
            MarkerID[3] = tMarkerID[temp[1].idx];
            MarkerID[4] = tMarkerID[temp[2].idx];
            temp.clear();
            temp.push_back(vt_y[4]);
            temp.push_back(vt_y[5]);
            temp.push_back(vt_y[6]);
            sort(temp.begin(),temp.end(),Sort_x);
            tM[1] = t[temp[0].idx];
            tM[6] = t[temp[1].idx];
            tM[5] = t[temp[2].idx];
            MarkerID[1] = tMarkerID[temp[0].idx];
            MarkerID[6] = tMarkerID[temp[1].idx];
            MarkerID[5] = tMarkerID[temp[2].idx];
            temp.clear();    
        }else{
            tM[2] = t[vt_x[3].idx];
            temp.push_back(vt_x[0]);
            temp.push_back(vt_x[1]);
            temp.push_back(vt_x[2]);
            sort(temp.begin(),temp.end(),Sort_y);
            tM[0] = t[temp[0].idx];
            tM[1] = t[temp[1].idx];
            tM[6] = t[temp[2].idx];
            MarkerID[2] = tMarkerID[vt_x[3].idx];
            MarkerID[0] = tMarkerID[temp[0].idx];
            MarkerID[1] = tMarkerID[temp[1].idx];
            MarkerID[6] = tMarkerID[temp[2].idx];
            temp.clear();
            temp.push_back(vt_x[4]);
            temp.push_back(vt_x[5]);
            temp.push_back(vt_x[6]);
            sort(temp.begin(),temp.end(),Sort_y);
            tM[3] = t[temp[0].idx];
            tM[4] = t[temp[1].idx];
            tM[5] = t[temp[2].idx];
            MarkerID[3] = tMarkerID[temp[0].idx];
            MarkerID[4] = tMarkerID[temp[1].idx];
            MarkerID[5] = tMarkerID[temp[2].idx];
             temp.clear();    
       }
       if(calculateDistance(tM[6],tM[2],true) < calculateDistance(tM[6],tM[1],true)){
           coin_flag=false; //GC<GB
           temp.push_back(vt_y[0]);
           temp.push_back(vt_y[1]);
           temp.push_back(vt_y[2]);
           sort(temp.begin(),temp.end(),Sort_x);
           mA = t[temp[0].idx];
           MarkerID[0]= tMarkerID[temp[0].idx];
           mD = t[temp[1].idx];
           MarkerID[3]= tMarkerID[temp[1].idx];
           mE = t[temp[2].idx];
           MarkerID[4]= tMarkerID[temp[2].idx];
           temp.clear();
           temp.push_back(vt_y[3]);
           temp.push_back(vt_y[4]);
           temp.push_back(vt_y[5]);
           sort(temp.begin(),temp.end(),Sort_x);
           mB = t[temp[0].idx];
           MarkerID[1]= tMarkerID[temp[0].idx];
           mC = t[temp[1].idx];
           MarkerID[2]= tMarkerID[temp[1].idx];
           mF = t[temp[2].idx];
           MarkerID[5]= tMarkerID[temp[2].idx];
           mG = t[vt_y[6].idx];
           MarkerID[6]=tMarkerID[vt_y[6].idx];
        }
        else{
           coin_flag=true;
           //  coin demonstration
           mA = tM[0];
           mB = tM[1];
           mC = tM[2];
           mD = tM[3];
           mE = tM[4];
           mF = tM[5];
           mG = tM[6];
           if(simulate_flag){simulateMode(centers);}
       }
       break;
   case 8:
      temp.push_back(vt_y[6]);
      temp.push_back(vt_y[7]);
      sort(temp.begin(),temp.end(),Sort_x);
      mH = t[temp[0].idx];
      MarkerID[7]= tMarkerID[temp[0].idx];
      mG = t[temp[1].idx];
      MarkerID[6]= tMarkerID[temp[1].idx];
      temp.clear();
      temp.push_back(vt_y[0]);
      temp.push_back(vt_y[1]);
      temp.push_back(vt_y[2]);
      sort(temp.begin(),temp.end(),Sort_x);
      mA = t[temp[0].idx];
      MarkerID[0]= tMarkerID[temp[0].idx];
      mD = t[temp[1].idx];
      MarkerID[3]= tMarkerID[temp[1].idx];
      mE = t[temp[2].idx];
      MarkerID[4]= tMarkerID[temp[2].idx];
      temp.clear();
      temp.push_back(vt_y[3]);
      temp.push_back(vt_y[4]);
      temp.push_back(vt_y[5]);
      sort(temp.begin(),temp.end(),Sort_x);
      mB = t[temp[0].idx];
      MarkerID[1]= tMarkerID[temp[0].idx];
      mC = t[temp[1].idx];
      MarkerID[2]= tMarkerID[temp[1].idx];
      mF = t[temp[2].idx];
      MarkerID[5]= tMarkerID[temp[2].idx];
      break;
    case 9:
      temp.push_back(vt_y[0]);
      temp.push_back(vt_y[1]);
      temp.push_back(vt_y[2]);
      sort(temp.begin(),temp.end(),Sort_x);
      mA = t[temp[0].idx];
      MarkerID[0]= tMarkerID[temp[0].idx];
      mD = t[temp[1].idx];
      MarkerID[3]= tMarkerID[temp[1].idx];
      mE = t[temp[2].idx];
      MarkerID[4]= tMarkerID[temp[2].idx];
      temp.clear();
      temp.push_back(vt_y[3]);
      temp.push_back(vt_y[4]);
      temp.push_back(vt_y[5]);
      sort(temp.begin(),temp.end(),Sort_x);
      mB = t[temp[0].idx];
      MarkerID[1]= tMarkerID[temp[0].idx];
      mC = t[temp[1].idx];
      MarkerID[2]= tMarkerID[temp[1].idx];
      mF = t[temp[2].idx];
      MarkerID[5]= tMarkerID[temp[2].idx];
      temp.clear();
      temp.push_back(vt_y[6]);
      temp.push_back(vt_y[7]);
      temp.push_back(vt_y[8]);
      sort(temp.begin(),temp.end(),Sort_x);
      mI = t[temp[0].idx];
      MarkerID[8]= tMarkerID[temp[0].idx];
      mH = t[temp[1].idx];
      MarkerID[7]= tMarkerID[temp[1].idx];
      mG = t[temp[2].idx];
      MarkerID[6]= tMarkerID[temp[2].idx];
      break;
   default: break;
 }
 //clear
 if(vt_x.size()>0) vt_x.clear();
 if(vt_y.size()>0) vt_y.clear();
 if(temp.size()>0) temp.clear();
 if(tMarkerID.size()>0) tMarkerID.clear();
}
// detect Markers
// if using this func, be cautious when putting markers
void detectMarker(vector<cv::Point2f> centers){
  // init
  mA.empty();
  mB.empty();
  mC.empty();
  mD.empty();
  mE.empty();
  mF.empty();
  mG.empty();
  mH.empty();
  mI.empty();

  switch(centers.size()){
      case 2:
        mA = TheMarkers[0].Tvec;
        mB = TheMarkers[1].Tvec;
        break;
      case 3: 
        mA = TheMarkers[1].Tvec;
        mB = TheMarkers[2].Tvec;
        mC = TheMarkers[0].Tvec;
        break;
      case 4:
        mA = TheMarkers[1].Tvec;
        mB = TheMarkers[2].Tvec;
        mC = TheMarkers[0].Tvec;
        mD = TheMarkers[3].Tvec;
        break;
      case 5:
        mA = TheMarkers[2].Tvec;
        mB = TheMarkers[3].Tvec;
        mC = TheMarkers[0].Tvec;
        mD = TheMarkers[4].Tvec;
        mE = TheMarkers[1].Tvec;
        break;
      case 6:
        mC = TheMarkers[0].Tvec;
        mF = TheMarkers[1].Tvec;
        mE = TheMarkers[2].Tvec;
        mA = TheMarkers[3].Tvec;
        mB = TheMarkers[4].Tvec;
        mD = TheMarkers[5].Tvec;
        break;
      case 7:
        mC = TheMarkers[0].Tvec;
        mG = TheMarkers[1].Tvec;
        mF = TheMarkers[2].Tvec;
        mE = TheMarkers[3].Tvec;
        mA = TheMarkers[4].Tvec;
        mB = TheMarkers[5].Tvec;
        mD = TheMarkers[6].Tvec;
        break;
      case 8:
        mH = TheMarkers[0].Tvec;
        mC = TheMarkers[1].Tvec;
        mG = TheMarkers[2].Tvec;
        mF = TheMarkers[3].Tvec;
        mE = TheMarkers[4].Tvec;
        mA = TheMarkers[5].Tvec;
        mB = TheMarkers[6].Tvec;
        mD = TheMarkers[7].Tvec;
        break;
      case 9: 
        mI = TheMarkers[0].Tvec;
        mH = TheMarkers[1].Tvec;
        mC = TheMarkers[2].Tvec;
        mG = TheMarkers[3].Tvec;
        mF = TheMarkers[4].Tvec;
        mE = TheMarkers[5].Tvec;
        mA = TheMarkers[6].Tvec;
        mB = TheMarkers[7].Tvec;
        mD = TheMarkers[8].Tvec;
        break;
      default: break;
   }
}

void freeMode(vector<cv::Point2f> centers,bool outline_flag = false){
   float lineWidth = 2;
   float AB, BC, AC, AD, BD, CD;
   float translateDistance;
   float r_eA,r_eB,r_eD,r_eE,r_eF,r_eG,r_eAD,r_eDE,r_eEF,r_eFG,r_eGB,r_eBA; // radius of extended circle for simuulation
   float r_hAC,r_hDC,r_hBC,r_hEC,r_hFC,r_hGC,r_hAD,r_hDE,r_hEF,r_hFG,r_hGB,r_hBA;
   Mat centered;// position for the shape's name
   String shape_name ="";
   float temp_area, temp_area1;
   glColor4ub(255,255,0,200);
   glPushMatrix();
   glLoadIdentity();

/*   
   linear_cb->disable();
   linear_rg->disable();
   grid_cb->disable();
   simulate_rg->disable();
   outline_cb->disable();
   unit_rg->disable();
*/
   switch(centers.size()){
        case 1: 
          linear_cb->disable();
          linear_rg->disable();
          grid_cb->disable();
          simulate_rg->disable();
          outline_cb->disable();
          unit_rg->disable();
          drawLetter(mA,lettermap[MarkerID[0]]);
          break;
        case 2: 
          linear_cb->disable();
          linear_rg->disable();
          grid_cb->disable();
          simulate_rg->disable();
          outline_cb->disable();
          unit_rg->enable();
          glBegin(GL_LINES);
          glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glEnd();
          glPopMatrix();
          drawSideText(mA,mB);
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          break;
        case 3:
          simulate_rg->disable();
          if(grid_flag){
           linear_cb->disable();
           outline_cb->disable();
          }else{
           grid_cb->enable();
           linear_cb->enable();
           outline_cb->enable();
          }
          if(line_flag){
            linear_rg->enable();
            grid_cb->disable();
            outline_cb->disable();
            translateDistance = -0.055;
            glColor3f(1,1,0);
            glLineWidth(lineWidth);
            glBegin(GL_LINES);
            glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
            glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
            glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
            glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
            glEnd();
            glTranslatef(0.0f,translateDistance,0.0f);
            glBegin(GL_LINES);
            glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
            glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
            glEnd();
            glBegin(GL_LINES);
            glTranslatef(0.0f,translateDistance,0.0f);
            glTranslatef(0.0f,translateDistance,0.0f);
            glEnd();
            glPopMatrix();
            if (xflag) drawX(mB,mA, 'X');
            else drawSideText(mB, mA);
            if (yflag) drawX(mC,mB,'Y');
            else drawSideText(mC,mB);
            drawSideTextTranslate(mC,mA,translateDistance);
            drawLetter(mA,lettermap[MarkerID[0]], 1, translateDistance);
            drawLetter(mC,lettermap[MarkerID[2]], 1, translateDistance);
          }else{
              linear_rg->disable();
              AB = floor(calculateDistance(mA,mB,true)+0.5);
              BC = floor(calculateDistance(mB,mC,true)+0.5);
              AC = floor(calculateDistance(mC,mA,true)+0.5);
              centered = (mA+mB+mC)/3;
              if(AB == BC && BC == AC){// equilateral triangle
                shape_name ="Equilateral";
                glColor4ub(0,255,0,200);
                glBegin(GL_TRIANGLES);
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glEnd();
                glColor4ub(255,0,0,200);
                glLineWidth(10);
                glBegin(GL_LINES);      
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glEnd();
              }
              else if(AB==BC || BC == AC || AB == AC){// isosceles triangle
               shape_name ="Isosceles";
               glColor4ub(0,255,255,200);
               glBegin(GL_TRIANGLES);
               glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
               glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
               glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
               glEnd();
               glColor4ub(255,0,0,200);
               glLineWidth(15);
               glBegin(GL_LINES);      
               if(AB==BC){
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
               }
               else if(BC==AC){
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
               }else if(AB==AC){
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              }
               glEnd();
              }
              else{
               glColor4ub(255,255,0,200);
               glBegin(GL_TRIANGLES);
               glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
               glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
               glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
               glEnd();
             }
              if(outline_flag){
                  drawHeight(centers);
                  glColor4ub(255,0,0,200);
                  glLineWidth(lineWidth);
                  glBegin(GL_LINES);      
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glEnd(); 
              }
              drawSideText(mA,mB);
              drawSideText(mA,mC);
              drawSideText(mB,mC);
              drawArea(centers);
              if(shape_name!=""){
                  drawLetter(centered,shape_name,2,0.0);
                  drawLetter(centered,"Triangle",2,0.01);
              }
          }
          glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
         break;
      case 4: 
          unit_rg->enable();
          linear_cb->disable();
          linear_rg->disable();
          simulate_rg->disable();
          if(grid_flag){
           outline_cb->disable();
          }else{
           grid_cb->enable();
           outline_cb->enable();
          }
          AB = floor(calculateDistance(mA,mB,true)+0.5);
          BC = floor(calculateDistance(mB,mC,true)+0.5);
          AC = floor(calculateDistance(mC,mA,true)+0.5);
          AD = floor(calculateDistance(mA,mD,true)+0.5);
          CD = floor(calculateDistance(mC,mD,true)+0.5);
          BD = floor(calculateDistance(mB,mD,true)+0.5);
          temp_area = AD*BD;
          temp_area1 = floor(calculateTriangleArea(mA,mB,mC,true)+calculateTriangleArea(mA,mC,mD,true)+0.5);
          centered = (mA+mB+mC+mD)/4;
          if(AB == BC && BC == CD  && AD == BC && AC == BD){// square 
              shape_name ="Square";
              glColor4ub(0,255,0,200);
              glBegin(GL_QUADS);
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glEnd();
              glColor4ub(255,0,0,200);
              glLineWidth(10);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glEnd();
        }
          else if (AB == BC && BC == CD  && temp_area == temp_area1 ){// rhombus
              shape_name ="Rhombus";
              glColor4ub(0,255,255,200);
              glBegin(GL_QUADS);
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glEnd();
              glColor4ub(255,0,0,200);
              glLineWidth(10);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glEnd();
       } 
          else{
           glColor4ub(255,255,0,200);
           glBegin(GL_QUADS);
           glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
           glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
           glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
           glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
           glEnd();
         }
          if(outline_flag){
              glColor4ub(255,0,0,200);
              glLineWidth(lineWidth);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
          }
          glPopMatrix();
          //print text 
         if(shape_name!=""){
                  drawLetter(centered,shape_name,2,0.0);
          }
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawSideText(mA,mB);
          drawSideText(mB,mC);
          drawSideText(mC,mD);
          drawSideText(mA,mD);
          drawArea(centers); 
          break;
       case 5:
          unit_rg->enable();
          linear_cb->disable();
          linear_rg->disable();
          simulate_rg->disable();
          outline_cb->enable();
          grid_cb->disable();
          glBegin(GL_QUADS);
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glEnd();
          glBegin(GL_TRIANGLES);
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
          glEnd();
          if(outline_flag){
              glColor4ub(255,0,0,200);
              glLineWidth(lineWidth);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd();
              glEnable(GL_LINE_STIPPLE);
              glLineStipple(2,0xaaaa);
              glBegin(GL_LINES);
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
              glDisable(GL_LINE_STIPPLE);
          }
           glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawLetter(mE,lettermap[MarkerID[4]]);
          drawSideText(mA,mB);
          drawSideText(mB,mC);
          drawSideText(mC,mE);
          drawSideText(mE,mD);
          drawSideText(mD,mA);
          drawArea(centers); 
          break;
        case 6:
          unit_rg->enable();
          linear_cb->disable();
          linear_rg->disable();
          simulate_rg->disable();
          outline_cb->enable();
          grid_cb->disable();
          glBegin(GL_QUADS);
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
          glEnd();
          if(outline_flag){
              glColor4ub(255,0,0,200);
              glLineWidth(lineWidth);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
              glEnable(GL_LINE_STIPPLE);
              glLineStipple(2,0xaaaa);
              glBegin(GL_LINES);
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
              glDisable(GL_LINE_STIPPLE);
          }
           glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawLetter(mE,lettermap[MarkerID[4]]);
          drawLetter(mF,lettermap[MarkerID[5]]);
          drawSideText(mA,mB);
          drawSideText(mB,mC);
          drawSideText(mC,mF);
          drawSideText(mF,mE);
          drawSideText(mE,mD);
          drawSideText(mD,mA);
          drawArea(centers); 
          break;

        case 7:
          unit_rg->enable();
          linear_cb->disable();
          linear_rg->disable();
          outline_cb->enable();
          grid_cb->disable();
          if(!coin_flag){
              simulate_rg->disable();
              glColor4ub(255,255,0,200);
              glBegin(GL_QUADS);
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glEnd();
              glBegin(GL_QUADS);
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glEnd();
              glBegin(GL_TRIANGLES);
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
              glEnd();
              if(outline_flag){
                  glColor4ub(255,0,0,200);
                  glLineWidth(lineWidth);
                  glBegin(GL_LINES);      
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                  glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                  glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                  glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                  glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                  glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                  glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                  glEnd(); 
                  glEnable(GL_LINE_STIPPLE);
                  glLineStipple(2,0xaaaa);
                  glBegin(GL_LINES);
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                  glEnd(); 
                  glDisable(GL_LINE_STIPPLE);
              }
              drawSideText(mA,mB);
              drawSideText(mB,mC);
              drawSideText(mC,mG);
              drawSideText(mG,mF);
              drawSideText(mF,mE);
              drawSideText(mE,mD);
              drawSideText(mD,mA);
           }
          else{ //coin demonstration
              simulate_rg->enable();
              if(!simulate_flag){
                  glColor4ub(255,255,0,200);
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glEnd();
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                  glEnd();
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                  glEnd();
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                  glEnd();
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                  glEnd();
                  glBegin(GL_TRIANGLES);
                  glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                  glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                  glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                  glEnd();
                  if(outline_flag){
                      simulate_rg->disable();
                      glColor4ub(255,0,0,200);
                      glLineWidth(lineWidth);
                      glBegin(GL_LINES);      
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glEnd(); 
                      glEnable(GL_LINE_STIPPLE);
                      glLineStipple(2,0xaaaa);
                      glBegin(GL_LINES);      
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glEnd(); 
                      glDisable(GL_LINE_STIPPLE);
                  }
              }
              else if(aflag && simulate_flag){
                      simulate_rg->enable();
                      outline_cb->disable();
                      r_eA= calculateDistance(mA,eA,false)/2;
                      r_eB= calculateDistance(mB,eB,false)/2;
                      r_eD= calculateDistance(mD,eD,false)/2;
                      r_eE= calculateDistance(mE,eE,false)/2;
                      r_eF= calculateDistance(mF,eF,false)/2;
                      r_eG= calculateDistance(mG,eG,false)/2;
                      glColor4ub(255,255,255,200);
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eA.at<float>(0,0)+cos(i) *r_eA, eA.at<float>(1,0)+sin(i)*r_eA, -eA.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eB.at<float>(0,0)+cos(i) *r_eB, eB.at<float>(1,0)+sin(i)*r_eB, -eB.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eD.at<float>(0,0)+cos(i) *r_eD, eD.at<float>(1,0)+sin(i)*r_eD, -eD.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eE.at<float>(0,0)+cos(i) *r_eE, eE.at<float>(1,0)+sin(i)*r_eE, -eE.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eF.at<float>(0,0)+cos(i) *r_eF, eF.at<float>(1,0)+sin(i)*r_eF, -eF.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eG.at<float>(0,0)+cos(i) *r_eG, eG.at<float>(1,0)+sin(i)*r_eG, -eG.at<float>(2,0));
                      glEnd();
                      glColor4ub(255,0,0,200);
                      glEnable(GL_LINE_STIPPLE);
                      glLineStipple(2,0xaaaa);
                      glBegin(GL_LINES);      
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(eB.at<float>(0,0),eB.at<float>(1,0) ,-eB.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(eD.at<float>(0,0),eD.at<float>(1,0) ,-eD.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(eF.at<float>(0,0),eF.at<float>(1,0) ,-eF.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glVertex3f(eD.at<float>(0,0),eD.at<float>(1,0) ,-eD.at<float>(2,0));
                      glVertex3f(eD.at<float>(0,0),eD.at<float>(1,0) ,-eD.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(eF.at<float>(0,0),eF.at<float>(1,0) ,-eF.at<float>(2,0));
                      glVertex3f(eF.at<float>(0,0),eF.at<float>(1,0) ,-eF.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      glVertex3f(eB.at<float>(0,0),eB.at<float>(1,0) ,-eB.at<float>(2,0));
                      glVertex3f(eB.at<float>(0,0),eB.at<float>(1,0) ,-eB.at<float>(2,0));
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glEnd(); 
                      glDisable(GL_LINE_STIPPLE);
                      glColor4ub(255,0,0,200);
                      glLineWidth(lineWidth);
                      glBegin(GL_LINES);      
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glEnd(); 
                      glEnable(GL_LINE_STIPPLE);
                      glLineStipple(2,0xaaaa);
                      glBegin(GL_LINES);      
                      glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glEnd(); 
                      glDisable(GL_LINE_STIPPLE);
                 }
                 else if(bflag && simulate_flag){
                      simulate_rg->enable();
                      outline_cb->disable();
                      r_eA= calculateDistance(mA,eA,false)/2;
                      r_eB= calculateDistance(mB,eB,false)/2;
                      r_eD= calculateDistance(mD,eD,false)/2;
                      r_eE= calculateDistance(mE,eE,false)/2;
                      r_eF= calculateDistance(mF,eF,false)/2;
                      r_eG= calculateDistance(mG,eG,false)/2;
                      r_eAD= (calculateDistance(eA,eD,false)-r_eA-r_eD)/2;
                      r_eDE= (calculateDistance(eD,eE,false)-r_eD-r_eE)/2;
                      r_eEF= (calculateDistance(eE,eF,false)-r_eE-r_eF)/2;
                      r_eFG= (calculateDistance(eF,eG,false)-r_eF-r_eG)/2;
                      r_eGB= (calculateDistance(eG,eB,false)-r_eG-r_eB)/2;
                      r_eBA= (calculateDistance(eB,eA,false)-r_eB-r_eA)/2;
                      glColor4ub(255,255,255,200);
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eA.at<float>(0,0)+cos(i) *r_eA, eA.at<float>(1,0)+sin(i)*r_eA, -eA.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eB.at<float>(0,0)+cos(i) *r_eB, eB.at<float>(1,0)+sin(i)*r_eB, -eB.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eD.at<float>(0,0)+cos(i) *r_eD, eD.at<float>(1,0)+sin(i)*r_eD, -eD.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eE.at<float>(0,0)+cos(i) *r_eE, eE.at<float>(1,0)+sin(i)*r_eE, -eE.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eF.at<float>(0,0)+cos(i) *r_eF, eF.at<float>(1,0)+sin(i)*r_eF, -eF.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eG.at<float>(0,0)+cos(i) *r_eG, eG.at<float>(1,0)+sin(i)*r_eG, -eG.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eAD.at<float>(0,0)+cos(i) *r_eAD, eAD.at<float>(1,0)+sin(i)*r_eAD, -eAD.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eDE.at<float>(0,0)+cos(i) *r_eDE, eDE.at<float>(1,0)+sin(i)*r_eDE, -eDE.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eEF.at<float>(0,0)+cos(i) *r_eEF, eEF.at<float>(1,0)+sin(i)*r_eEF, -eEF.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eFG.at<float>(0,0)+cos(i) *r_eFG, eFG.at<float>(1,0)+sin(i)*r_eFG, -eFG.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eGB.at<float>(0,0)+cos(i) *r_eGB, eGB.at<float>(1,0)+sin(i)*r_eGB, -eGB.at<float>(2,0));
                      glEnd();
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(eBA.at<float>(0,0)+cos(i) *r_eBA, eBA.at<float>(1,0)+sin(i)*r_eBA, -eBA.at<float>(2,0));
                      glEnd();
                      glLineWidth(5);
                      glBegin(GL_LINES);      
                      glColor4ub(0,0,0,255);
                      glBegin(GL_LINES);      
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glVertex3f(eD.at<float>(0,0),eD.at<float>(1,0) ,-eD.at<float>(2,0));
                      glVertex3f(eD.at<float>(0,0),eD.at<float>(1,0) ,-eD.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(eF.at<float>(0,0),eF.at<float>(1,0) ,-eF.at<float>(2,0));
                      glVertex3f(eF.at<float>(0,0),eF.at<float>(1,0) ,-eF.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      glVertex3f(eB.at<float>(0,0),eB.at<float>(1,0) ,-eB.at<float>(2,0));
                      glVertex3f(eB.at<float>(0,0),eB.at<float>(1,0) ,-eB.at<float>(2,0));
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glVertex3f(eA.at<float>(0,0),eA.at<float>(1,0) ,-eA.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(eE.at<float>(0,0),eE.at<float>(1,0) ,-eE.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glVertex3f(eG.at<float>(0,0),eG.at<float>(1,0) ,-eG.at<float>(2,0));
                      glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
                      glEnd(); 
                 }
                 else if(cflag && simulate_flag){
                      simulate_rg->enable();
                      outline_cb->disable();
                      r_hAC= calculateDistance(mA,mC,false)/2;
                      r_hDC= calculateDistance(mD,mC,false)/2;
                      r_hBC= calculateDistance(mB,mC,false)/2;
                      r_hEC= calculateDistance(mE,mC,false)/2;
                      r_hFC= calculateDistance(mF,mC,false)/2;
                      r_hGC= calculateDistance(mG,mC,false)/2;
                      r_hAD= calculateDistance(mA,mD,false)/2;
                      r_hDE= calculateDistance(mD,mE,false)/2;
                      r_hEF= calculateDistance(mE,mF,false)/2;
                      r_hFG= calculateDistance(mF,mG,false)/2;
                      r_hGB= calculateDistance(mG,mB,false)/2;
                      r_hBA= calculateDistance(mB,mA,false)/2;
                      glColor4ub(0,0,0,255);
                      glLineWidth(3);
                      glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hAC.at<float>(0,0)+cos(deg)*r_hAC,hAC.at<float>(1,0)+sin(deg)*r_hAC,-hAC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hDC.at<float>(0,0)+cos(deg)*r_hDC,hDC.at<float>(1,0)+sin(deg)*r_hDC,-hDC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hBC.at<float>(0,0)+cos(deg)*r_hBC,hBC.at<float>(1,0)+sin(deg)*r_hBC,-hBC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hEC.at<float>(0,0)+cos(deg)*r_hEC,hEC.at<float>(1,0)+sin(deg)*r_hEC,-hEC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hFC.at<float>(0,0)+cos(deg)*r_hFC,hFC.at<float>(1,0)+sin(deg)*r_hFC,-hFC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hGC.at<float>(0,0)+cos(deg)*r_hGC,hGC.at<float>(1,0)+sin(deg)*r_hGC,-hGC.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hAD.at<float>(0,0)+cos(deg)*r_hAD,hAD.at<float>(1,0)+sin(deg)*r_hAD,-hAD.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hDE.at<float>(0,0)+cos(deg)*r_hDE,hDE.at<float>(1,0)+sin(deg)*r_hDE,-hDE.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hEF.at<float>(0,0)+cos(deg)*r_hEF,hEF.at<float>(1,0)+sin(deg)*r_hEF,-hEF.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hFG.at<float>(0,0)+cos(deg)*r_hFG,hFG.at<float>(1,0)+sin(deg)*r_hFG,-hFG.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hGB.at<float>(0,0)+cos(deg)*r_hGB,hGB.at<float>(1,0)+sin(deg)*r_hGB,-hGB.at<float>(2,0));
                      }
                      glEnd();
                     glBegin(GL_LINE_LOOP);
                      for(int i=0;i<360;i++){
                       float deg = i*PI/180;
                       glVertex3f(hBA.at<float>(0,0)+cos(deg)*r_hBA,hBA.at<float>(1,0)+sin(deg)*r_hBA,-hBA.at<float>(2,0));
                      }
                      glEnd();
                     /* glColor4ub(255,255,255,200);
                      glBegin(GL_POLYGON);
                      for(double i=0;i<2*PI;i+=PI/20) glVertex3f(hAC.at<float>(0,0)+cos(i) *r_hAC,
                          hAC.at<float>(1,0)+sin(i)*r_hAC, -hAC.at<float>(2,0));
                      glEnd();
                     */
                 }
              drawSideText(mA,mB);
              drawSideText(mB,mG);
              drawSideText(mG,mF);
              drawSideText(mF,mE);
              drawSideText(mE,mD);
              drawSideText(mD,mA);
          }
          glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawLetter(mE,lettermap[MarkerID[4]]);
          drawLetter(mF,lettermap[MarkerID[5]]);
          drawLetter(mG,lettermap[MarkerID[6]]);
          drawArea(centers);
          break;
       case 8:
          unit_rg->enable();
          outline_cb->enable();
          glBegin(GL_QUADS);
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
          glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
          glEnd();
          if(outline_flag){
              glColor4ub(255,0,0,200);
              glLineWidth(lineWidth);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
              glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
              glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
              glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
              glEnable(GL_LINE_STIPPLE);
              glLineStipple(2,0xaaaa);
              glBegin(GL_LINES);
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glEnd(); 
              glDisable(GL_LINE_STIPPLE);
          }
           glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawLetter(mE,lettermap[MarkerID[4]]);
          drawLetter(mF,lettermap[MarkerID[5]]);
          drawLetter(mG,lettermap[MarkerID[6]]);
          drawLetter(mH,lettermap[MarkerID[7]]);
          drawSideText(mA,mB);
          drawSideText(mB,mC);
          drawSideText(mC,mH);
          drawSideText(mH,mG);
          drawSideText(mG,mF);
          drawSideText(mF,mE);
          drawSideText(mE,mD);
          drawSideText(mD,mA);
          drawArea(centers); 
          break;
        case 9:
          unit_rg->enable();
          outline_cb->enable();
          glBegin(GL_QUADS);
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
          glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
          glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
          glEnd();
          glBegin(GL_QUADS);
          glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
          glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
          glVertex3f(mI.at<float>(0,0),mI.at<float>(1,0) ,-mI.at<float>(2,0));
          glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
          glEnd();
          if(outline_flag){
              glColor4ub(255,0,0,200);
              glLineWidth(lineWidth);
              glBegin(GL_LINES);      
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mI.at<float>(0,0),mI.at<float>(1,0) ,-mI.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mI.at<float>(0,0),mI.at<float>(1,0) ,-mI.at<float>(2,0));
              glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
              glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
              glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
              glVertex3f(mG.at<float>(0,0),mG.at<float>(1,0) ,-mG.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mE.at<float>(0,0),mE.at<float>(1,0) ,-mE.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mA.at<float>(0,0),mA.at<float>(1,0) ,-mA.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glEnd(); 
              glEnable(GL_LINE_STIPPLE);
              glLineStipple(2,0xaaaa);
              glBegin(GL_LINES);
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mD.at<float>(0,0),mD.at<float>(1,0) ,-mD.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mF.at<float>(0,0),mF.at<float>(1,0) ,-mF.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mB.at<float>(0,0),mB.at<float>(1,0) ,-mB.at<float>(2,0));
              glVertex3f(mC.at<float>(0,0),mC.at<float>(1,0) ,-mC.at<float>(2,0));
              glVertex3f(mH.at<float>(0,0),mH.at<float>(1,0) ,-mH.at<float>(2,0));
              glEnd(); 
              glDisable(GL_LINE_STIPPLE);
          }
           glPopMatrix();
          //print text
          drawLetter(mA,lettermap[MarkerID[0]]);
          drawLetter(mB,lettermap[MarkerID[1]]);
          drawLetter(mC,lettermap[MarkerID[2]]);
          drawLetter(mD,lettermap[MarkerID[3]]);
          drawLetter(mE,lettermap[MarkerID[4]]);
          drawLetter(mF,lettermap[MarkerID[5]]);
          drawLetter(mG,lettermap[MarkerID[6]]);
          drawLetter(mH,lettermap[MarkerID[7]]);
          drawLetter(mI,lettermap[MarkerID[8]]);
          drawSideText(mA,mB);
          drawSideText(mB,mI);
          drawSideText(mI,mH);
          drawSideText(mH,mG);
          drawSideText(mG,mF);
          drawSideText(mF,mE);
          drawSideText(mE,mD);
          drawSideText(mD,mA);
          drawArea(centers); 
          break;
        default: break;
   }
}
/************************************
 *
 *
 ************************************/
void vDrawScene()
{
  if (TheResizedImage.rows==0)// prevent from going on until the image is initialized
    return;
    //clear
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    //draw image in the buffer
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, TheGlWindowSize.width, 0, TheGlWindowSize.height, -1.0, 1.0);
    glViewport(0, 0, TheGlWindowSize.width , TheGlWindowSize.height);
    glDisable(GL_TEXTURE_2D);
    
    glPixelZoom( 1, -1);
    glRasterPos3f( 0, TheGlWindowSize.height  - 0.5, -1.0 );
    glDrawPixels ( TheGlWindowSize.width , TheGlWindowSize.height , GL_RGB , GL_UNSIGNED_BYTE , TheResizedImage.ptr(0) );
    ///Set the appropriate projection matrix so that rendering is done in a enrvironment
    //like the real camera (without distorsion)
    glMatrixMode(GL_PROJECTION);
    double proj_matrix[16];
    TheCameraParams.glGetProjectionMatrix(TheInputImage.size(),TheGlWindowSize,proj_matrix,0.05,10);
    glLoadIdentity();
    glLoadMatrixd(proj_matrix);
    
    //now, for each marker,
    double modelview_matrix[16];
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    vector<cv::Point2f> centers;
    for (unsigned int m=0;m<TheMarkers.size();m++)
    {
       centers.push_back(TheMarkers[m].getCenter());
    }
    
    char textString[100] = "Free Mode";
    char unitString[100] = "Metric Units";
    // detect Markers
    assignMarker(centers);
    //detectMarker(centers);
     if ( mode == Free){
      freeMode(centers,outline_flag);
      int a = sprintf(textString,"%s","Free Mode");
    } else if (mode == Grid){
      int a = sprintf(textString,"%s","Grid Mode");
      gridMode(centers);
    }
    int m =sprintf(unitString,"%s",imperialUnitFlag?"(in)":"(cm)");
     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        
     // anti-aliasing
     if(alias_flag){
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_POLYGON_SMOOTH);
      }
      else{
        glDisable(GL_POINT_SMOOTH);
        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_POLYGON_SMOOTH);
      }
    
    // Print Text : Mode + Unit
     if (centers.size() > 1){
      float xtranslateArea = -4*0.015;
      float ytranslateArea = 2.5*0.015;
      float x_area = mA.at<float>(0,0) + xtranslateArea;
      float y_area = mA.at<float>(1,0) - 1.75*ytranslateArea;
      float z_area = -mA.at<float>(2,0);
      glPushMatrix();
      glLoadIdentity();
      glColor4f(1.0f,0.0f,0.0f,0.2f);
      glTranslatef(x_area,y_area,z_area);
      glRasterPos3f( 0.0f, 0.0f, 0.0f);
      char buffer[100];
      int n = sprintf(buffer,"%s \n%s",textString,unitString);
      drawString(buffer);
      glPopMatrix();
    }

    // when Screen Capturing
    if(capture_flag){
      glClearColor(1.0,1.0,1.0,1.0);
      glClear(GL_COLOR_BUFFER_BIT);
      sleep(0.1); 
      capture_flag = false;
      system("mplayer ~/Documents/MathMAR/camera1.mp3");
    }
 
    glutSwapBuffers();
}
/************************************
 *
 *
 ************************************/
void vIdle()
{
    if (TheCaptureFlag) {
        //capture image
        TheVideoCapturer.grab();
        TheVideoCapturer.retrieve( TheInputImage);
        TheUndInputImage.create(TheInputImage.size(),CV_8UC3);
        //transform color that by default is BGR to RGB because windows systems do not allow reading BGR images with opengl properly
        cv::cvtColor(TheInputImage,TheInputImage,CV_BGR2RGB);
        //remove distorion in image
        cv::undistort(TheInputImage,TheUndInputImage, TheCameraParams.CameraMatrix, TheCameraParams.Distorsion);
        //detect markers
        PPDetector.detect(TheUndInputImage,TheMarkers, TheCameraParams.CameraMatrix,Mat(),TheMarkerSize);
        //resize the image to the size of the GL window
        cv::resize(TheUndInputImage,TheResizedImage,TheGlWindowSize);
    }
    glutSetWindow(window_id);
    glutPostRedisplay();
}
/************************************
 *
 *
 ************************************/
void vResize( GLsizei iWidth, GLsizei iHeight )
{
    TheGlWindowSize=Size(iWidth,iHeight);
    //not all sizes are allowed. OpenCv images have padding at the end of each line in these that are not aligned to 4 bytes
    if (iWidth*3%4!=0) {
        iWidth+=iWidth*3%4;//resize to avoid padding
        vResize(iWidth,TheGlWindowSize.height);
    }
    else {
        //resize the image to the size of the GL window
        if (TheUndInputImage.rows!=0)
            cv::resize(TheUndInputImage,TheResizedImage,TheGlWindowSize);
    }
}
/************************************
 *
 *
 ************************************/
int main(int argc,char **argv)
{
    try
    {//parse arguments
        if (readArguments (argc,argv)==false) return 0;
        //read from camera
        if (TheInputVideo=="live"){
            TheVideoCapturer.open(0); // change number 0 or 1
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH,1280);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT,720);
        }
        else TheVideoCapturer.open(TheInputVideo);
        if (!TheVideoCapturer.isOpened())
        {
            cerr<<"Could not open video"<<endl;
            return -1;

        }
        lettermap[255] = "I";
        lettermap[666] = "H";
        lettermap[771] = "C";
        lettermap[787] = "G";
        lettermap[816] = "F";
        lettermap[819] = "E";
        lettermap[922] = "A";
        lettermap[923] = "B";
        lettermap[939] = "D";
        lettermap[508] = "J";
        lettermap[184] = "K";
        lettermap[1] = "L";
        lettermap[409] = "M";
        lettermap[938] = "N";
        lettermap[855] = "O";
        lettermap[943] = "P"; 
        lettermap[937] = "Q"; 
        lettermap[942] = "R"; 
        lettermap[281] = "S"; 
        lettermap[511] = "T"; 

        //read first image
        TheVideoCapturer>>TheInputImage;
        //read camera paramters if passed
        TheCameraParams.readFromXMLFile(TheIntrinsicFile);
        TheCameraParams.resize(TheInputImage.size());
        
        init();
        glutInit(&argc, argv);
        glutInitWindowPosition( 0, 0);
        glutInitWindowSize(TheInputImage.size().width,TheInputImage.size().height);
        glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE );
        window_id = glutCreateWindow( "MathMAR" );
        //glutFullScreen();
        
        glui_button = GLUI_Master.create_glui_subwindow(window_id,GLUI_SUBWINDOW_BOTTOM);
        show_btn = new GLUI_Button(glui_button,"Show MENU",2,vButton);
        for(int i=1;i<90;i++)glui_button->add_column(false);
        quit_btn = new GLUI_Button(glui_button,"Quit",0,exit);
        glui_button->set_main_gfx_window( window_id);
        
        glui_subwin = GLUI_Master.create_glui_subwindow(window_id,GLUI_SUBWINDOW_RIGHT);
        glui_subwin->set_main_gfx_window( window_id);
        glui_subwin->add_statictext("");
        hide_btn = new GLUI_Button(glui_subwin,"Hide",3,vButton);
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Grid");
        grid_cb = new GLUI_Checkbox(glui_subwin,"On",&mode_grid,-1,vMenu);
        //glui_subwin->add_checkbox("Enabled",&mode_type,-1,vMenu);
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Linear");
        linear_cb = new GLUI_Checkbox(glui_subwin,"On",&mode_linear,-1,vMenu);
        linear_rg = new GLUI_RadioGroup(glui_subwin,&linear_type,-1,vMenu);
            new GLUI_RadioButton( linear_rg, "X" );
            new GLUI_RadioButton( linear_rg, "Y" );
            new GLUI_RadioButton( linear_rg, "Z" );
            new GLUI_RadioButton( linear_rg, "None" );
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Unit");
        unit_rg = new GLUI_RadioGroup(glui_subwin,&unit_type,-1,vMenu);
            new GLUI_RadioButton( unit_rg, "cm" );
            new GLUI_RadioButton( unit_rg, "in" );
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();
        
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Outline");
        outline_cb = new GLUI_Checkbox(glui_subwin,"On",&outline_type,-1,vMenu);
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();

        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Simulate");
        simulate_rg = new GLUI_RadioGroup(glui_subwin,&simulate_type,-1,vMenu);
            new GLUI_RadioButton( simulate_rg, "A" );
            new GLUI_RadioButton( simulate_rg, "B" );
            new GLUI_RadioButton( simulate_rg, "C" );
            new GLUI_RadioButton( simulate_rg, "None" );
//        glui_subwin->add_statictext("");
        glui_subwin->add_separator();
          
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("Screen");
        capture_btn = new GLUI_Button(glui_subwin,"Capture",1,vButton);
        glui_subwin->add_separator();
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("");
        glui_subwin->add_statictext("");
        quit_btn = new GLUI_Button(glui_subwin,"Quit",0,exit);

        glui_subwin->hide();
        glutDisplayFunc( vDrawScene );
        //glutIdleFunc( vIdle );
        GLUI_Master.set_glutIdleFunc( vIdle);
        glutReshapeFunc( vResize );
        //glutMouseFunc(vMouse);
        glutKeyboardFunc(vKeyboard);
        glClearColor( 0.0, 0.0, 0.0, 1.0 );
        glClearDepth( 1.0 );
        TheGlWindowSize=TheInputImage.size();
        vResize(TheGlWindowSize.width,TheGlWindowSize.height);
        glutMainLoop();
        
    } catch (std::exception &ex)

    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}
