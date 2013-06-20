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

void drawSideTextTranslate(cv::Mat t3,cv::Mat t2, float unit){
The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#ifdef __APPLE__
#include <gl.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif
#include "aruco.h"
using namespace cv;
using namespace aruco;
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

void vMenu(int value);
void vDrawScene();
void vIdle();
void vResize( GLsizei iWidth, GLsizei iHeight );
void vMouse(int b,int s,int x,int y);
void vKeyboard(unsigned char key,int x,int y);
//void lineMode(cv::<Point2f> center);
float calculateTriangleArea(cv::Mat t0, cv::Mat t1, cv::Mat t2);

//Enumeration for modes
enum Mode{
  Free,
  Triangle,
  Grid,
  Line,
  Line2
};

bool imperialUnitFlag = false;
bool xflag = false;
bool yflag = false;
bool zflag = false;
bool alias_flag = false;
float M2CM = 100.0f;
float M2IN = 39.3700787f;

Mode mode;

void init(){
  mode = Free;
}

/************************************
 *
 *
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


/************************************
 *
 *
 *
 *
 ************************************/

int main(int argc,char **argv)
{
    try
    {//parse arguments
        if (readArguments (argc,argv)==false) return 0;
        //read from camera
	//Deepak changed to 1 from 0
        if (TheInputVideo=="live"){
            TheVideoCapturer.open(0);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH,1280);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT,720);
        }
        else TheVideoCapturer.open(TheInputVideo);
        if (!TheVideoCapturer.isOpened())
        {
            cerr<<"Could not open video"<<endl;
            return -1;

        }
        lettermap[922] = "A";
        lettermap[923] = "B";
        lettermap[771] = "C";
        lettermap[939] = "D";
        lettermap[819] = "E";
        lettermap[816] = "F";
        lettermap[787] = "G";
        lettermap[666] = "H";
        lettermap[509] = "I";

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
        glutCreateWindow( "MathMAR" );
        
        // Main Menu by Chan
        glutCreateMenu( vMenu );
        glutAddMenuEntry("Free Mode",1);
        glutAddMenuEntry("Grid Mode",2);
        glutAddMenuEntry("Line Mode",3);
        glutAddMenuEntry("Change Unit",4);
        glutAddMenuEntry("Change Alias",5);
        glutAttachMenu(GLUT_LEFT_BUTTON);


        glutDisplayFunc( vDrawScene );
        glutIdleFunc( vIdle );
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
/************************************
 * Mode Selection using Touch
 *
 *
*************************************/
void vMenu(int value){
    switch(value){
        case 1:
            mode = Free;
            cout << "I am in the default free mode" << endl;
            break;
        case 2:
            mode = Grid;
            cout << "I am in the default free mode" << endl;
            break;
        case 3:
            mode = Line2;
            xflag = !xflag;
            cout << "I am in the default free mode" << endl;
            break;
        case 4:
            imperialUnitFlag = !imperialUnitFlag;
            break;
        case 5:
            alias_flag = !alias_flag;
        }
    glutPostRedisplay();
}

/************************************
 *Mode Selection using Keyboard
 *
 *
 *
 ************************************/
void vKeyboard(unsigned char key,int x,int y){
  if (key == 't'){
    mode = Triangle;
    cout << "I am in the triangle exploration mode" << endl;
    } else if (key == 'g'){
    mode = Grid;
    cout << "I am in the grid mode" << endl;
  } else if (key == 'l'){
    mode = Line;
    cout << "I am in the line mode" << endl;
  } else if (key == 'x'){
    mode = Line2;
    xflag = !xflag;
    cout << "I am in the line mode2" << endl;
    //glutPostRedisplay();
  }
  else if (key == 'y') {
     mode = Line2;
     yflag = !yflag;
  }
  else if (key == 'z') {
     mode = Line2;
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
/************************************
 *
 *
 *
 *
 ************************************/

void vMouse(int b,int s,int x,int y)
{
//    if (b==GLUT_LEFT_BUTTON && s==GLUT_DOWN) {
//        TheCaptureFlag=!TheCaptureFlag;
//    }
}

/************************************
 *
 *
 *
 ************************************/
float convertD(float distance){
   return imperialUnitFlag?(M2IN*distance):(M2CM*distance);
}
void convertDistance(float *distance){
  *distance = imperialUnitFlag?(M2IN**distance):(M2CM**distance);
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
int calculatePerimeter() {
    int perimeter = 0;
  cv::Mat t0 = TheMarkers[0].Tvec;
  std::cout << "TheMarker[0] index: " << TheMarkers[0].id;
  cv::Mat t1 = TheMarkers[1].Tvec;
  std::cout << "The Marker[1].index: " << TheMarkers[1].id;
  cv::Mat t2 = TheMarkers[2].Tvec;
  std::cout << "TheMarkers[2].index: " << TheMarkers[2].id;

  cv::Mat t3;
  if (TheMarkers.size() == 4) {
     std::cout << "The Markers[3] index: " << TheMarkers[3].id;
     t3 = TheMarkers[3].Tvec;
     t2 = TheMarkers[1].Tvec;
     t1 = TheMarkers[2].Tvec;
 // }
    //for (int i = 0; i < 3; i++) {
   // if (TheMarkers.size() == 4) {
    int length = floor (calculateDistance(t0,t1)+0.5);
    int width = floor (calculateDistance(t1,t2)+0.5);
    int len2 = floor (calculateDistance(t2, t3)+0.5);
    int wid2 = floor (calculateDistance(t3, t0)+0.5); 
    perimeter += length + len2 + width + wid2;
    }
    if (TheMarkers.size() == 3) {
    perimeter += floor (calculateDistance(t0,t1)+0.5);
    perimeter += floor (calculateDistance(t1,t2)+0.5);
    perimeter += floor (calculateDistance(t2,t0)+0.5);;
    }
    return perimeter;
}
float calculateTriangleArea(cv::Mat t0, cv::Mat t1, cv::Mat t2){
    float side1 = calculateDistance(t0,t1,false);
    float side2 = calculateDistance(t1,t2,false);
    float side3 = calculateDistance(t0,t2,false);
    float perimeter = side1 + side2 + side3;
    float s = perimeter / 2;
    float area = sqrt(s * (s - side1) * (s - side2) * (s - side3));
    return area;
}

float calculateArea(){
  float area = 0;
  
  cv::Mat t0 = TheMarkers[0].Tvec;
  cv::Mat t1 = TheMarkers[2].Tvec;
  cv::Mat t2 = TheMarkers[1].Tvec;
  	
  std::cout << "TheMarkers[0].index: " << TheMarkers[0].id << std::endl;
  std::cout << "TheMarkers[1].index: " << TheMarkers[1].id << std::endl;
  std::cout << "TheMarkers[2].index: " << TheMarkers[2].id << std::endl;

  int sz =  TheMarkers.size();

  //If the input is a square  
  if (TheMarkers.size() == 4) {
    cv::Mat t3 = TheMarkers[3].Tvec;
    float a = calculateDistance(t0,t1);
    float b = calculateDistance(t1,t2);
    float c = calculateDistance(t2, t3);
    float d = calculateDistance(t3, t0);
    float p = calculateDistance(t0,t2);
    float q = calculateDistance(t1, t3);
    float s = (a + b + c + d)/2;
    area = sqrt((s-a)*(s-b)*(s-c)*(s-d) - (1/4)*(a*c + b*d + p*q)*(a*c+b*d-p*q));   
  }
  //The input is a triangle
  else if (TheMarkers.size() == 3){
    float side1 = calculateDistance(t0,t1);
    float side2 = calculateDistance(t1,t2);
    float side3 = calculateDistance(t0,t2);
    float perimeter = side1 + side2 + side3;
    float s = perimeter / 2;
    area = sqrt(s * (s - side1) * (s - side2) * (s - side3));
  }
  return area;
}

void drawString(char* string, int sub=0){

  char *c;
  for (c=string; *c != '\0'; c++) 
    {
      if (!sub) {
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
      // glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *c);
      }
      else {
      //glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, *c);
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
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
  if (black) {
  glColor4ub(28,248,255,128);
  }
  else {
  glColor4ub(28,248,255,128); 
  }

  glLoadIdentity();
glTranslatef(xcoordinate, ycoordinate, zcoordinate);//x_bmid,y_bmid,z_bmid);
if (dist !=  0.0) {
glTranslatef(0.0, dist, 0.0);
}
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawStringLetter(letter);
  
glPopMatrix();
  #ifdef DEBUG
  cout << "Drawing a character"<<endl;
  #endif
}

float calcSlope(cv::Mat t0, cv::Mat t1);

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
/* 
  float x_bmid2 = (3*t3.at<float>(0,0) + t2.at<float>(0,0))/4;
  float y_bmid2 = (3*t3.at<float>(1,0) + t2.at<float>(1,0))/4;
  float z_bmid2 = (-3*t3.at<float>(2,0) - t2.at<float>(2,0))/4;
  float x_bmid3 = (t3.at<float>(0,0) +3*t2.at<float>(0,0))/4;
  float y_bmid3 = (t3.at<float>(1,0) +3*t2.at<float>(1,0))/4;
  float z_bmid3 = (-3*t3.at<float>(2,0) - t2.at<float>(2,0))/4;
*/   

  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(x_bmid,y_bmid,z_bmid);
  glRasterPos3f( 0.0f, 0.0f, 0.0f );
  drawString(buffer);
  
/*  glLineWidth(5);
  glBegin(GL_LINES);
  glVertex3f(t3.at<float>(0,0),t3.at<float>(1,0),-t3.at<float>(2,0));
  glVertex3f(x_bmid2,y_bmid2,z_bmid2);
  glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0),-t2.at<float>(2,0));
  glVertex3f(x_bmid3,y_bmid3,z_bmid3);
  glEnd();
*/
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
/*
  float s = calculateDistance(t3,t2);
*/
 // char buffer[50];
 // int n = sprintf(buffer, letter);
/* 
 if (imperialUnitFlag){
    int n = sprintf(buffer,"%.0f in\n",floor(s+0.5));
  } else{
    int n = sprintf(buffer,"%.0f cm\n",floor(s+0.5));
  }
*/
//  float xtranslateArea = -4*0.015;
//  float ytranslateArea = 10*0.015;
    float xcoordinate = tnaught.at<float>(0,0);
    float ycoordinate = tnaught.at<float>(1,0);
    float zcoordinate = -tnaught.at<float>(2,0);

//  float xcoordinate = tnaught.at<float>(0,0)
//  float zcoordinate = -tnaught.at<float>(2,0)

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
  int perimeter = calculatePerimeter();
  float area = calculateArea();
  char buffer[50];
  char smallbuffer[50]; 
  char buffer2[50]; 
  if (imperialUnitFlag){
    int n = sprintf(buffer,"Area = %4.0f in",floor(area+0.5));
    int m = sprintf(smallbuffer, "2\n");
    int l = sprintf(buffer2,"Perimeter = %i in\n", perimeter);
  }
  else{
    int n = sprintf(buffer,"Area = %4.0f cm\n",floor(area+0.5));
    int m = sprintf(smallbuffer,"2\n");
    int l = sprintf(buffer2,"Perimeter = %i cm\n", perimeter);
  }

  float x_area = 0;
  float y_area = 0;
  float z_area = 0;
  float y_area2 = 0;
  float xtranslateArea = -4*0.015;
  float ytranslateArea = 2.5*0.015;
  
  if (centers.size() == 3 || centers.size() ==4 ){
    cv::Mat t0 = TheMarkers[0].Tvec;//C
    cv::Mat t1 = TheMarkers[2].Tvec;//B
    cv::Mat t2 = TheMarkers[1].Tvec;//A
    cv::Mat t3;
    if(centers.size() ==4){t3= TheMarkers[3].Tvec;}//D
    x_area = t2.at<float>(0,0) + xtranslateArea;
    y_area = t2.at<float>(1,0) - 0.75*ytranslateArea;
    z_area = -t2.at<float>(2,0);
    y_area2 = t2.at<float>(1,0) -1.25*ytranslateArea;
 } 
  glPushMatrix();
  glColor3f(0,0,1);
  glLoadIdentity();
  glTranslatef(x_area,y_area2,z_area);
  glRasterPos3f( 0.0f, 0.0f, 0.0f);
  drawString(buffer);
  drawString(smallbuffer, 1);
  glPopMatrix();

  glPushMatrix();
  glLoadIdentity();  
  glTranslatef(x_area,y_area,z_area);
  glRasterPos3f(0.0f,0.0f,0.0f);
  drawString(buffer2);
  glPopMatrix();
  
}

void triangleMode(vector<cv::Point2f> centers){
  float length = 0;
  float height = 0;
  
  
  if (centers.size() == 3){
    cv::Mat t0 = TheMarkers[0].Tvec;
    cv::Mat t1 = TheMarkers[1].Tvec;
    cv::Mat t2 = TheMarkers[2].Tvec;
    
    length = calculateDistance(t2,t1);
    height = calculateDistance(t0,t2);
    
    glColor3f(1,1,0);
    glPushMatrix();
    glLoadIdentity();
    
    glBegin(GL_TRIANGLES);
    glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
    glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
    glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
    glEnd();

    glColor3f(0,1,1);
    glEnd();

    glPopMatrix();
    //drawing char representations
    drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
    drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);//.idletter);
    drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);

    drawSideText(t0,t1);
    drawSideText(t1,t2);
    drawSideText(t0,t2);
    drawArea(centers);  
  }
}

float calcSlope(cv::Mat t0, cv::Mat t1) {
   // cv::Mat t0 = TheMarkers[0].Tvec;
   // cv::Mat t1 = TheMarkers[1].Tvec;
    float run = t0.at<float>(0,0) - t1.at<float>(0,0);
    float rise = t0.at<float>(1,0) - t1.at<float>(1,0);
 
//   convertDistance(&run);
 //   convertDistance(&rise);
     float slope = rise/run;
     
    return slope;
}



void lineMode(vector<cv::Point2f> centers){
  
  float translateDistance = -0.055;
  float lineWidth = 2;
  
  if (centers.size() == 3){
    cv::Mat t0 = TheMarkers[0].Tvec;
    cv::Mat t1 = TheMarkers[1].Tvec;
    cv::Mat t2 = TheMarkers[2].Tvec;

    glPushMatrix();
    glLoadIdentity();
    glColor3f(1,1,0);
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
     
    glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
    glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));

    glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
    glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
    //glEnd();
    glTranslatef(0.0f,translateDistance,0.0f);
    
    glBegin(GL_LINES);
    glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
    glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
    glEnd();
    
    glPopMatrix();
    for (int i = 0; i < 3; i++) {
       if (TheMarkers[i].id == 922) {
           drawLetter(TheMarkers[i].Tvec, "A");
       }
       else if (TheMarkers[i].id == 771) {
           drawLetter(TheMarkers[i].Tvec, "C");
       }
       else if (TheMarkers[i].id == 923) {
           drawLetter(TheMarkers[i].Tvec, "B");
       }
       else if (TheMarkers[i].id == 939) {
           drawLetter(TheMarkers[i].Tvec, "D"); 
       }
     }
//    drawLetter(TheMarkers[0].Tvec, "A");
//    drawLetter(TheMarkers[1].Tvec, "B");
//    drawLetter(TheMarkers[2].Tvec, "C");
    drawSideText(t0,t1);
    drawSideText(t1,t2);
    drawSideTextTranslate(t0,t2,translateDistance);
    
  }
  
}

void lineMode2(vector<cv::Point2f> centers){
  
  float translateDistance = -0.055;
  float translateLetterDifference = -0.1;
  float lineWidth = 2;
  
  if (centers.size() == 3){
    cv::Mat t0 = TheMarkers[0].Tvec;
    cv::Mat t1 = TheMarkers[2].Tvec;
    cv::Mat t2 = TheMarkers[1].Tvec;

    glPushMatrix();
    glLoadIdentity();
    glColor3f(1,1,0);
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    
    glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
    glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));

    glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
    glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
    
    glEnd();
    
    glTranslatef(0.0f,translateDistance,0.0f);
    
    glBegin(GL_LINES);
    glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
    glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
    glEnd();
    glBegin(GL_LINES);
    glTranslatef(0.0f,translateDistance,0.0f);
    glTranslatef(0.0f,translateDistance,0.0f);
    glEnd();
    glPopMatrix();
 // if (xflag) {
 //    glPushMatrix();
 //    glLoadIdentity();
 //    glTranslatef(0.0f,translateDistance,0.0f);
  //   drawLetter(TheMarkers[2].Tvec, "C", 1);
  //  drawLetter(TheMarkers[0].Tvec, "A", 1);
 //}  
//  glPushMatrix();
 // glColor3f(1,1,1);
 // glLoadIdentity();
// glTranslatef(0.0f,translateDistance,0.0f);
  //glRasterPos3f( 0.0f, 0.0f, 0.0f );
  //drawLetter(t2, "C", 1);
 //drawLetter(t0, "A", 1);
  //glColor3f(0,0,0);
//  glPopMatrix();
// }
    
    //drawSideText(t0,t1);
    
    if (xflag){
      drawX(t1,t2, 'X');
    }
    else {
      drawSideText(t1, t2);
    }
    if (yflag) {
      drawX(t0,t1,'Y');
    }
    else {
      drawSideText(t0,t1);
    }
    drawSideTextTranslate(t0,t2,translateDistance);
    drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
    drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);
    drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
 //   glTranslatef(0.0f,translateDistance,0.0f);
if (xflag) {
   // if (TheMarkers[2].id == 771) {
    drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id], 1, translateDistance);
  //  }
  //  if (TheMarkers[0].id == 922) {
    drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id], 1, translateDistance);
  //  }
}
 //   if (xflag) {
 //       drawLetterOnXTranslate(TheMarkers[0].Tvec, "A", translateDistance);
 //       drawLetterOnXTranslate(TheMarkers[2].Tvec, "C", translateDistance);
 //   }
                
 
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

// available mode only for triangle and quadranglie
void gridMode(vector<cv::Point2f> centers){
  //triangle
  if (centers.size() == 3){
    cv::Mat t0 = TheMarkers[0].Tvec; //C
    cv::Mat t1 = TheMarkers[2].Tvec; //B
    cv::Mat t2 = TheMarkers[1].Tvec; //A

   // Bottom line (B-C)
   float cos = getCos(t1,t0);
   float sin = getSin(t1,t0);
   // calculate left bottom & right bottom (+-2)
   float BC = calculateDistance(t1,t0,false);
   float AB = calculateDistance(t1,t2,false);
   float AC = calculateDistance(t0,t2,false);
   float Area = calculateTriangleArea(t0,t1,t2);
   float H = 2*Area/BC;
   float A[2],B[2],C[2];
   A[0]=t2.at<float>(0,0);
   A[1]=t2.at<float>(1,0);
   B[0]=t1.at<float>(0,0);
   B[1]=t1.at<float>(1,0);
   C[0]=t0.at<float>(0,0);
   C[1]=t0.at<float>(1,0);
   float lb[2],lu[2],rb[2],ru[2];
   
   // A
   //   B   C
   if (A[0]<B[0]){
       lu[0] = A[0];
       lu[1] = A[1];
       lb[0] = B[0]+cal_bottom_len(AB,H)*cos;
       lb[1] = B[1]+cal_bottom_len(AB,H)*sin;
       ru[0] = A[0]+(C[0]-lb[0]);
       ru[1] = A[1]+(C[1]-lb[1]);
       rb[0] = C[0];
       rb[1] = C[1];
   }
   //        A
   //  B   C
   else if (A[0] > C[0]){
       lb[0] = B[0];
       lb[1] = B[1];
       ru[0] = A[0];
       ru[1] = A[1];
       rb[0] = C[0]-cal_bottom_len(AC,H)*cos;
       rb[1] = C[1]-cal_bottom_len(AC,H)*sin;
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
       lu[0] = A[0]+cal_bottom_len(AB,H)*cos;
       lu[1] = A[1]+cal_bottom_len(AB,H)*sin;
       ru[0] = A[0]-cal_bottom_len(AC,H)*cos;
       ru[1] = A[1]-cal_bottom_len(AC,H)*sin;
   } 
       
   int cols = floor(convertD(H)+0.5);
   float col_unit = H/cols;
   #ifdef DEBUG
   cout <<"A : " << A[0] <<"," <<A[1] << " B : " << B[0]<<","<<B[1]<< " C : "<< C[0]<<","<<C[1]<<endl;
   cout <<"lu : " << lu[0] <<"," <<lu[1] << " lb : " << lb[0]<<","<<lb[1]<< " rb : "<< rb[0]<<","<<rb[1]<< " ru : " <<
   ru[0] <<","<<ru[1]<<endl;
   #endif
   int rows = floor(convertD(sqrt((rb[0]-lb[0])*(rb[0]-lb[0])+(rb[1]-lb[1])*(rb[1]-lb[1])))+0.5);
   // calculate left top & right top
   cout << "col_unit : "<< col_unit  << " cols : " << cols << endl;
   
   // draw yellow area
   glColor4ub(255,255,0,200);
   glPushMatrix();
   glLoadIdentity();
   glBegin(GL_TRIANGLES);
   glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
   glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
   glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
   glEnd();
   glPopMatrix();
   
   GLfloat grid2x2[12] = {rb[0],rb[1],-t0.at<float>(2,0),lb[0],lb[1],-t1.at<float>(2,0),ru[0],ru[1],-t0.at<float>(2,0),lu[0],lu[1],-t0.at<float>(2,0)};

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
   std::cout << rows << " rows" << std::endl;
   std::cout << cols << " columns" << std::endl; 
   glMapGrid2f(ceil(rows), 0.0, 1.0,
            ceil(cols), 0.0, 1.0);
        glLineWidth(2); 
        glEvalMesh2(GL_LINE,
              0, ceil(rows),   // Starting at 0 mesh 5 steps (rows). 
              0, ceil(cols));  // Starting at 0 mesh 6 steps (columns).
    glPopMatrix();
    
    //Draw Text 
    drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
    drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
    drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);
    drawSideText(t2,t0);
    drawSideText(t0,t1);
    drawSideText(t1,t2);
    drawArea(centers);    
  }
  //quadrangle
  else if (centers.size() == 4){
    cv::Mat t0 = TheMarkers[0].Tvec; //C
    cv::Mat t1 = TheMarkers[2].Tvec; //B
    cv::Mat t2 = TheMarkers[1].Tvec; //A
    cv::Mat t3 = TheMarkers[3].Tvec; //D
   // Bottom line (B-C)
   float cos = getCos(t1,t0);
   float sin = getSin(t1,t0);
   // length of each side
   float BC = calculateDistance(t1,t0,false);
   float AB = calculateDistance(t1,t2,false);
   float AC = calculateDistance(t0,t2,false);
   float CD = calculateDistance(t0,t3,false);
   float Area1 = calculateTriangleArea(t0,t1,t2);
   float Area2 = calculateTriangleArea(t0,t3,t1);
   float H1 = 2*Area1/BC;
   float H2 = 2*Area2/BC;

   float A[2],B[2],C[2],D[2];
   A[0]=t2.at<float>(0,0);
   A[1]=t2.at<float>(1,0);
   B[0]=t1.at<float>(0,0);
   B[1]=t1.at<float>(1,0);
   C[0]=t0.at<float>(0,0);
   C[1]=t0.at<float>(1,0);
   D[0]=t3.at<float>(0,0);
   D[1]=t3.at<float>(1,0);

   float lb[2],lu[2],rb[2],ru[2];
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
   int cols= floor(convertD(H1)+0.5);
   float col_unit = H1/cols;
   #ifdef DEBUG
   cout <<"A : " << A[0] <<"," <<A[1] << " B : " << B[0]<<","<<B[1]<< " C : "<< C[0]<<","<<C[1]<<endl;
   cout <<"lu : " << lu[0] <<"," <<lu[1] << " lb : " << lb[0]<<","<<lb[1]<< " rb : "<< rb[0]<<","<<rb[1]<< " ru : " <<
   ru[0] <<","<<ru[1]<<endl;
   // calculate left top & right top
   cout << "col_unit : "<< col_unit  << " cols : " << cols << endl;
  #endif
   int rows = floor(convertD(sqrt((rb[0]-lb[0])*(rb[0]-lb[0])+(rb[1]-lb[1])*(rb[1]-lb[1])))+0.5);
  
   // draw yellow area
   glColor4ub(255,255,0,200);
   glPushMatrix();
   glLoadIdentity();
   glBegin(GL_QUADS);
   glVertex3f(t3.at<float>(0,0),t3.at<float>(1,0) ,-t3.at<float>(2,0));
   glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
   glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
   glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
   glEnd();
   glPopMatrix();
   
   GLfloat grid2x2[12] = {rb[0],rb[1],-t0.at<float>(2,0),lb[0],lb[1],-t1.at<float>(2,0),ru[0],ru[1],-t0.at<float>(2,0),lu[0],lu[1],-t0.at<float>(2,0)};

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
   std::cout << rows << " rows" << std::endl;
   std::cout << cols << " columns" << std::endl; 
   glMapGrid2f(ceil(rows), 0.0, 1.0,
            ceil(cols), 0.0, 1.0);
        glLineWidth(2); 
        glEvalMesh2(GL_LINE,
              0, ceil(rows),   // Starting at 0 mesh 5 steps (rows). 
              0, ceil(cols));  // Starting at 0 mesh 6 steps (columns).
    glPopMatrix();
    
    //Draw Text 
    drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
    drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
    drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);
    drawLetter(TheMarkers[3].Tvec, lettermap[TheMarkers[3].id]);
    drawSideText(t3,t2);
    drawSideText(t0,t3);
    drawSideText(t0,t1);
    drawSideText(t1,t2);
    drawArea(centers);    
  }
}
/*
//randomize marker
   float y_value[size];
   int y_order[size]; // marker order
   float x_value[size];
   int x_order[size];// choose bottom points
   for(int i=0;i<size;i++){
     y_value[i] = grid2x2[3*i+1];
     x_value[i] = grid2x2[3*i];
   }
   std::sort(& y_value[0],& y_value[size]);
   std::sort(& x_value[0],& x_value[size]);
   for(int i=0;i<size;i++){
     for(int j=0;j<size;j++){
       if(grid2x2[3*j+1]==y_value[i]){y_order[i]=j;}
       if(grid2x2[3*j]==x_value[i]){x_order[i]=j;}
     }    
   }
   cout << "order" << y_order[0] << " " << y_order[1] << " " <<y_order[2] <<" " << y_order[3] <<endl;
   cout << "number" << y_value[0] << " " << y_value[1] << " " <<y_value[2] <<" " << y_value[3] <<endl;
*/   


void freeMode(vector<cv::Point2f> centers){
    if (centers.size() == 4){
 
      glColor4ub(255,255,0,200);
      glPushMatrix();
      glLoadIdentity();
      glBegin(GL_QUADS);
      cv::Mat t0 = TheMarkers[0].Tvec;//C
      cv::Mat t2 = TheMarkers[1].Tvec;//A
      cv::Mat t1 = TheMarkers[2].Tvec;//B
      cv::Mat t3 = TheMarkers[3].Tvec;//D
      glVertex3f(t3.at<float>(0,0),t3.at<float>(1,0) ,-t3.at<float>(2,0));
      glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
      glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
      glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
      glEnd();
      glPopMatrix();

      //text graphic
      drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
      drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
      drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);
      drawLetter(TheMarkers[3].Tvec, lettermap[TheMarkers[3].id]); 
      drawSideText(t3,t2);
      drawSideText(t0,t3);
      drawSideText(t0,t1);
      drawSideText(t1,t2);
      drawArea(centers);  
      
    }
    else if (centers.size() == 3){
   
      glColor4ub(255,255,0,200);
      glPushMatrix();
      glLoadIdentity();
      glBegin(GL_TRIANGLES);
      cv::Mat t0 = TheMarkers[0].Tvec;
      cv::Mat t1 = TheMarkers[1].Tvec;
      cv::Mat t2 = TheMarkers[2].Tvec;
      glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
      glVertex3f(t2.at<float>(0,0),t2.at<float>(1,0) ,-t2.at<float>(2,0));
      glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
      glEnd();
      glPopMatrix();
      drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
      drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]);
      drawLetter(TheMarkers[2].Tvec, lettermap[TheMarkers[2].id]);
 
       drawSideText(t0,t1);
       drawSideText(t1,t2);
       drawSideText(t0,t2);
       drawArea(centers);
      
    }

    else if (centers.size() == 2){

      glColor4ub(255,255,0,200);
      glPushMatrix();
      glLoadIdentity();
      glBegin(GL_LINES);
      cv::Mat t0 = TheMarkers[0].Tvec;
      cv::Mat t1 = TheMarkers[1].Tvec;
      glVertex3f(t0.at<float>(0,0),t0.at<float>(1,0) ,-t0.at<float>(2,0));
      glVertex3f(t1.at<float>(0,0),t1.at<float>(1,0) ,-t1.at<float>(2,0));
      glEnd();
      glPopMatrix();
      drawSideText(t0,t1);
      drawLetter(TheMarkers[0].Tvec, lettermap[TheMarkers[0].id]);
      drawLetter(TheMarkers[1].Tvec, lettermap[TheMarkers[1].id]); 
    }
}


/************************************
 *
 *
 *
 *
 ************************************/
void vDrawScene()
{
  if (TheResizedImage.rows==0)// prevent from going on until the image is initialized
        return;
    ///clear
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    //glColor3f(1.0f,0.0f,0.0f);
    //    draw image in the buffer
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
    
    //Deepak Comment here
    glPushMatrix();
    vector<cv::Point2f> centers;

    for (unsigned int m=0;m<TheMarkers.size();m++)
    {
       centers.push_back(TheMarkers[m].getCenter());
    }
    
    char textString[100] = "Free Mode";
    char unitString[100] = "Metric Units";
     if ( mode == Free){
      freeMode(centers);
      int a = sprintf(textString,"%s","Free Mode");
    } else if (mode == Triangle){
      triangleMode(centers);
      int a = sprintf(textString,"%s","Triangle Exploration Mode");
    } else if (mode == Grid){
      int a = sprintf(textString,"%s","Grid Mode");
      gridMode(centers);
    } else if (mode == Line){
       int a = sprintf(textString,"%s","Line Mode");
       lineMode(centers);
    } else if (mode == Line2){
       int a = sprintf(textString,"%s","Line Mode2");
       lineMode2(centers);
     }
     int m =sprintf(unitString,"%s",imperialUnitFlag?"(Imperial Units)":"(Metric Units)");
     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        
     //anti-aliasing
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
    
    if (centers.size() > 0){
      cout << "!!!" << TheMarkers[0].id <<endl;
    }
    if (centers.size() > 1){
      cv::Mat t0=TheMarkers[1].Tvec;
      float xtranslateArea = -4*0.015;
      float ytranslateArea = 2.5*0.015;
      float x_area = t0.at<float>(0,0) + xtranslateArea;
      float y_area = t0.at<float>(1,0) - 1.75*ytranslateArea;
      float z_area = -t0.at<float>(2,0);
      glPushMatrix();
      glLoadIdentity();
      glColor4f(1.0f,0.0f,0.0f,0.6f);
      glTranslatef(x_area,y_area,z_area);
      glRasterPos3f( 0.0f, 0.0f, 0.0f);
      char buffer[100];
      int n = sprintf(buffer,"%s \n%s",textString,unitString);
      drawString(buffer);
      glPopMatrix();
    }
    glutSwapBuffers();
}
/************************************
 *
 *
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
    glutPostRedisplay();
}
/************************************
 *
 *
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

