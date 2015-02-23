// All Functions in aruco_test_gl.cpp 
// 2013.6.20
// Chan Kim

// Main Process Func.
int main(int argc,char **argv);
void init();
bool readArguments ( int argc,char **argv );
void vDrawScene();
void vIdle();
void vResize( GLsizei iWidth, GLsizei iHeight );

// Menu Manipulation Func.
void vMenu(int value);
//void vMouse(int b,int s,int x,int y);
void vKeyboard(unsigned char key,int,int);

// Mode Func.
void freeMode(vector<cv::Point2f> centers,bool);
void gridMode(vector<cv::Point2f> centers);
void lineMode(vector<cv::Point2f> centers);
void lineMode2(vector<cv::Point2f> centers);
void ScreenCapture();
// Marker Func.
void detectMarker(vector<cv::Point2f> centers);
void assignMarker(vector<cv::Point2f> centers);
void assignMarkerLine(vector<cv::Point2f> centers);

// Calculate Func.
float convertD(float distance); // convert UNIT from meter to inch or centimeter
void convertDistance(float *distance);
string convertInt(int);
float calculateDistance(cv::Mat t0,cv::Mat t1,bool convert);
float calculateTriangleArea(cv::Mat t0, cv::Mat t1, cv::Mat t2);
int calculatePerimeter(vector<cv::Point2f> centers);
float calculateTriangleArea(cv::Mat t0, cv::Mat t1, cv::Mat t2, bool flag);
float calculateArea(vector<cv::Point2f> centers);   
float getCos(cv::Mat t0, cv::Mat t1);
float getSin(cv::Mat t0, cv::Mat t1);
float cal_bottom_len(float hypotenuse, float height);
float calcSlope(cv::Mat t0, cv::Mat t1);

// Draw Func.
void drawArea(vector<cv::Point2f> centers);
void drawTriArea(vector<cv::Point2f> centers);
void drawString(char* string, int sub);
void drawStringLetter(std::string sstring);
void drawX(cv::Mat t3,cv::Mat t2, char xchar);
void drawLetter(cv::Mat currentMat, std::string letter, int black,float dist);
void drawSideText(cv::Mat t3,cv::Mat t2);
void drawSideTextTranslate(cv::Mat t3,cv::Mat t2, float unit);
void drawLetterOnXTranslate(cv::Mat tnaught, std::string letter);


