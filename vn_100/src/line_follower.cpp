#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cmath>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include<iostream>

using namespace cv;
using namespace std;

Mat src, src_gray, op,poly,r;
double d, dist, d_min;
Size s;
vector<int> f1, f2;
int flag;
vector<float> point_x,point_y;
double grad=0;
int ma = 0,p=0;
int an = 0;
Point2f p_end_1, p_end_2;
int shift=2;
Mat cartesianRotate(Mat, int);
Point2f pt;
int flg = 1;

int 	c_index_l, c_i2_l, c_index_r, c_i2_r;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

vector<vector<Point> > polygons;

//functions
double dst(Point2f, Point2f);

int main(int argc,char** argv)
{
    
    ros::init(argc,argv,"line_follow");
    ros::NodeHandle n;
ros::Publisher leftpub=n.advertise<std_msgs::UInt16>("sideleftspeed",1);
ros::Publisher rightpub=n.advertise<std_msgs::UInt16>("siderightspeed",1);
std_msgs::UInt16 msg1,msg2;
    VideoCapture cap(0);
    
    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video file" << endl;
        return -1;
    }
    namedWindow("input", CV_WINDOW_AUTOSIZE);
    namedWindow("thres", CV_WINDOW_AUTOSIZE);
    //src = imread("E:/AUV/3.jpg");
    //createTrackbar("angle", "Track", &an, 360, NULL);
    while (1)
    {

        //Mat img, gr;

        bool bSuccess1 = cap.read(src);
        if (!bSuccess1) //if not success, break loop
        {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        //cout << s.width << "  " << s.height;
        //transpose(src, src);
        src = cartesianRotate(src, -1);
        //flip(src,src,1);
        resize(src, src, Size(640, 480));
        //transpose(src, src);
        op = src(Rect(0, src.rows*0.5, (src.cols), src.rows*0.5));
    //	cout << s.height << "  " << s.width;
        s = op.size();
        //transpose(op, op);
        poly = Mat::zeros(op.size(), CV_8UC3);



        imshow("input", op);

        cvtColor(op, src_gray, CV_BGR2GRAY);
         threshold(src_gray, src_gray, 0, 255, CV_THRESH_OTSU);
        Mat elm = getStructuringElement(MORPH_RECT, Size(17, 17));

        erode(src_gray.clone(), src_gray, elm);
        erode(src_gray, src_gray, elm);
        dilate(src_gray, src_gray, elm);
        erode(src_gray, src_gray, elm);
        dilate(src_gray, src_gray, elm);

        findContours(src_gray.clone(), contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        int aHighIndex=0;
            double aHigh = 0;
            for( int i = 0; i< contours.size(); i++ )
            {
                if(contourArea(contours[i]) > aHigh)
                {
                    aHighIndex = i;
                    aHigh = contourArea(contours[i]);
                }
            }
        Mat line = Mat::zeros(src_gray.size(),CV_8UC3);
        drawContours(line,contours,aHighIndex,Scalar::all(255),CV_FILLED);
        /*for(int i=0;i<contours[aHighIndex].size();i++)
        {
            circle(line,contours[aHighIndex][i],2,Scalar(0,0,255));
        }*/
        Point2f lastpt = Point(0,0);
        Point2f cent = Point(0,0);
        int count = 0;
        double noiseDist = line.cols/80;
        vector<double> linecent;
        for(int i = 0;i<line.rows;i++)
        {
            for(int j=0;j<line.cols;j++)
            {
                if(line.at<Vec3b>(Point(j,i))[0])
                {
                    if(count==0)
                    {
                        cent = Point(j,i);
                        count++;
                        continue;
                    }
                //	/*else if(dst(cent,Point(j,i)) < noiseDist)
                //	{
                //		cent.x = (cent.x+j)/(cnt+1);
                //	}
                    else
                    {
                        cent.x = (cent.x*count+j)/(count+1);
                        count++;
                    }
                }
            }
            circle(line,cent,2,Scalar(0,0,255));
            if(count>1)
            linecent.push_back(cent.x);
            cent = Point(0,0);
            count = 0;
        }

        //for(int i = 0;i<)   */
        for (int i = 0; i < (linecent.size() - 2); i++)
        {
            grad += (linecent[i + 1] - linecent[i]);
            //cout << linecent[i] << endl;
        }
        //cout << "gradient  " << grad << endl;
        if(grad>10)
        {
msg1.data=80;
msg2.data=40;
      leftpub.publish(msg2);
rightpub.publish(msg1);
}
else if(grad<-5)
{
msg1.data=80;
msg2.data=40;
leftpub.publish(msg1);
rightpub.publish(msg2);
}
else
{
msg1.data=80;
msg2.data=80;
leftpub.publish(msg1);
rightpub.publish(msg2);
}
        grad = 0;

        imshow("thres", src_gray);
        imshow("line", line);




            //	waitKey(0);




            int c = waitKey(10);
            if (c == 27)
                break;
            else if (c == 112)
            {
                while (1)
                {

                    if (waitKey(10) == 32)
                        break;

                }
            }

        }
    }
double dst(Point2f p1, Point2f p2)
{
    double d = (p1.x - p2.x)*(p1.x - p2.x) +(p1.y - p2.y)*(p1.y - p2.y);
    double dist = pow(d, 0.5);
    return dist;
}

Mat cartesianRotate(Mat image, int method)
{
    if (method <0)
        transpose(image, image);
    //if(method == 1)
    //return image;
    Mat rotated = Mat(image.rows, image.cols, CV_MAKETYPE(CV_8U, image.channels()));
    //Vec3b value;
    //cout << image.size() << " " << rotated.size() <<  endl;
    flip(image, rotated, 1);
    /*for (int i = 0; i<rotated.cols; i++)
    for (int j = 0; j<rotated.rows; j++)
    rotated.at<Vec3b>(Point(i, j)) = image.at<Vec3b>(Point(image.cols - i - 1, j));*/

    if (method >0)
        transpose(rotated, rotated);
    return rotated;
}
