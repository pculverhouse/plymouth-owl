
/* Phil Culverhouse Oct 2016 (c) Plymouth University
 *
 *    Permission is granted to copy, distribute and/or modify this document
 *    under the terms of the GNU Free Documentation License, Version 1.3
 *    or any later version published by the Free Software Foundation;
 *    with no Invariant Sections.
 *
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demonstration programs does the following:
 * a) loop 1 - take picture, check arrow keys
 *             move servos +5 pwm units for each loop
 *             draw 64x64 pixel square overlaid on Right image
 *             if 'c' is pressed copy patch into a template for matching with left
 *              exit loop 1;
 *		if ‘v’ pressed then arrange to capture 20 left/right pairs for camera calibration (code missing)
 * b) loop 2 - perform Normalised Cross Correlation between template and left image
 *             move Left eye to centre on best match with template
 *             (treats Right eye are dominate in this example).
 *             loop
 *             on exit by ESC key
 *                  go back to loop 1
 *
 * First start communications on Pi by running 'python PFCpacket.py' or the new ./OWLsocket (c++ variant)
 * Then run this program. The Pi IP server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 * NOTE: this program is just a demonstrator, the right eye does not track, just the left.
 * 
 * CHANGES
 * June 2017 PFC tidied up code
 *
 */

#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <unistd.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"


#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;



int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;

    Rx = RxLm; Lx = LxLm;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    string PiADDR = "10.0.0.10"; // DEFAULT OWL IP address on host
    string CALFolder ="../../Data/"; // place to save stereo-pairs calibration - should be the Data dir in Repo.

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

    /*******************************
    * LOOP continuously for testing
    */
    // RyC=RyC-40; LyC=LyC+40; // offset for cross on card
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images
    bool inLOOP=true; // run through cursor control first, capture a target then exit loop

//****** PROGRAM LOOP starts here ***********
// first loop just shows video, and waits for key press
// ‘c’ to capture a right target patch (from centre of field of view 64x64)
// ‘v’ to loop to capture camera calibration frames, not implemented in this demo version
// ‘other keys’ to move right camera to centre of desired target
//*******************************************
    string source ="http://10.0.0.10:8080/stream/video.mjpeg";  // the source file name as an MJPEG stream from OWL

    while (inLOOP){
        // move servos to centre of field
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck; // construct servo position packet
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str()); // get echo back to ensure all OK

        VideoCapture cap (source,CAP_FFMPEG);              // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        
	// Loop whilst video stream is working, else quit and try to get it again (above)
        while (inLOOP){
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                //         break;
            }
            //Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG image
            Right= Frame( Rect(0, 0, 640, 480));
            Left =Frame( Rect(640, 0, 640, 480)); // using a VGA rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy); //copy to avoid overwriting image that we use for correlation below
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
            imshow("Left",Left);imshow("Right", RightCopy);
            waitKey(30); // display the images
            int key = waitKey(0); // this is a pause long enough to allow a stable photo to be taken.
            
            switch (key){
            //left eye
            case 'w':
                Ly=Ly-5; // was Ly=+5 Changed BILL
                break;
            case 'z': //down left
                Ly=Ly+5; // was Ly=-5 BILL
                break;
            case 'a':
                Lx=Lx-5;
                break;
            case 's':
                Lx=Lx+5;
                break;
            // right eye
            case 'i':
                Ry=Ry+5;
                break;
            case 'm': //down right
                Ry=Ry-5;
                break;
            case 'j'://2424832: Changed BILL//left arrow
                Rx=Rx-5;
                break;
            case 'k'://2555904: Changed BILL// right arrow
                Rx=Rx+5;
                break;
                // other commands
            case 'v':  // video capture
                OwlCalCapture(cap, CALFolder);
                return(0);
            case 'c': // lowercase 'c' // set correlation target patch
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false; // quit loop and start tracking target
                break; 
                Ry=Ry+5;Ly=Ly-5; // was Ly=+5 Changed BILL
                break;
            default:
                key=' ';
                break;
                //nothing at present
            }

                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                if (0) {
                    for (int i=0;i<10;i++){
                        Rx=Rx-50; Lx=Lx-50;
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                        //waitKey(100); // cut the pause for a smooth pursuit camera motion
                    }
                }
            } // END cursor control loop
            // close windows down
            destroyAllWindows();

	    //** now run through correlation tracking verged cameras loop until user quits loop, 
	    //   then return to top loop **//
            // just a ZMCC -- see OpenCV for details
            // right is the template, just captured manually using the ‘c’ command key press above
            inLOOP=true; // run through the loop until decided to exit
            while (inLOOP) {
                if (!cap.read(Frame))
                {
                    cout  << "Could not open the input video: " << source << endl;
                    break;
                }
                //Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
                //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
                // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
                Right= Frame( Rect(0, 0, 640, 480)); // using a rectangle
                Left=Frame( Rect(640, 0, 640, 480)); // using a rectangle

                // NOW template match and move Left eye to Right eye target best fit template (not right eye does not track)
                OwlCorrel OWL;
                OWL = Owl_matchTemplate( Right,  Left, OWLtempl, target);
                /// Show me what you got
                Mat RightCopy;
                Right.copyTo(RightCopy);
                rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );
                rectangle( Left, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
                rectangle( OWLresult, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

                imshow("Owl-L", Left);
                imshow("Owl-R", RightCopy);
                imshow("Correl",OWL.Result );
                if (waitKey(10)== 27) inLOOP=false;
		//** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo
                double KPx=0.1; // track rate X
                double KPy=0.1; // track rate Y
                double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
                double Xoff= 320-(OWL.Match.x + OWLtempl.cols)/LxScaleV ; // compare to centre of image
                int LxOld=Lx;
                Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset]

                double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
                double Yoff= (250+(OWL.Match.y + OWLtempl.rows)/LyScaleV)*KPy ; // compare to centre of image
                int LyOld=Ly;
                Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset]

                cout << Lx << " " << Xoff << " " << LxOld << endl;
                cout << Ly << " " << Yoff << " " << LyOld << endl;

                //** ACTION
                // move to get minimise distance from centre of both images, ie verge in to target
                // move servos to position
                CMDstream.str(""); // clear string stream buffer, seems to not flush sometimes
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());
            } // end if ZMCC
        } // end while outer loop

#ifdef __WIN32__
        closesocket(u_sock);
#else
        close(clientSock);
#endif
        exit(0); // exit here for servo testing only
    }
