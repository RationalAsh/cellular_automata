#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <SerialStream.h>
#include "string.h"
//#include <system>
//#include <math.h>
//#include <complex>

using namespace std;
using namespace cv;

///Global variables
Point clickpoint(1,1);
Point initialPos(1,1);
Point goalPoint(1,1);
int homeFlag = 0;
int destinationFlag = 0;
int TO_BUFFER = 1;

class imageCell
{
    public:
    ///constructor
    imageCell(Mat &image);
    ///Set image
    void setImage(Mat &image);
    ///convert from pixel to grid coordinates
    Point pixToGrid(int pix_row, int pix_col);
    ///Set the grid location
    void setGridLoc(int row, int col);
    void setGridLoc(Point gridLoc);

    ///Fill a cell with specified color
    void fillCell(int R, int G, int B);
    void fillTargetCell(int R, int G, int B, Point gridLoc);
    //void printCellColor();

    ///Check if cell is alive
    bool cellIsAlive(Point gridLoc);
    ///Check if neighbours are nice
    int neighbourCount(Point gridLoc);
    ///Step through simulation
    void simUpdate();
    ///Clear imgbuf
    void clearImgBuf();
    int getRows();
    int getCols();

    ///Get distance between two grid points
    int getDirectDistance(Point gridPoint1, Point gridPoint2);

    int **buf;

    private:
    int rows, cols,i,j;
    Mat img;
    int cell_length;
    Point gridPoint;
    Point cell_start;
    Point cell_end;

};

imageCell::imageCell(Mat &image)
{
    img = image;
    cell_length = 20;
    rows = (img.rows)/cell_length; cols = (img.cols)/cell_length;
    gridPoint.x = 1; gridPoint.y = 1;
    buf = new int*[rows+2];
    for(i=0;i<rows+2;i++) buf[i] = new int[cols+2];
    cout<<"\nThe size of the image is: "<<img.rows<<"x"<<img.cols<<"\n";
}

void imageCell::setImage(Mat &image)
{
    img = image;
}

Point imageCell::pixToGrid(int pix_row, int pix_col)
{
    Point pt((int)(pix_col/cell_length)+1,(int)(pix_row/cell_length)+1);
    return pt;
}

void imageCell::setGridLoc(int row, int col)
{
    gridPoint.x  = col;
    gridPoint.y  = row;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

void imageCell::setGridLoc(Point gridLoc)
{
    gridPoint = gridLoc;
    cell_start.y = (gridPoint.y - 1)*cell_length;
    cell_start.x = (gridPoint.x - 1)*cell_length;
    cell_end.y   = (gridPoint.y)*cell_length;
    cell_end.x   = (gridPoint.x)*cell_length;
}

void imageCell::fillCell(int R, int G, int B)
{
    i=0; j=0;
    for(i=cell_start.y+1; i<cell_end.y; i++)
    {
        for(j=cell_start.x+1; j<cell_end.x; j++)
        {
            img.at<Vec3b>(i,j)[0] = B;
            img.at<Vec3b>(i,j)[1] = G;
            img.at<Vec3b>(i,j)[2] = R;
        }
    }
}

void imageCell::fillTargetCell(int R, int G, int B, Point gridLoc)
{
    i=0; j=0;
    for(i=(gridLoc.y - 1)*cell_length+1; i<(gridLoc.y)*cell_length; i++)
    {
        for(j=(gridLoc.x - 1)*cell_length+1; j<(gridLoc.x)*cell_length; j++)
        {
            img.at<Vec3b>(i,j)[0] = B;
            img.at<Vec3b>(i,j)[1] = G;
            img.at<Vec3b>(i,j)[2] = R;
        }
    }

}


bool imageCell::cellIsAlive(Point gridLoc)
{

    if((img.at<Vec3b>(((gridLoc.y-1)*cell_length)+2,((gridLoc.x-1)*cell_length)+2)[0]==255)&&
        (img.at<Vec3b>(((gridLoc.y-1)*cell_length)+2,((gridLoc.x-1)*cell_length)+2)[1]==0)&&
        (img.at<Vec3b>(((gridLoc.y-1)*cell_length)+2,((gridLoc.x-1)*cell_length)+2)[2]==0)) return true;
        else return false;
}

int imageCell::neighbourCount(Point gridLoc)
{
    int lifeCount=0;
    Point loc = gridLoc + Point(-1,-1);

    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(0,1);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(0,1);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(1,0);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(1,0);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(0,-1);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(0,-1);
    if(cellIsAlive(loc)) lifeCount++; loc = loc+Point(-1,0);
    if(cellIsAlive(loc)) lifeCount++;

    return lifeCount;
}

void imageCell::clearImgBuf()
{
    i=0; j=0;
    for(i=0; i<rows+2; i++)
    {
        for(j=0; j<cols+2; j++)
        {
            buf[i][j]=0;
        }
    }
}

int imageCell::getRows()
{
    return rows;
}

int imageCell::getCols()
{
    return cols;
}

///For some odd reason this function doesn't seem to work.
///In fact, filling all the cells with a colour doesn't seem to work
///When I try to do it from inside the class. It works if I call the
///fillCell functions on each cell INSIDE the main loop though. I have
///no Idea why this is the case.
void imageCell::simUpdate()
{
    //int i, j;
    for(i=2;i<rows;i++)
    {
        for(j=2;j<cols;j++)
        {
            clearImgBuf();
            if(cellIsAlive(Point(i,j)))
            {
                if(neighbourCount(Point(i,j))<2) buf[i][j]=0;
                if(neighbourCount(Point(i,j))>3) buf[i][j]=0;
            }
            else
            {
                if(neighbourCount(Point(i,j))==3) buf[i][j]=1;
            }
        }
    }

    for(i=2;i<=rows;i++)
    {
        for(j=2;j<=cols;j++)
        {
            setGridLoc(i,j);
            if(buf[i][j]==1) fillCell(0,0,255);
            else fillCell(255,255,255);
        }
    }

//    for(i=1;i<rows;i++)
//    {
//        for(j=1;j<cols;j++)
//        {
//            fillTargetCell(0,0,255, Point(j,i));
//        }
//    }
}

int imageCell::getDirectDistance(Point gridPoint1, Point gridPoint2)
{
    Point delta = gridPoint2 - gridPoint1;
    return (int)sqrt(pow(delta.x, 2) + pow(delta.y, 2));
}

///Pathfinder class
class automata
{
    public:

    private:
    Point currentPos;
    std::vector<Point> frontier;
    std::vector<Point> explored;
    int estim_goal; int path_cost;
    int heuristic;

};


///The mousecallback function
void mouseEvent(int event, int x, int y, int flags, void *param)
{
    imageCell *cellptr = (imageCell*) param;

    if((event==EVENT_LBUTTONDOWN) || ((event==EVENT_MOUSEMOVE)&&(flags==EVENT_FLAG_LBUTTON)) )
    {
        clickpoint.x = x;
        clickpoint.y = y;
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(0,0,255);
        //cout<<"\nX: "<<x<<" Y: "<<y;
        //cout<<"\n"<<"neighbourcount: "<<cellptr->neighbourCount(cellptr->pixToGrid(y,x));
    }

    if((event==EVENT_LBUTTONDOWN) && (homeFlag==1))
    {
        cellptr->setGridLoc(initialPos.y, initialPos.x);
        cellptr->fillCell(255, 255, 255);
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(255, 0, 0);
        initialPos = cellptr->pixToGrid(y,x);
        homeFlag = 0;
    }

    if((event==EVENT_LBUTTONDOWN)&&(destinationFlag==1))
    {
        cellptr->setGridLoc(goalPoint.y, goalPoint.x);
        cellptr->fillCell(255, 255, 255);
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(0, 255, 0);
        goalPoint = cellptr->pixToGrid(y,x);
        destinationFlag = 0;
    }

    if((event==EVENT_RBUTTONDOWN) || ((event==EVENT_MOUSEMOVE)&&(flags==EVENT_FLAG_RBUTTON)))
    {
        clickpoint.x = x;
        clickpoint.y = y;
        cellptr->setGridLoc(cellptr->pixToGrid(y,x));
        cellptr->fillCell(255,255,255);
    }
}

///External clearbuf function because the one internal to the
///class doesn't seem to be working.
void clearBuf(int **buf, int  buf_rows, int buf_cols)
{
    int i,j;
    for(i=0;i<buf_rows;i++)
    {
        for(j=0;i<buf_cols;i++)
        {
            buf[i][j]=0;
        }
    }
}


int main(int argc, char** argv)
{
    ///Variables
    char keypress; int sim_start=0;
    Mat tempp(900, 1600, CV_8UC3, Scalar(0));
    ///loopstuff
    int i,j;
    ///Read image
    Mat maze = tempp;
    if(maze.empty()) return -1;


    ///Img processing class
    imageCell cells(maze);

    int **img_buf;
    img_buf = new int*[cells.getCols()+1];
    for(i=0;i<cells.getCols()+1;i++) img_buf[i] = new int[cells.getRows()+1];

    clearBuf(img_buf, cells.getCols()+1, cells.getCols()+1);

    cells.setGridLoc(1,1);
    namedWindow("maze");
    setMouseCallback("maze", mouseEvent,&cells);

    for(i=1; i<=cells.getRows(); i++)
    {
        for(j=1; j<=cells.getCols(); j++)
        {
            cells.setGridLoc(i,j);
            cells.fillCell(255, 255, 255);
        }
    }
    i=1;
    j=1;
    while(true)
    {
        imshow("maze", maze);

        if(cells.cellIsAlive(Point(1,1))) cout<<"\n First cell is alive!\n";
        if(sim_start==1 )
        {
//            for(i=1; i<cells.getRows(); i++)
//            {
//                for(i=1; i<cells.getCols(); j++)
//                {
//                    cout<<cells.cellIsAlive(Point(j,i));
//                }
//            }
            clearBuf(img_buf, cells.getCols()+1, cells.getRows()+1);
            for(i=1;i<cells.getRows();i++)
            {
                for(j=1;j<cells.getCols();j++)
                {

                    if(cells.cellIsAlive(Point(j,i)))
                    {
                        if(cells.neighbourCount(Point(j,i))<2) img_buf[j][i]=0;
                        if(cells.neighbourCount(Point(j,i))>3) img_buf[j][i]=0;
                        if(cells.neighbourCount(Point(j,i))==2)img_buf[j][i]=1;
                    }
                    else
                    {
                        if(cells.neighbourCount(Point(j,i))==3) img_buf[j][i]=1;
                        //cout<<"Alive!";
                    }
                }
            }

            for(i=1;i<cells.getRows();i++)
            {
                for(j=1;j<cells.getCols();j++)
                {
                    cells.setGridLoc(i,j);
                    if(img_buf[j][i]==1) cells.fillCell(0,0,255);
                    else cells.fillCell(255,255,255);
                }
            }

        }

        keypress = waitKey(300);
        if(keypress==27) break;
        if(keypress=='h') homeFlag = 1;
        if(keypress=='d') destinationFlag = 1;
        if(keypress=='s') sim_start ^= 1;

    }


    return 0;
}


