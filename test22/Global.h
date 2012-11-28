#include "stdafx.h"

CvxText text("simhei.ttf");
float p = 0.5;//透明度

IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0,*img_out = 0;

CvHistogram *hist = 0;
CvHistogram *hist_0 = 0;
CvHistogram *hist_1 = 0;
CvHistogram *hist_2 = 0;

int backproject_mode = 0;
int select_object = 0;
int track_object = 0;
int show_hist = 1;  
CvPoint origin;
CvRect selection;
CvRect track_window,track_window_temp;
CvBox2D track_box;  // tracking 返回的区域 box，带角度
CvConnectedComp track_comp;
int hdims = 80;     // 划分HIST的个数，越高越精确
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 40, vmax = 256, smin = 30;

//--------------------------------------
CvPoint mousePosition;//这个用于储存 camshift 得到的 track_box.center.x and y

CvPoint predict_pt;//这个就是 kalman 的预测坐标

const int winHeight=640;//这个就是采集到的视频大小，这个写固定320 * 240  640 * 480
const int winWidth=480;

BOOL	bOnceSave = TRUE;//保存数据只运行一次
int	minWidth = 0;//保存初始化时，跟踪的矩形框大小，之后跟踪的矩形框不能小于这个
int minHeight = 0;


//----------------------------------------
POINT NowCursorPos;//存放当前的鼠标坐标
POINT OldCursorPos;
POINT OldBox;		//跟踪矩形框
POINT NowBox;

int	iOldSize = 0;//保存第一次运行的矩阵面积
int	iNowSize = 0;

int		iframe = 0;//统计帧数，每3帧数进行一次跟踪坐标的计算，获取一次当前鼠标位置，然后计算



//----------------------------------------

//鼠标事件，用于手动寻找目标
void on_mouse( int event, int x, int y, int flags ,void* param)
{
	if( !image )
		return;

	if( image->origin )//这个是原点坐标的判断
		y = image->height - y;

	if( select_object )
	{
		selection.x = MIN(x,origin.x);//查找两个数组元素之间 的较小值 
		selection.y = MIN(y,origin.y);
		selection.width = selection.x + CV_IABS(x - origin.x); //a^b：a与b异或   或者是   绝对值
		selection.height = selection.y + CV_IABS(y - origin.y);

		//下面是控制选取在image大小范围之内
		selection.x = MAX( selection.x, 0 );
		selection.y = MAX( selection.y, 0 );
		selection.width = MIN( selection.width, image->width );
		selection.height = MIN( selection.height, image->height );
		selection.width -= selection.x;
		selection.height -= selection.y;

	}

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = cvPoint(x,y);
		selection = cvRect(x,y,0,0);
		select_object = 1;
		break;
	case CV_EVENT_LBUTTONUP:
		select_object = 0;
		if( selection.width > 0 && selection.height > 0 )
			track_object = -1;
#ifdef _DEBUG
		printf("\n # 鼠标的选择区域："); 
		printf("\n   X = %d, Y = %d, Width = %d, Height = %d",
			selection.x, selection.y, selection.width, selection.height);
#endif
		break;
	}
}

CvScalar hsv2rgb( float hue )
{
	int rgb[3], p, sector;
	static const int sector_data[][3]=
	{{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
	hue *= 0.033333333333333333333333333333333f;
	sector = cvFloor(hue);
	p = cvRound(255*(hue - sector));
	p ^= sector & 1 ? 255 : 0;

	rgb[sector_data[sector][0]] = 255;
	rgb[sector_data[sector][1]] = 0;
	rgb[sector_data[sector][2]] = p;

#ifdef _DEBUG
	printf("\n # Convert HSV to RGB："); 
	printf("\n   HUE = %f", hue);
	printf("\n   R = %d, G = %d, B = %d", rgb[0],rgb[1],rgb[2]);
#endif

	return cvScalar(rgb[2], rgb[1], rgb[0],0);
}
//读取Red初始化图片，以便进行tracking
BOOL loadTemplateImage_R()
{
	IplImage *tempimage = cvLoadImage("d:/Read.jpg",1);
	if (!tempimage)
	{
		return	FALSE;
	}
	cvCvtColor( tempimage, hsv, CV_BGR2HSV );
	int _vmin = vmin, _vmax = vmax;

	cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
		cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

	cvSplit( hsv, hue, 0, 0, 0 );

	selection.x = 1;
	selection.y = 1;
	selection.width = winHeight-1;//640:480
	selection.height= winWidth-1;

	cvSetImageROI( hue, selection );
	cvSetImageROI( mask, selection );
	cvCalcHist( &hue, hist, 0, mask );

	float max_val = 0.f;  

	cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
	cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
	cvResetImageROI( hue );
	cvResetImageROI( mask );
	track_window = selection;
	track_object = 1;

	cvReleaseImage(&tempimage);

	return TRUE;
}
//读取Green初始化图片，以便进行tracking
BOOL loadTemplateImage_G()
{
	IplImage *tempimage = cvLoadImage("d:/Green.jpg",1);
	if (!tempimage)
	{
		return	FALSE;
	}
	cvCvtColor( tempimage, hsv, CV_BGR2HSV );
	int _vmin = vmin, _vmax = vmax;

	cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
		cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

	cvSplit( hsv, hue, 0, 0, 0 );

	selection.x = 1;
	selection.y = 1;
	selection.width = winHeight-1;//640:480
	selection.height= winWidth-1;

	cvSetImageROI( hue, selection );
	cvSetImageROI( mask, selection );
	cvCalcHist( &hue, hist, 0, mask );

	float max_val = 0.f;  

	cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
	cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
	cvResetImageROI( hue );
	cvResetImageROI( mask );
	track_window = selection;
	track_object = 1;

	cvReleaseImage(&tempimage);

	return TRUE;
}
//读取Blue初始化图片，以便进行tracking
BOOL loadTemplateImage_B()
{
	IplImage *tempimage = cvLoadImage("d:/Blue.jpg",1);
	if (!tempimage)
	{
		return	FALSE;
	}
	cvCvtColor( tempimage, hsv, CV_BGR2HSV );
	int _vmin = vmin, _vmax = vmax;

	cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
		cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

	cvSplit( hsv, hue, 0, 0, 0 );

	selection.x = 1;
	selection.y = 1;
	selection.width = winHeight-1;//640:480
	selection.height= winWidth-1;

	cvSetImageROI( hue, selection );
	cvSetImageROI( mask, selection );
	cvCalcHist( &hue, hist, 0, mask );

	float max_val = 0.f;  

	cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
	cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
	cvResetImageROI( hue );
	cvResetImageROI( mask );
	track_window = selection;
	track_object = 1;

	cvReleaseImage(&tempimage);

	return TRUE;
}

//减法求绝对值的
int  iAbsolute(int a, int b)
{
	int c = 0;
	if (a > b)
	{
		c = a - b;
	}
	else
	{
		c = b - a;
	}
	return	c;
}