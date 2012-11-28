// test22.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "Global.h"
//-----------------------------------------------------------------



int _tmain(int argc, _TCHAR* argv[])
{
// 	CvCapture* capture = 0;
//  	IplImage* frame = 0;
// 	CCameraDS camera;
// //	打开第一个摄像头
// 	if(! camera.OpenCamera(0, true)) //弹出属性选择窗口
// 	if(! camera.OpenCamera(0, false, 320,240)) //不弹出属性选择窗口，用代码制定图像宽和高
// 	{
// 		fprintf(stderr, "Can not open camera.\n");
// 		return -1;
// 	}

	int width=640;//640:480
	int height=480;

	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SCRIPT_COMPLEX,1,1);
	
 	IplImage *frame=cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
 
 	videoInput video;//创建视频捕获对象
	video.setupDevice(0, width, height);//配置设备
	video.showSettingsWindow(0);//该语句可以显示视频设置窗口，可以去掉

	//----------------------------------------------------------------------
	//用到 kalman 滤波进行预测，这样能智能的扩大范围进行 camshift 处理
	//先初始化
	//1.kalman filter setup
	const int stateNum=4;
	const int measureNum=2;
	CvKalman* kalman = cvCreateKalman( stateNum, measureNum, 0 );//state(x,y,detaX,detaY)
	CvMat* process_noise = cvCreateMat( stateNum, 1, CV_32FC1 );
	CvMat* measurement = cvCreateMat( measureNum, 1, CV_32FC1 );//measurement(x,y)
	CvRNG rng = cvRNG(-1);
	float A[stateNum][stateNum] ={//transition matrix
		1,0,25,0,
		0,1,0,25,
		0,0,1,0,
		0,0,0,1
	};

	memcpy( kalman->transition_matrix->data.fl,A,sizeof(A));
	cvSetIdentity(kalman->measurement_matrix,cvRealScalar(1) );
	cvSetIdentity(kalman->process_noise_cov,cvRealScalar(1e-5));
	cvSetIdentity(kalman->measurement_noise_cov,cvRealScalar(1e-1));
	cvSetIdentity(kalman->error_cov_post,cvRealScalar(1));
	//initialize post state of kalman filter at random
	cvRandArr(&rng,kalman->state_post,CV_RAND_UNI,cvRealScalar(0),cvRealScalar(winHeight>winWidth?winWidth:winHeight));
	//----------------------------------------------------------------------

//	capture = cvCreateCameraCapture( 0 );

	//-----------------------------------------------------------------------
	printf( "Hot keys: \n"
		"\tESC - quit the program\n"
		"\tc - stop the tracking\n"
		"\tb - switch to/from backprojection view\n"
		"\th - show/hide object histogram\n"
		"\tg - tracking Green\n"
		"To initialize tracking, select the object with mouse\n" );
	//-----------------------------------------------------------------------
	cvNamedWindow( "CamShiftDemo", 1 );
	cvNamedWindow("out",1);
	cvSetMouseCallback( "CamShiftDemo", on_mouse, NULL ); // on_mouse 自定义事件
	cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
	cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
	cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );

	for(;;)
	{
		int i, bin_w, c;

//		frame = camera.QueryFrame();
		video.getPixels(0, (unsigned char *)frame->imageData, false, true);//获取一帧
//		frame = cvQueryFrame( capture );
		if (!frame){
			break;
		}

		if( !image )
		{
			/* allocate all the buffers */
			image = cvCreateImage( cvGetSize(frame), 8, 3 );

			image->origin = frame->origin;
			hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
			hue = cvCreateImage( cvGetSize(frame), 8, 1 );
			mask = cvCreateImage( cvGetSize(frame), 8, 1 );
			backproject = cvCreateImage( cvGetSize(frame), 8, 1 );

			hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );  // 计算直方图
			hist_0 = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
			hist_1 = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 ); 
			hist_2 = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

			histimg = cvCreateImage( cvSize(320,240), 8, 3 );
			cvZero( histimg );
		//	loadTemplateImage();
		}

		cvCopy( frame, image, 0 );
		cvCvtColor( image, hsv, CV_BGR2HSV );  // 彩色空间转换 BGR to HSV 

		if( track_object )
		{
			int _vmin = vmin, _vmax = vmax;

			cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
				cvScalar(180,256,MAX(_vmin,_vmax),0), mask );  // 得到二值的MASK
			cvSplit( hsv, hue, 0, 0, 0 );  // 只提取 HUE 分量

			if( track_object < 0 )
			{
				float max_val = 0.f;
				cvSetImageROI( hue, selection );  // 得到选择区域 for ROI
				cvSetImageROI( mask, selection ); // 得到选择区域 for mask
				cvCalcHist( &hue, hist, 0, mask ); // 计算直方图
				cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );  // 只找最大值
				cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 ); // 缩放 bin 到区间 [0,255] 
				/*
				bins : 用于存放直方图每个灰度级数目的数组指针，数组在cvCreateHist 的时候创建，
				其维数由cvCreateHist 确定（一般以一维比较常见）*/
				cvResetImageROI( hue );  // remove ROI
				cvResetImageROI( mask );
				track_window = selection;
				track_object = 1;

				cvZero( histimg );
				bin_w = histimg->width / hdims;  // hdims: 条的个数，则 bin_w 为条的宽度

				// 画直方图
				for( i = 0; i < hdims; i++ )
				{
					//cvRound 对一个double型的数进行四舍五入，并返回一个整型数！
					int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
					//cvGetReal1D返回单通道数组的指定元素 
					CvScalar color = hsv2rgb(i*180.f/hdims);
					cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
						cvPoint((i+1)*bin_w,histimg->height - val),
						color, -1, 8, 0 );
				}
			}

			cvCalcBackProject( &hue, backproject, hist );  // 使用 back project 方法
			cvAnd( backproject, mask, backproject, 0 );

			// calling CAMSHIFT 算法模块
			cvCamShift( backproject, track_window,
				cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
				&track_comp, &track_box );
			track_window = track_comp.rect;//这里赋值给下一次查找的区域，可以稍微扩大一点搜索区域
		//	track_window_temp = track_comp.rect;//

			//保存最小要求的矩形框大小
			if (bOnceSave)
			{
				minWidth = track_box.size.width;
				minHeight = track_box.size.height;
				iOldSize = minHeight * minWidth;
				bOnceSave = FALSE;
			}


			///////////////////////////////////////////////////////////////////////////////////
			mousePosition.x = track_box.center.x;
			mousePosition.y = track_box.center.y;
			//进行 kalman 预测，可以得到 predict_pt 预测坐标
			//2.kalman prediction
			const CvMat* prediction=cvKalmanPredict(kalman,0);
			CvPoint predict_pt=cvPoint((int)prediction->data.fl[0],(int)prediction->data.fl[1]);
			//3.update measurement
			measurement->data.fl[0]=(float)mousePosition.x;
			measurement->data.fl[1]=(float)mousePosition.y;
			//4.update
			cvKalmanCorrect( kalman, measurement );	
			///////////////////////////////////////////////////////////////////////////////////
			//因为视频设置的采集大小是640 * 480，那么这个搜索区域夸大后也得在这范围内
			//-------------------------------------------------------------------------------------
			//下面这里其实就是一个粗略的预测 track_window 的范围
			//因为在鼠标选取的时候，有时可能只是点击了窗体，所以没得 width  和 height 都为0
			int iBetween = 0;
			//确保预测点 与 实际点之间 连线距离 在 本次 track_box 的size 之内
			iBetween = sqrt(	powf((kalman->PosterState[0] - track_box.center.x),2) 
				+  
				powf((kalman->PosterState[1] - track_box.center.y),2) );

			CvPoint prePoint;//预测的点 相对于 实际点 的对称点

			if ( iBetween > 5)
			{
				//当实际点 在 预测点 右边
				if (track_box.center.x > kalman->PosterState[0])
				{
					//且，实际点在 预测点 下面
					if (track_box.center.y > kalman->PosterState[1])
					{
						prePoint.x = track_box.center.x + iAbsolute(track_box.center.x,kalman->PosterState[0]);
						prePoint.y = track_box.center.y + iAbsolute(track_box.center.y,kalman->PosterState[1]);
					}
					//且，实际点在 预测点 上面
					else
					{
						prePoint.x = track_box.center.x + iAbsolute(track_box.center.x,kalman->PosterState[0]);
						prePoint.y = track_box.center.y - iAbsolute(track_box.center.y,kalman->PosterState[1]);
					}
					//宽高
					if (track_window.width != 0)
					{
						track_window.width += iBetween + iAbsolute(track_box.center.x,kalman->PosterState[0]);
					}

					if (track_window.height != 0)
					{
						track_window.height += iBetween + iAbsolute(track_box.center.x,kalman->PosterState[0]);
					}
				}
				//当实际点 在 预测点 左边
				else
				{
					//且，实际点在 预测点 下面
					if (track_box.center.y > kalman->PosterState[1])
					{
						prePoint.x = track_box.center.x - iAbsolute(track_box.center.x,kalman->PosterState[0]);
						prePoint.y = track_box.center.y + iAbsolute(track_box.center.y,kalman->PosterState[1]);
					}
					//且，实际点在 预测点 上面
					else
					{
						prePoint.x = track_box.center.x - iAbsolute(track_box.center.x,kalman->PosterState[0]);
						prePoint.y = track_box.center.y - iAbsolute(track_box.center.y,kalman->PosterState[1]);
					}
					//宽高
					if (track_window.width != 0)
					{
						track_window.width += iBetween + iAbsolute(track_box.center.x,kalman->PosterState[0]);
					}

					if (track_window.height != 0)
					{
						track_window.height += iBetween +iAbsolute(track_box.center.x,kalman->PosterState[0]);
					}
				}

				track_window.x = prePoint.x - iBetween;	
				track_window.y = prePoint.y - iBetween;
			}
			else
			{
				track_window.x -= iBetween;
				track_window.y -= iBetween;
				//宽高
				if (track_window.width != 0)
				{
					track_window.width += iBetween;
				}

				if (track_window.height != 0)
				{
					track_window.height += iBetween;
				}
			}

			//跟踪的矩形框不能小于初始化检测到的大小，当这个情况的时候，X 和 Y可以适当的在缩小
			if (track_window.width < minWidth)
			{
				track_window.width = minWidth;
				track_window.x -= iBetween;
			}
			if (track_window.height < minHeight)
			{
				track_window.height = minHeight;
				track_window.y -= iBetween;
			}

			//确保调整后的矩形大小在640 * 480之内
			if (track_window.x <= 0)
			{
				track_window.x = 0;
			}
			if (track_window.y <= 0)
			{
				track_window.y = 0;
			}
			if (track_window.x >= 600)
			{
				track_window.x = 600;
			}
			if (track_window.y >= 440)
			{
				track_window.y = 440;
			}

			if (track_window.width + track_window.x >= 640)
			{
				track_window.width = 640 - track_window.x;
			}
			if (track_window.height + track_window.y >= 640)
			{
				track_window.height = 640 - track_window.y;
			}
			//-------------------------------------------------------------------------------------
			img_out=cvCreateImage(cvSize(winWidth,winHeight),8,3);
			cvSet(img_out,cvScalar(255,255,255,0));
			char buf[256];
			sprintf(buf,"%d",iBetween);
			cvPutText(img_out,buf,cvPoint(10,30),&font,CV_RGB(0,0,0));
			sprintf(buf,"%d : %d",track_window.x,track_window.y);
			cvPutText(img_out,buf,cvPoint(10,50),&font,CV_RGB(0,0,0));
			sprintf(buf,"%d : %d",track_window.width,track_window.height);
			cvPutText(img_out,buf,cvPoint(10,70),&font,CV_RGB(0,0,0));

			sprintf(buf,"size: %0.2f",track_box.size.width * track_box.size.height);
			cvPutText(img_out,buf,cvPoint(10,90),&font,CV_RGB(0,0,0));
			//-------------------------------------------------------------------------------------
			if( backproject_mode )
				cvCvtColor( backproject, image, CV_GRAY2BGR ); // 使用backproject灰度图像
			if( image->origin )
				track_box.angle = -track_box.angle;

			//--------------------------
			cvRectangle(image,cvPoint(track_window.x,track_window.y),
						cvPoint(track_window.x + track_window.width,track_window.y + track_window.height),
						CV_RGB(0,255,255)
						);
			//--------------------------
			cvCircle(image,predict_pt,5,CV_RGB(0,0,255),3);//predicted point with green
			cvEllipseBox( image, track_box, CV_RGB(0,255,0), 3, CV_AA, 0 );

			//--------------------------------------------------------------------------
			if (iframe == 0)
			{
				GetCursorPos(&OldCursorPos);//获取当前鼠标坐标
				OldBox.x = track_box.center.x;
				OldBox.y = track_box.center.y;
			}
			if (iframe < 3)//每3帧进行一次判断
			{
				iframe++;
			}
			else
			{
				iframe = 0;
				iNowSize = track_box.size.width * track_box.size.height;

				if ((iNowSize / iOldSize) > (3/5))
				{
					NowBox.x = track_box.center.x;
					NowBox.y = track_box.center.y;

					SetCursorPos(OldCursorPos.x - (NowBox.x - OldBox.x)*1366/640,
						OldCursorPos.y - (NowBox.y - OldBox.y)*768/480);
				}
			}
			//--------------------------------------------------------------------------
		}

		if( select_object && selection.width > 0 && selection.height > 0 )
		{
			cvSetImageROI( image, selection );
			cvXorS( image, cvScalarAll(255), image, 0 );
			cvResetImageROI( image );
		}

		//----------------------------------------------------------------------
		int x=(cvGetSize(frame).width-cvGetSize(frame).height)/2;
		int y=cvGetSize(frame).height;
		cvLine(image,cvPoint(x,0),cvPoint(x,y),CV_RGB(255,255,0));
		cvLine(image,cvPoint(cvGetSize(frame).width-x,0),cvPoint(cvGetSize(frame).width-x,y),CV_RGB(255,255,0));
		x=cvGetSize(frame).width/2;
		y=cvGetSize(frame).height/2;
		cvLine(image,cvPoint(x+10,y),cvPoint(x-10,y),CV_RGB(255,0,0));
		cvLine(image,cvPoint(x,y+10),cvPoint(x,y-10),CV_RGB(255,0,0));
		//----------------------------------------------------------------------
		cvShowImage( "out", img_out );
		cvShowImage( "CamShiftDemo", image );
		//----------------------------------------------------------------------
		if (!show_hist)
		{
			cvShowImage( "Histogram", histimg );
		}
		else
		{
			cvDestroyWindow("Histogram");
		}
		//----------------------------------------------------------------------
		cvReleaseImage(&img_out);
		//----------------------------------------------------------------------
		c = cvWaitKey(10);
		if( c == 27 )
			break;  // exit from for-loop
		switch( c )
		{
		case 'b':
			backproject_mode ^= 1;
			break;
		case 'c':
			track_object = 0;
			bOnceSave = TRUE;
			cvZero( histimg );
			break;
		case 'h':
			show_hist ^= 1;
			if( !show_hist )
				cvDestroyWindow( "Histogram" );
			else
				cvNamedWindow( "Histogram", 1 );
			break;
		case 'g':
			loadTemplateImage_G();
			break;
		default:
			;
		}
		//---------------------------------------------	
	}
	video.stopDevice(0);
//	cvReleaseCapture( &capture );
	cvDestroyWindow("CamShiftDemo");
	cvDestroyWindow("out");

	return 0;
}

