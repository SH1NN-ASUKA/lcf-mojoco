#include "zf_device_uvc.h"
#include "zf_common_headfile.h"


using namespace cv;
using namespace std;

// 创建一个摄像头
VideoCapture cap(0);
// 获取视频流
Mat frame;


char img[60][80];
char img4[60][160];
char img5[120][160];   //灰度数组
uint8_t Threshold_real;

//摄像头初始化，成功返回1，失败返回-1；
char image_cv_Init(void)
{
    //打开摄像头
    if (!cap.isOpened())
    {
        cerr << "Error open video stream" << endl;
        return -1;
    }
    else
    {
        printf("open video successful");
    }
    // 设置视频流编码器
    cap.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // 设置摄像头图像宽高和帧率
    cap.set(CAP_PROP_FRAME_WIDTH, 160);
    cap.set(CAP_PROP_FRAME_HEIGHT,120);
    cap.set(CAP_PROP_FPS, 120);

    // 设置摄像头的曝光
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);   //非完全关闭自动曝光
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);   //完全关闭自动曝光
    // cap.set(cv::CAP_PROP_EXPOSURE, 300);      //设置曝光

    // 获取摄像头图像宽高和帧率
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double frame_fps = cap.get(cv::CAP_PROP_FPS);
    printf("frame:%d*%d, fps:%3f", frame_width, frame_height, frame_fps);
    sleep(1);
    return 1;
}

uint8_t Threshold_deal(uint8_t* image,
    uint16_t col,
    uint16_t row,
    uint16_t pixel_threshold)
{
    #define GrayScale 256
    uint16_t width = CAMERA_W;
    uint16_t height = CAMERA_H;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height;
    uint8_t threshold = 0;
    uint8_t* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++) {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    uint32_t gray_sum = 0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i += 1) {
        for (j = 0; j < width; j += 1) {
            // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
            //{
            pixelCount[(
                int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum += (int)data[i * width + j];  //灰度值总和
            //}
        }
    }

    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++) {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }


    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < pixel_threshold; j++) {
        w0 +=
            pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

        w1 = 1 - w0;
        u1tmp = gray_sum / pixelSum - u0tmp;

        u0 = u0tmp / w0;    //背景平均灰度
        u1 = u1tmp / w1;    //前景平均灰度
        u = u0tmp + u1tmp;  //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax) {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax) break;
    }
    return threshold;
}

char image_cv_zip(void)
{
    
    // 读取摄像头一帧图像
    cap.read(frame);    // cap >> frame;
    // cap >> frame;
    if (frame.empty())
    {
        cerr << "Error read frame" << endl;
        return -1;
    }
    // 转化为灰度
    Mat gray;
    //frame为彩色输入图像，gray为灰度输出图像
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    for (int i=0; i<120;i+=1)
    {
        for (int j=0; j<160; j+=1)
        {
            img5[i][j] = gray.at<unsigned char>(i, j);;
        }
    }
    // ips200_show_gray_image(0, 0, (uint8_t*)img5, 160, 120); // 假设 gray 是 CV_8U 类型的单通道 Mat
        // ips200_show_gray_image(0, 0, gray.ptr<uint8_t>(), 160, 120);

    // 二值化处理
    Mat binary;
    //gray为输入的灰度图像，binary为二值化后输出的图像
    // threshold(gray, binary, 0, 1,  THRESH_BINARY+THRESH_OTSU);
    //第一个binary为要压缩的输入图像，第二个binary为压缩后的输出图像，第三个是压缩为80*60的大小
    // resize(binary, binary, Size(160, 120));

    // resize(gray, gray, Size(160, 120));


    // auto start2 = std::chrono::high_resolution_clock::now();
    // cv::threshold(gray, binary1, 0, 1, THRESH_BINARY + THRESH_OTSU);
    // cv::Mat resized2;
    // resize(binary1, binary1, Size(80, 60));
    // auto end2 = std::chrono::high_resolution_clock::now();
    // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();
    // std::cout << "先二值化再压缩耗时: " << duration2 << " 毫秒" << std::endl;


    for(int i=45,p=0;i<105;i++,p++)
    {
        for(int j=0,k=0;j<160;j+=2,k++) 
        {
            img_temp[p][k]=gray.at<unsigned char>(i, j);
            // img1[p][k]=binary.at<unsigned char>(i, j);
            // img3[p][k]=binary.at<unsigned char>(i, j);
            // printf("%d",binary.at<unsigned char>(i, j));
        }
    }


Threshold_real = Threshold_deal(img_temp[0], CAMERA_W, CAMERA_H, 250)+15;

    // threshold(gray, binary, Threshold, 255, THRESH_BINARY);


    // 创建 img5 用于存储倒转后的图像


    for (int i = 0; i < 60; i+=1)
    {
        for (int j = 0; j < 80; j+=1)
        {  
            // img3[i][j]=binary.at<unsigned char>(i, j);
            if(img_temp[i][j]>Threshold_real){img3[i][j]=255;img1[i][j]=1;}
            else {img3[i][j]=0;img1[i][j]=0;}
            // img3[i][j] = img_temp[i][j];
        }
    }
    // // 创建 img 用于存储裁剪并压缩后的图像
    // for (int i = 60,k=0; i < 120; i+=1,k++)
    // {
    //     for (int j = 0,p=0; j < 160; j+=2,p++)
    //     {
    //         img1[k][p]=img5[i][j];
    //         img3[k][p]=img5[i][j];
    //         //printf("%d",img3[i][j]);
    //     }
    //     //printf("\n");
    // }
    // //printf("\n\n");

    return 1;
}

void show_two()
{
    for(int i=0;i<60;i++)
    {
        for(int j=0;j<80;j++)
        {
            if(img3[i][j] == 255)  ips200_draw_point(j,i,RGB565_WHITE);
            else ips200_draw_point(j,i,RGB565_BLACK);
        }
        
    }

}