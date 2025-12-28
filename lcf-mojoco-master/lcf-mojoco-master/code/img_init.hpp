
#ifndef __IMG_INIT_H__
#define __IMG_INIT_H__

char image_cv_zip(void);
char image_cv_Init(void);
void show_two();
// 获取视频流

// // 转化为灰度
// extern Mat gray;
extern char img[60][80];
extern char img4[60][160];
extern char img5[120][160];   //灰度数组
extern uint8_t Threshold_real;




#endif