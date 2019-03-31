#ifndef __PLANT_H__
#define __PLANT_H__

typedef struct _plant_block
{
    float angle;      //当前方位角 rad
    float angle_pre;  //上次方位角 rad
    float dangle;     //当前角速度 rad/s
    float dangle_pre; //上次角速度 rad/s

    float ddangle; //角加速度   rad/s2

    float ts; //采样时间
    float Jm; //转动惯量
    float Lm; //推进力力臂长度

    short water_angle; //来流角度  用度表示
    float water_vs; //来流速度 用m/s表示
} plant_block;

extern plant_block plant;

void plant_init(plant_block *pt);
void plant_step(plant_block *pt, float force);
float map_angle2resistance(plant_block *pt);
void plant_water_set(plant_block *pt, float wspd, short wangle);

#endif

/*-----------------------------------end of file--------*/
