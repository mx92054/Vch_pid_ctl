#include "plant.h"
#include <string.h>

plant_block plant;
/*********************************************************
 *	@brief	目标初始化化
 *	@param	None
 *	@retval	None
 * ******************************************************/
void plant_init(plant_block *pt)
{
    memset(pt, 0, sizeof(plant_block));
    pt->ts = 0.1f; //采样时间为100ms
    pt->Jm = 4090.0f;
    pt->Lm = 3.08f;
}

/*********************************************************
 *	@brief	设定流速和夹角
 *	@param	spd  流速   wangle 夹角
 *	@retval	None
 * ******************************************************/
void plant_water_set(plant_block *pt, float wspd, short wangle)
{
    pt->water_vs = wspd;
    pt->water_angle = wangle % 360;
}
/*********************************************************
 *	@brief	目标计算
 *	@param	None
 *	@retval	None
 * ******************************************************/
void plant_step(plant_block *pt, float force)
{
    float val1;
    float val2;

    val1 = pt->angle + pt->ts * pt->dangle;
    val2 = 4265.0f * pt->dangle * pt->dangle - 13.2f * pt->dangle;
    val2 = (pt->dangle > 0.0f) ? (-val2) : val2;
    val2 += force * pt->Lm + map_angle2resistance(pt);
    val2 = pt->dangle + pt->ts * val2 / pt->Jm;

    pt->angle_pre = pt->angle;
    pt->dangle_pre = pt->dangle_pre;
    pt->angle = val1;
    pt->dangle = val2;
}

//-----------------------------------------------------------------
float flow_torque_table[25] = {
    0.0f, -13.3f, -99.0f, -144.3f, -187.7f,
    -120.0f, -81.3f, -32.5f, 33.8f, 19.1f,
    8.9f, -7.2f, 0.0f, -7.2f, 8.9f,
    19.1f, 33.8f, -32.5f, -81.3f, -120.0f,
    -187.7f, -144.3f, -99.0f, -13.3f, 0.0f};

/*********************************************************
 *	@brief	水阻力计算
 *	@param	None
 *	@retval	None
 * ******************************************************/
float map_angle2resistance(plant_block *pt)
{
    short vch_angle, cur_angle, index, offset;
    float result;
    vch_angle = (short)(pt->angle * 180.0f / 3.14f);
    vch_angle %= 360;
    cur_angle = pt->water_angle - vch_angle;
    cur_angle = (cur_angle > 0) ? cur_angle : (cur_angle + 360);

    index = cur_angle / 15;
    offset = cur_angle % 15;
    result = flow_torque_table[index];
    result += (float)offset * (flow_torque_table[index + 1] - flow_torque_table[index]) / 15.0f;
    result *= pt->water_vs * pt->water_vs / 0.25;

    return result;
}
