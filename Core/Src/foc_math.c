#include "foc_math.h"

void FOC_Trig_Calc(float angle_rad, Trig_Vals_t *trig) 
{
    
    trig->Sin = arm_sin_f32(angle_rad);
    trig->Cos = arm_cos_f32(angle_rad);
}

void Clarke_Transform(const Vector_3Phase_t *in, Vector_AlBe_t *out) 
{

    out->Alpha = in->A;
    out->Beta  = VALUE_1_SQRT3 * (in->B - in->C);
}

void Inv_Clarke_Transform(const Vector_AlBe_t *in, Vector_3Phase_t *out) 
{
    out->A = in->Alpha;
    out->B = -0.5f * in->Alpha + VALUE_SQRT3_2 * in->Beta;
    out->C = -0.5f * in->Alpha - VALUE_SQRT3_2 * in->Beta;
}

void Park_Transform(const Vector_AlBe_t *in, const Trig_Vals_t *trig, Vector_DQ_t *out) 
{
    /*
     * 将 Alpha-Beta 坐标系投影到旋转的 d-q 轴上。
     * 注意：d轴对齐转子磁链，q轴超前d轴90度。
     */
    out->D =  (in->Alpha * trig->Cos) + (in->Beta * trig->Sin);
    out->Q = -(in->Alpha * trig->Sin) + (in->Beta * trig->Cos);
}

void Inv_Park_Transform(const Vector_DQ_t *in, const Trig_Vals_t *trig, Vector_AlBe_t *out) 
{
    /*
     * 将 PI 控制器输出的 Vd, Vq 指令转换回两相静止坐标系。
     */
    out->Alpha = (in->D * trig->Cos) - (in->Q * trig->Sin);
    out->Beta  = (in->D * trig->Sin) + (in->Q * trig->Cos);
}