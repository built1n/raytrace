#include "vector.h"

void vect_to_rect(vector *v)
{
    if(v->type == RECT)
        return;
    switch(v->type)
    {
    case SPH:
    {
        vector old = *v;
        v->type = RECT;
        v->rect.x = old.sph.r * cos(old.sph.elevation) * sin(old.sph.azimuth);
        v->rect.y = old.sph.r * sin(old.sph.elevation);
        v->rect.z = old.sph.r * cos(old.sph.elevation) * cos(old.sph.azimuth);
        break;
    }
    }
}

void vect_to_sph(vector *v)
{
    if(v->type == SPH)
        return;
    switch(v->type)
    {
    case RECT:
    {
        vector old = *v;
        v->type = SPH;
        v->sph.r = vect_abs(&old);
        v->sph.elevation = atan2(old.rect.y, sqrt(old.rect.x*old.rect.x + old.rect.z*old.rect.z));
        v->sph.azimuth = atan2(old.rect.z, old.rect.x);
        break;
    }
    }
}

scalar vect_abs(const vector *v)
{
    switch(v->type)
    {
    case SPH:
        return v->sph.r;
    case RECT:
        return sqrt(v->rect.x * v->rect.x + v->rect.y * v->rect.y + v->rect.z * v->rect.z);
    }
}

void vect_mul(vector *v, scalar s)
{
    switch(v->type)
    {
    case SPH:
        v->sph.r *= s;
        break;
    case RECT:
        v->rect.x *= s;
        v->rect.y *= s;
        v->rect.z *= s;
        break;
    }
}

void vect_add(vector *v1, const vector *v2)
{
    int old_type = v1->type;
    vector tmp1 = *v1;
    vector tmp2 = *v2;
    vect_to_rect(&tmp1);
    vect_to_rect(&tmp2);
    tmp1.rect.x += tmp2.rect.x;
    tmp1.rect.y += tmp2.rect.y;
    tmp1.rect.z += tmp2.rect.z;

    *v1 = tmp1;
    if(old_type == SPH)
        vect_to_sph(v1);
}

void vect_negate(vector *v)
{
    switch(v->type)
    {
    case SPH:
        v->sph.r = -v->sph.r;
        break;
    case RECT:
        v->rect.x = -v->rect.x;
        v->rect.y = -v->rect.y;
        v->rect.z = -v->rect.z;
        break;
    }
}

/* v1 = v1 - v2 */
void vect_sub(vector *v1, const vector *v2)
{
    vector neg = *v2;
    vect_negate(&neg);
    vect_add(v1, &neg);
}

scalar vect_dot(const vector *v1, const vector *v2)
{
    vector a = *v1, b = *v2;
    vect_to_rect(&a);
    vect_to_rect(&b);
    return a.rect.x * b.rect.x + a.rect.y * b.rect.y + a.rect.z * b.rect.z;
}

void vect_normalize(vector *v)
{
    scalar abs = vect_abs(v);
    vect_mul(v, 1./abs);
}
