#include <math.h>

typedef double scalar;

typedef struct vector_t {
    enum { RECT, SPH } type;
    union {
        struct {
            scalar x, y, z;
        } rect;
        struct {
            scalar r, elevation, azimuth;
        } sph;
    };
} vector;

scalar vect_abs(const vector*);

void vect_mul(vector*, scalar);
void vect_add(vector*, const vector*);

void vect_to_rect(vector*);
void vect_to_sph(vector*);
void vect_sub(vector*, const vector*);
void vect_negate(vector*);

scalar vect_dot(const vector *v1, const vector *v2);
