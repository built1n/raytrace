#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "vector.h"
#include <SDL/SDL.h>
#include <SDL/SDL_video.h>

#define WIDTH 320
#define HEIGHT 240

struct rgb_t { unsigned char r, g, b; };

struct object_t {
    enum { SPHERE, TRI } type;
    union {
        struct { vector center; scalar radius; } sphere;
        struct { vector points[3]; } tri;
    };
    struct rgb_t color;
    int specularity; /* 0-255 */
};

struct light_t {
    struct rgb_t color;
    vector position;
};

struct scene_t {
    struct rgb_t bg;
    struct object_t *objects;
    size_t n_objects;
    struct light_t *lights;
    size_t n_lights;
};

struct camera_t {
    vector origin;
    vector offset; /* direction to center of camera */
    scalar fov_x, fov_y; /* radians */
};

#define MAX(a, b) ((a>b)?(a):(b))
#define MIN(a, b) ((a<b)?(a):(b))
#define SQR(a) ((a)*(a))
#define MAX_BOUNCES 0

/* { o, d } form a ray */
bool object_intersects(struct object_t *obj, const vector* o, const vector* d, scalar *t)
{
    assert(o->type == RECT);
    assert(d->type == RECT);
    switch(obj->type)
    {
    case SPHERE:
    {
        scalar a = SQR(d->rect.x) + SQR(d->rect.y) + SQR(d->rect.z),
            b = 2 * (o->rect.x * d->rect.x + o->rect.y * d->rect.y + o->rect.z * d->rect.z),
            c = SQR(o->rect.x) + SQR(o->rect.y) + SQR(o->rect.z) - SQR(obj->sphere.radius);
        scalar disc = b*b - 4*a*c;
        if(disc < 0)
        {
            //printf("no intersection (%f)\n", disc);
            return false;
        }
        scalar t1 = (-b - sqrt(disc)) / (2*a), t2 = (-b + sqrt(disc)) / (2*a);
        /* both are negative */
        if(t1 < 0 && t2 < 0)
        {
            //printf("no intersection\n");
            return false;
        }
        /* one is negative */
        if(t1 * t2 < 0)
        {
            //printf("camera is inside sphere (%f, %f)!\n", t1, t2);
            *t = MAX(t1, t2);
            return true;
        }
        vector prod = *d;
        vect_mul(&prod, t2);
        vect_add(&prod, o);
        //printf("ray from (%f, %f, %f) intersects sphere at point %f, %f, %f (%f)\n", o->rect.x, o->rect.y, o->rect.z, prod.rect.x, prod.rect.y, prod.rect.z, vect_abs(&prod));
        *t = MIN(t1, t2);
        return true;
    }
    }
}

/* return the direction of the reflected ray */
vector reflect_ray(const vector *pt, const vector *d, const struct object_t *obj)
{
    vector normal;
    switch(obj->type)
    {
    case SPHERE:
    {
        vector p = *pt;
        vect_sub(&p, &obj->sphere.center);
        normal = p;
        break;
    }
    default:
        assert(false);
    }

    scalar c = -2 * vect_dot(d, &normal);
    vector reflected = normal;
    vect_mul(&reflected, c);
    vect_add(&reflected, d);
    return reflected;
}

struct rgb_t blend(struct rgb_t a, struct rgb_t b, int alpha)
{
    struct rgb_t ret;
    ret.r = ((a.r * alpha) + (b.r * (255 - alpha))) >> 8;
    ret.g = ((a.g * alpha) + (b.g * (255 - alpha))) >> 8;
    ret.b = ((a.b * alpha) + (b.b * (255 - alpha))) >> 8;
    return ret;
}

struct rgb_t trace_ray(const struct scene_t *scene, const vector *orig, const vector *d, int max_iters)
{
    struct rgb_t primary = scene->bg;
    scalar closest = -1; /* distance from camera in terms of d */
    const struct object_t *closest_obj = NULL;

    /* check intersections */
    for(int i = 0; i < scene->n_objects; ++i)
    {
        scalar t;
        if(object_intersects(scene->objects + i, orig, d, &t))
        {
            if(closest < 0 || t < closest)
            {
                closest = t;
                closest_obj = scene->objects + i;
            }
        }
    }

    if(closest_obj)
    {
        //printf("pow!\n");
        primary = closest_obj->color;
    }

    struct rgb_t reflected = {0, 0, 0};
    /* shade */
    if(closest_obj && closest_obj->specularity && max_iters > 0)
    {
        vector pt = *d;
        vect_mul(&pt, closest);
        vect_add(&pt, orig);
        vector ref = reflect_ray(&pt, d, closest_obj);
        reflected = trace_ray(scene, &pt, &ref, max_iters - 1);
    }

    return blend(primary, reflected, closest_obj?(255 - closest_obj->specularity):255);
}

unsigned char *render_scene(int w, int h, const struct scene_t *scene,
                            const struct camera_t *cam)
{
    unsigned char *fb = malloc(w * h * 3);
    scalar scale_x = cam->fov_x / w, scale_y = cam->fov_y / h;

    for(int y = 0; y < h; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            /* trace a ray from the camera into the scene */
            /* figure out how to rotate the offset vector to suit the
             * current pixel */

            /* rot in range of [-fov / 2, fov / 2) */
            scalar rot_x = (x - w / 2) * scale_x, rot_y = (y - h / 2) * scale_y;

            /* rotate the offset vector */
            vector d = cam->offset;
            vect_to_sph(&d);

            d.sph.elevation -= rot_y;
            d.sph.azimuth += rot_x;

            vect_to_rect(&d);

            //printf("(%d, %d) maps to (%f, %f, %f)\n", x, y, d.rect.x, d.rect.y, d.rect.z);

            /* cam->origin and d now form the camera ray */

            struct rgb_t color = trace_ray(scene, &cam->origin, &d, MAX_BOUNCES);

            fb[y * w * 3 + 3 * x] = color.r;
            fb[y * w * 3 + 3 * x + 1] = color.g;
            fb[y * w * 3 + 3 * x + 2] = color.b;
        }
    }
    return fb;
}

int main()
{
    vector test = (vector) { RECT, { 0, 1, 1 } };
    vect_to_sph(&test);
    //printf("1, 0, 0, is r = %f, elev = %f, azi = %f", test.sph.r, test.sph.elevation, test.sph.azimuth);
    //return 0;

    struct scene_t scene;
    scene.bg.r = 0xff;
    scene.bg.g = 0x00;
    scene.bg.b = 0xff;

    struct object_t sph;
    sph.type = SPHERE;
    sph.sphere.center = (vector) { RECT, {0, 0, 0 } };
    sph.sphere.radius = 1;
    sph.color = (struct rgb_t){0, 0, 0xff};
    sph.specularity = 100;

    scene.objects = &sph;
    scene.n_objects = 1;
    scene.n_lights = 0;

    struct camera_t cam;
    cam.origin = (vector){ RECT, {-5, 0, 0} };
    cam.offset = (vector){ RECT, {0, 0, 1} };
    cam.fov_x = M_PI / 2;
    cam.fov_y = M_PI / 2;

#if 0
    unsigned char *fb = render_scene(WIDTH, HEIGHT, &scene, &cam);
    FILE *f = fopen("test.ppm", "w");
    fprintf(f, "P6\n%d %d\n%d\n", WIDTH, HEIGHT, 255);
    fwrite(fb, WIDTH * HEIGHT, 3, f);
    fclose(f);
    return 0;

#else
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Surface *screen = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_SWSURFACE);
    SDL_EnableKeyRepeat(500, 100);

    while(1)
    {
        unsigned char *fb = render_scene(WIDTH, HEIGHT, &scene, &cam);
        memcpy(screen->pixels, fb, WIDTH * HEIGHT * 3);
        SDL_UpdateRect(screen, 0, 0, 0, 0);
        free(fb);
        SDL_Event e;
        printf("camera at %f, %f, %f\n", cam.origin.rect.x, cam.origin.rect.y, cam.origin.rect.z);
        while(SDL_PollEvent(&e))
        {
            switch(e.type)
            {
            case SDL_QUIT:
                return 0;
            case SDL_KEYDOWN:
                switch(e.key.keysym.sym)
                {
                case SDLK_LEFT:
                    cam.origin.rect.x -= .1;
                    break;
                case SDLK_RIGHT:
                    cam.origin.rect.x += .1;
                    break;
                case SDLK_UP:
                    cam.origin.rect.y += .1;
                    break;
                case SDLK_DOWN:
                    cam.origin.rect.y -= .1;
                    break;
                case SDLK_w:
                    cam.origin.rect.z += .1;
                    break;
                case SDLK_s:
                    cam.origin.rect.z -= .1;
                    break;
                }
                break;
            }
        }
    }
#endif
}
