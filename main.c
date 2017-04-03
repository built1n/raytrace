#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "vector.h"

#include <SDL/SDL.h>
#include <SDL/SDL_video.h>

#define WIDTH 320
#define HEIGHT 240
#define MAX(a, b) ((a>b)?(a):(b))
#define MIN(a, b) ((a<b)?(a):(b))
#define SQR(a) ((a)*(a))
#define MAX_BOUNCES 5

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
    scalar ambient;
};

struct camera_t {
    vector origin;
    vector offset; /* direction to center of camera */
    scalar fov_x, fov_y; /* radians */
};

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
            b = 2 * ((o->rect.x - obj->sphere.center.rect.x) * d->rect.x +
                     (o->rect.y - obj->sphere.center.rect.y) * d->rect.y +
                     (o->rect.z - obj->sphere.center.rect.z) * d->rect.z),
            c = SQR(o->rect.x - obj->sphere.center.rect.x) + SQR(o->rect.y - obj->sphere.center.rect.y) + SQR(o->rect.z - obj->sphere.center.rect.z) - SQR(obj->sphere.radius);
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

vector normal_at_point(const vector *pt, const struct object_t *obj)
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
    vect_normalize(&normal);
    return normal;
}

/* return the direction of the reflected ray */
vector reflect_ray(const vector *pt, const vector *d, const vector *normal, const struct object_t *obj)
{
    scalar c = -2 * vect_dot(d, normal);
    vector reflected = *normal;
    vect_mul(&reflected, c);
    vect_add(&reflected, d);
    return reflected;
}

struct rgb_t blend(struct rgb_t a, struct rgb_t b, int alpha)
{
    struct rgb_t ret;
    unsigned long r1, g1, b1;
    r1 = (((int)a.r * alpha) + ((int)b.r * (255 - alpha))) / 255;
    g1 = (((int)a.g * alpha) + ((int)b.g * (255 - alpha))) / 255;
    b1 = (((int)a.b * alpha) + ((int)b.b * (255 - alpha))) / 255;
    ret = (struct rgb_t){r1, g1, b1};
    return ret;
}

static const struct object_t *scene_intersections(const struct scene_t *scene,
                                                  const vector *orig, const vector *d, scalar *closest)
{
    *closest = -1;
    const struct object_t *obj = NULL;
    /* check intersections */
    for(int i = 0; i < scene->n_objects; ++i)
    {
        scalar t;
        if(object_intersects(scene->objects + i, orig, d, &t))
        {
            if(*closest < 0 || t < *closest)
            {
                *closest = t;
                obj = scene->objects + i;
            }
        }
    }
    return obj;
}

struct rgb_t trace_ray(const struct scene_t *scene, const vector *orig, const vector *d, int max_iters)
{
    struct rgb_t primary = scene->bg;
    scalar closest; /* distance from camera in terms of d */
    const struct object_t *closest_obj = scene_intersections(scene, orig, d, &closest);

    struct rgb_t reflected = {0, 0, 0};
    if(closest_obj)
    {
        //printf("pow!\n");
        primary = closest_obj->color;

        /* shade */
        scalar shade_total = 0;

        vector pt = *d;
        vect_mul(&pt, closest);
        vect_add(&pt, orig);

        vector normal = normal_at_point(&pt, closest_obj);

        for(int i = 0; i < scene->n_lights; ++i)
        {
            /* get vector to light */
            vector light_dir = scene->lights[i].position;
            vect_sub(&light_dir, &pt);

            scalar light_dist = vect_abs(&light_dir);

            vect_normalize(&light_dir);

            /* see if light is occluded */
            scalar nearest;
            if(scene_intersections(scene, &pt, &light_dir, &nearest))
                if(nearest < light_dist)
                    continue;

            scalar shade = vect_dot(&normal, &light_dir);
            if(shade > 0)
                shade_total += shade;
        }

        scalar diffuse = 1 - scene->ambient;
        primary.r *= (scene->ambient + diffuse * shade_total);
        primary.g *= (scene->ambient + diffuse * shade_total);
        primary.b *= (scene->ambient + diffuse * shade_total);

        /* reflections */
        if(closest_obj->specularity && max_iters > 0)
        {
            vector ref = reflect_ray(&pt, d, &normal, closest_obj);
            reflected = trace_ray(scene, &pt, &ref, max_iters - 1);
        }
    }

    return blend(primary, reflected, closest_obj ? (255 - closest_obj->specularity) : 255);
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
    scene.bg.r = 0xa0;
    scene.bg.g = 0xa0;
    scene.bg.b = 0xa0;
    scene.ambient = .2;

    struct object_t sph[3];
    sph[0].type = SPHERE;
    sph[0].sphere.center = (vector) { RECT, {0, 0, 1 } };
    sph[0].sphere.radius = 1;
    sph[0].color = (struct rgb_t){0, 0, 0xff};
    sph[0].specularity = 0;

    sph[1].type = SPHERE;
    sph[1].sphere.center = (vector) { RECT, {0, 0, -1 } };
    sph[1].sphere.radius = 1;
    sph[1].color = (struct rgb_t){0, 0xe0, 0};
    sph[1].specularity = 0x60;

    sph[2].type = SPHERE;
    sph[2].sphere.center = (vector) { RECT, {0, 0, -3 } };
    sph[2].sphere.radius = 1;
    sph[2].color = (struct rgb_t){0, 0xe0, 0};
    sph[2].specularity = 0x60;

    struct light_t lights[1];
    lights[0].position = (vector) { RECT, {0, 10, -10} };

    scene.objects = sph;
    scene.n_objects = 3;
    scene.lights = lights;
    scene.n_lights = 1;

    struct camera_t cam;
    cam.origin = (vector){ RECT, {-5, 0, 0} };
    cam.offset = (vector){ RECT, {0, 0, 1} };
    cam.fov_x = M_PI / 2;
    cam.fov_y = M_PI / 2 * HEIGHT / WIDTH;

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
    SDL_EnableKeyRepeat(500, 50);

    while(1)
    {
        unsigned char *fb = render_scene(WIDTH, HEIGHT, &scene, &cam);
        memcpy(screen->pixels, fb, WIDTH * HEIGHT * 3);
        SDL_UpdateRect(screen, 0, 0, 0, 0);
        free(fb);
        SDL_Event e;
        //printf("camera at %f, %f, %f\n", cam.origin.rect.x, cam.origin.rect.y, cam.origin.rect.z);
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
                    cam.origin.rect.z += .1;
                    break;
                case SDLK_RIGHT:
                    cam.origin.rect.z -= .1;
                    break;
                case SDLK_UP:
                    cam.origin.rect.y += .1;
                    break;
                case SDLK_DOWN:
                    cam.origin.rect.y -= .1;
                    break;
                case SDLK_w:
                    cam.origin.rect.x += .1;
                    break;
                case SDLK_s:
                    cam.origin.rect.x -= .1;
                    break;
                case SDLK_MINUS:
                    cam.fov_x += M_PI/36;
                    cam.fov_y = cam.fov_x * HEIGHT/WIDTH;
                    break;
                case SDLK_EQUALS:
                    cam.fov_x -= M_PI/36;
                    cam.fov_y = cam.fov_x * HEIGHT/WIDTH;
                    break;
                case SDLK_d:
                    vect_to_sph(&cam.offset);
                    cam.offset.sph.elevation -= M_PI/720;
                    printf("elevation is %f\n",                    cam.offset.sph.elevation);
                    vect_to_rect(&cam.offset);
                    break;
                case SDLK_a:
                    vect_to_sph(&cam.offset);
                    cam.offset.sph.elevation += M_PI/720;
                    vect_to_rect(&cam.offset);
                    break;
                }
                break;
            }
        }
    }
#endif
}
