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
#define SIGN(x) ((x)<0?-1:1)

#define MAX_BOUNCES 2
#define MOVE_FACTOR .1


#define MOUSELOOK

struct rgb_t { unsigned char r, g, b; };

struct object_t {
    enum { SPHERE, PLANE } type;
    union {
        struct { vector center; scalar radius; } sphere;
        struct { vector point, normal; } plane;
    };
    struct rgb_t color;
    int specularity; /* 0-255 */
};

struct light_t {
    vector position;
    scalar intensity;
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
    vector direction; /* direction to center of camera */
    scalar fov_x, fov_y; /* radians */
};

/* { o, d } form a ray */
/* point of intersection is *t * d units away */
inline bool object_intersects(const struct object_t *obj, vector o, vector d, scalar *t)
{
    assert(o.type == RECT);
    vect_to_rect(&d);
    assert(d.type == RECT);
    switch(obj->type)
    {
    case SPHERE:
    {
        scalar a = SQR(d.rect.x) + SQR(d.rect.y) + SQR(d.rect.z),
            b = 2 * ((o.rect.x - obj->sphere.center.rect.x) * d.rect.x +
                     (o.rect.y - obj->sphere.center.rect.y) * d.rect.y +
                     (o.rect.z - obj->sphere.center.rect.z) * d.rect.z),
            c = SQR(o.rect.x - obj->sphere.center.rect.x) +
                SQR(o.rect.y - obj->sphere.center.rect.y) +
                SQR(o.rect.z - obj->sphere.center.rect.z) - SQR(obj->sphere.radius);
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
        vector prod = d;
        prod = vect_mul(prod, t2);
        prod = vect_add(prod, o);
        //printf("ray from (%f, %f, %f) intersects sphere at point %f, %f, %f (%f)\n", o->rect.x, o->rect.y, o->rect.z, prod.rect.x, prod.rect.y, prod.rect.z, vect_abs(&prod));
        *t = MIN(t1, t2);
        return true;
    }
    case PLANE:
    {
        scalar denom = vect_dot(obj->plane.normal, d);
        if(!denom)
            return false;
        scalar t1 = vect_dot(obj->plane.normal, vect_sub(obj->plane.point, o)) / denom;
        if(t1 <= 0)
            return false;
        *t = t1;
        return true;
    }
    }
}

vector normal_at_point(vector pt, const struct object_t *obj)
{
    vector normal;
    switch(obj->type)
    {
    case SPHERE:
        normal = vect_sub(pt, obj->sphere.center);
        break;
    case PLANE:
        normal = obj->plane.normal;
        break;
    default:
        assert(false);
    }
    return normal;
}

/* return the direction of the reflected ray */
vector reflect_ray(vector pt, vector d, vector normal, const struct object_t *obj)
{
    scalar c = -2 * vect_dot(d, normal);
    vector reflected = normal;
    reflected = vect_mul(reflected, c);
    reflected = vect_add(reflected, d);
    return reflected;
}

struct rgb_t blend(struct rgb_t a, struct rgb_t b, unsigned char alpha)
{
    struct rgb_t ret;
    unsigned char r1, g1, b1;
    ret.r = (((int)a.r * alpha) + ((int)b.r * (255 - alpha))) / 255;
    ret.g = (((int)a.g * alpha) + ((int)b.g * (255 - alpha))) / 255;
    ret.b = (((int)a.b * alpha) + ((int)b.b * (255 - alpha))) / 255;
    return ret;
}

static const struct object_t *scene_intersections(const struct scene_t *scene,
                                                  vector orig, vector d, scalar *dist, const struct object_t *avoid)
{
    *dist = -1;
    const struct object_t *obj = NULL;
    /* check intersections */
    for(int i = 0; i < scene->n_objects; ++i)
    {
        /* avoid intersections with the same object */
        if(avoid == scene->objects + i)
            continue;
        scalar t;
        if(object_intersects(scene->objects + i, orig, d, &t))
        {
            if(*dist < 0 || t < *dist)
            {
                *dist = t;
                obj = scene->objects + i;
            }
        }
    }
    return obj;
}

#define ABS(x) ((x)<0?-(x):(x))

struct rgb_t trace_ray(const struct scene_t *scene, vector orig, vector d, int max_iters, const struct object_t *avoid)
{
    vector copy = d;
    vect_to_sph(&copy);

    struct rgb_t primary = blend((struct rgb_t) {0xff, 0xff, 0xff}, (struct rgb_t) { 0x00, 0x00, 0xff },
                                 ABS(copy.sph.elevation * 2 / M_PI * 255));
    scalar hit_dist; /* distance from camera in terms of d */
    const struct object_t *hit_obj = scene_intersections(scene, orig, d, &hit_dist, avoid);

    struct rgb_t reflected = {0, 0, 0};
    int specular = 255;

    /* by default */
    scalar shade_total = 1.0;

    if(hit_obj)
    {
        //printf("pow!\n");
        primary = hit_obj->color;

        /* shade */

        vector pt = d;
        pt = vect_mul(pt, hit_dist);
        pt = vect_add(pt, orig);

        vector normal = normal_at_point(pt, hit_obj);

        shade_total = 0;

        for(int i = 0; i < scene->n_lights; ++i)
        {
            /* get vector to light */
            vector light_dir = vect_sub(scene->lights[i].position, pt);

            scalar light_dist = vect_abs(light_dir);

            light_dir = vect_normalize(light_dir);

            /* see if light is occluded */
            scalar nearest;
            const struct object_t *obj = scene_intersections(scene, pt, light_dir, &nearest, hit_obj);

            if(obj && nearest < light_dist)
                continue;

            scalar shade = vect_dot(normal, light_dir);
            if(shade > 0)
                shade_total += shade * scene->lights[i].intensity;
        }

        if(shade_total > 1)
            shade_total = 1;

        specular = 255 - hit_obj->specularity;
        /* reflections */
        if(specular != 255 && max_iters > 0)
        {
            vector ref = reflect_ray(pt, d, normal, hit_obj);
            reflected = trace_ray(scene, pt, ref, max_iters - 1, hit_obj);
        }
    }

    struct rgb_t mixed = blend(primary, reflected, specular);

    scalar diffuse = 1 - scene->ambient;
    mixed.r *= (scene->ambient + diffuse * shade_total);
    mixed.g *= (scene->ambient + diffuse * shade_total);
    mixed.b *= (scene->ambient + diffuse * shade_total);

    return mixed;
}

void render_lines(unsigned char *fb, int w, int h,
                  const struct scene_t *scene,
                  const struct camera_t *cam,
                  int start, int end)
{
    scalar scale_x = cam->fov_x / w, scale_y = cam->fov_y / h;

    vector direction = cam->direction;
    vect_to_sph(&direction);

    for(int y = start; y < end; ++y)
    {
        for(int x = 0; x < w; ++x)
        {
            /* trace a ray from the camera into the scene */
            /* figure out how to rotate the offset vector to suit the
             * current pixel */

            /* rot in range of [-fov / 2, fov / 2) */
            scalar rot_x = (x - w / 2) * scale_x, rot_y = (y - h / 2) * scale_y;

            /* rotate the offset vector */

            vector d = direction;
            d.sph.elevation -= rot_y;
            d.sph.azimuth += rot_x;

            vect_to_rect(&d);

            //printf("(%d, %d) maps to (%f, %f, %f)\n", x, y, d.rect.x, d.rect.y, d.rect.z);

            /* cam->origin and d now form the camera ray */

            struct rgb_t color = trace_ray(scene, cam->origin, d, MAX_BOUNCES, NULL);

            fb[y * w * 3 + 3 * x] = color.b;
            fb[y * w * 3 + 3 * x + 1] = color.g;
            fb[y * w * 3 + 3 * x + 2] = color.r;
        }
    }
}

struct renderinfo_t {
    unsigned char *fb;
    int w, h;
    const struct scene_t *scene;
    const struct camera_t *cam;
    int start, end;

};

void *thread(void *ptr)
{
    struct renderinfo_t *info = ptr;
    render_lines(info->fb, info->w, info->h, info->scene, info->cam, info->start, info->end);
    return NULL;
}

void render_scene(unsigned char *fb, int w, int h,
                  const struct scene_t *scene,
                  const struct camera_t *cam, int n_threads)
{
    struct renderinfo_t *info = malloc(sizeof(struct renderinfo_t) * n_threads);
    pthread_t *threads = malloc(sizeof(pthread_t) * n_threads);
    for(int i = 0; i < n_threads; ++i)
    {
        info[i].fb = fb;
        info[i].w = w;
        info[i].h = h;
        info[i].scene = scene;
        info[i].cam = cam;
        info[i].start = h * i / n_threads;
        info[i].end = h * (i + 1) / n_threads;
        pthread_create(threads + i, NULL, thread, info + i);
    }

    for(int i = 0; i < n_threads; ++i)
        pthread_join(threads[i], NULL);

    free(info);
}

int main()
{
    struct scene_t scene;
    scene.bg.r = 0x87;
    scene.bg.g = 0xce;
    scene.bg.b = 0xeb;
    scene.ambient = .2;

    struct object_t sph[4];
    sph[0].type = SPHERE;
    sph[0].sphere.center = (vector) { RECT, {0, 1, 1 } };
    sph[0].sphere.radius = 1;
    sph[0].color = (struct rgb_t){0, 0, 0xff};
    sph[0].specularity = 40;

    sph[1].type = SPHERE;
    sph[1].sphere.center = (vector) { RECT, {0, 1, -1 } };
    sph[1].sphere.radius = 1;
    sph[1].color = (struct rgb_t){0, 0, 0xff};
    sph[1].specularity = 40;

    sph[2].type = SPHERE;
    sph[2].sphere.center = (vector) { RECT, {0, 1, -3 } };
    sph[2].sphere.radius = 1;
    sph[2].color = (struct rgb_t){0xff, 0xff, 0xff};
    sph[2].specularity = 0xf0;

    sph[3].type = PLANE;
    sph[3].plane.point = (vector) { RECT, {0, 0, 100 } };
    sph[3].plane.normal = (vector) { RECT, {0, 1, 0 } };
    sph[3].color = (struct rgb_t) {0, 0xff, 0};
    sph[3].specularity = 0;

    struct light_t lights[1];
    lights[0].position = (vector) { RECT, {0, 10, 0} };
    lights[0].intensity = 2;

    scene.objects = sph;
    scene.n_objects = 4;
    scene.lights = lights;
    scene.n_lights = 1;

    struct camera_t cam;
    cam.origin = (vector){ RECT, {-5, 0, 0} };
    cam.direction = (vector){ RECT, {0, 0, 1} };
    cam.fov_x = M_PI / 2;
    cam.fov_y = M_PI / 2 * HEIGHT / WIDTH;

    unsigned char *fb = malloc(WIDTH * HEIGHT * 3);

#if 0
    render_scene(fb, WIDTH, HEIGHT, &scene, &cam, 2);
    FILE *f = fopen("test.ppm", "w");
    fprintf(f, "P6\n%d %d\n%d\n", WIDTH, HEIGHT, 255);
    fwrite(fb, WIDTH * HEIGHT, 3, f);
    fclose(f);
    free(fb);
    return 0;

#else
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Surface *screen = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE | SDL_FULLSCREEN);
    SDL_EnableKeyRepeat(500, 50);

    while(1)
    {
        lights[0].position.rect.z += .05;
#ifdef MOUSELOOK
        /* mouse look */
        int x, y;
        unsigned mouse = SDL_GetMouseState(&x, &y);
        x -= WIDTH / 2;
        y -= HEIGHT / 2;
        vect_to_sph(&cam.direction);
        cam.direction.sph.azimuth += M_PI/30 * SIGN(x)*SQR((scalar)x / WIDTH);
        cam.direction.sph.elevation += M_PI/30 * SIGN(y)*SQR((scalar)y / HEIGHT);
#endif

        render_scene(fb, WIDTH, HEIGHT, &scene, &cam, 2);
        memcpy(screen->pixels, fb, WIDTH * HEIGHT * 3);
        SDL_UpdateRect(screen, 0, 0, 0, 0);
        SDL_Event e;
        //printf("camera at %f, %f, %f\n", cam.origin.rect.x, cam.origin.rect.y, cam.origin.rect.z);
        while(SDL_PollEvent(&e))
        {
            switch(e.type)
            {
            case SDL_QUIT:
                free(fb);
                return 0;
            case SDL_KEYDOWN:
                switch(e.key.keysym.sym)
                {
                case SDLK_ESCAPE:
                    free(fb);
                    SDL_Quit();
                    return 0;
                case SDLK_UP:
                    cam.origin.rect.y += .1;
                    break;
                case SDLK_DOWN:
                    cam.origin.rect.y -= .1;
                    break;
                case SDLK_a:
                    vect_to_sph(&cam.direction);
                    cam.direction.sph.azimuth += M_PI/2;
                    cam.origin = vect_sub(cam.origin, vect_mul(cam.direction, MOVE_FACTOR));
                    cam.direction.sph.azimuth -= M_PI/2;
                    break;
                case SDLK_d:
                    vect_to_sph(&cam.direction);
                    cam.direction.sph.azimuth -= M_PI/2;
                    cam.origin = vect_sub(cam.origin, vect_mul(cam.direction, MOVE_FACTOR));
                    cam.direction.sph.azimuth += M_PI/2;
                    break;
                case SDLK_w:
                    cam.origin = vect_add(cam.origin, vect_mul(cam.direction, MOVE_FACTOR));
                    break;
                case SDLK_s:
                    cam.origin = vect_sub(cam.origin, vect_mul(cam.direction, MOVE_FACTOR));
                    break;
                case SDLK_MINUS:
                    cam.fov_x += M_PI/36;
                    cam.fov_y = cam.fov_x * HEIGHT/WIDTH;
                    break;
                case SDLK_EQUALS:
                    cam.fov_x -= M_PI/36;
                    cam.fov_y = cam.fov_x * HEIGHT/WIDTH;
                    break;
                case SDLK_SPACE:
                    cam.origin.rect.y += .1;
                    break;
                case SDLK_LSHIFT:
                    cam.origin.rect.y -= .1;
                    break;
                case SDLK_LEFT:
                {
                    vect_to_sph(&cam.direction);
                    cam.direction.sph.azimuth -= M_PI/180;
                    break;
                }
                case SDLK_RIGHT:
                {
                    vect_to_sph(&cam.direction);
                    cam.direction.sph.azimuth += M_PI/180;
                    break;
                }
                }
                break;
            }
        }
    }
#endif
}
