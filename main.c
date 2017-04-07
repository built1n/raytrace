#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <pthread.h>

#include "vector.h"

#include <SDL/SDL.h>
#include <SDL/SDL_video.h>

#define WIDTH 320
#define HEIGHT 240

#define MAX(a, b) ((a>b)?(a):(b))
#define MIN(a, b) ((a<b)?(a):(b))
#define SQR(a) ((a)*(a))
#define SIGN(x) ((x)<0?-1:1)

#define MAX_BOUNCES 50
#define MIN_BOUNCES 2
#define MOVE_FACTOR .15
#define TARGET_MS 50

#define N_LIGHTS 1

#define PPMOUT

#define MOUSELOOK

struct rgb_t { unsigned char r, g, b; };

struct object_t {
    enum { SPHERE, PLANE, TRI } type;
    union {
        struct { vector center; scalar radius; } sphere;
        struct { vector point, normal; } plane;
        struct { vector points[3], normal, u, v; scalar uu, uv, vv, dn; } tri;
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
    vector origin; /* position */
    vector direction;
    scalar fov_x, fov_y; /* radians */
};

void preprocess_object(struct object_t *obj)
{
    switch(obj->type)
    {
    case TRI:
        obj->tri.u = vect_sub(obj->tri.points[1], obj->tri.points[0]);
        obj->tri.v = vect_sub(obj->tri.points[2], obj->tri.points[0]);
        obj->tri.normal = vect_cross(obj->tri.u, obj->tri.v);
        obj->tri.uu = vect_dot(obj->tri.u, obj->tri.u);
        obj->tri.uv = vect_dot(obj->tri.u, obj->tri.v);
        obj->tri.vv = vect_dot(obj->tri.v, obj->tri.v);
        obj->tri.dn = SQR(obj->tri.uv) - obj->tri.uu * obj->tri.vv;
        break;
    }
}

/* { o, d } form a ray */
/* point of intersection is *t * d units away */
inline bool object_intersects(const struct object_t *obj, vector o, vector d, scalar *t)
{
    assert(o.type == RECT);
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
    case TRI:
    {
        if(vect_abs(obj->tri.normal) == 0)
        {
            //printf("degenerate triangle\n");
            return false;
        }
        scalar denom = vect_dot(obj->tri.normal, d);
        /* doesn't intersect plane of triangle */
        if(!denom)
        {
            //printf("parallel\n");
            return false;
        }
        scalar t1 = vect_dot(obj->tri.normal, vect_sub(obj->tri.points[0], o)) / denom;
        /* behind camera */
        if(t1 <= 0)
        {
            //printf("behind camera\n");
            return false;
        }

        vector pt = vect_add(vect_mul(d, t1), o);
        vect_to_rect(&pt);
        scalar wu, wv;
        vector w = vect_sub(pt, obj->tri.points[0]);
        wu = vect_dot(w, obj->tri.u);
        wv = vect_dot(w, obj->tri.v);
        scalar s1 = (obj->tri.uv * wv - obj->tri.vv * wu) / obj->tri.dn;
        if(s1 < 0. || s1 > 1.)
        {
            //printf("not inside\n");
            return false;
        }
        scalar s2 = (obj->tri.uv * wu - obj->tri.uu * wv) / obj->tri.dn;
        if(s2 < 0. || (s1 + s2) > 1.)
        {
            //printf("not inside\n");
            return false;
        }
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
    case TRI:
        normal = vect_negate(obj->tri.normal);
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

    struct rgb_t primary = blend((struct rgb_t) {0, 0x96, 0xff}, (struct rgb_t) { 0xfe, 0xfe, 0xfe },
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
                shade_total += shade * scene->lights[i].intensity * 1 / SQR(light_dist);
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

        scalar diffuse = 1 - scene->ambient;
        primary.r *= (scene->ambient + diffuse * shade_total);
        primary.g *= (scene->ambient + diffuse * shade_total);
        primary.b *= (scene->ambient + diffuse * shade_total);
    }

    struct rgb_t mixed = blend(primary, reflected, specular);


    return mixed;
}

vector ray_to_pixel(vector origin, vector direction, int x, int y, int w, int h, const struct camera_t *cam)
{
    scalar scale_x = tan(.5 * cam->fov_x / w), scale_y = tan(.5 * cam->fov_y / h);

    /* rot in range of [-fov / 2, fov / 2) */
    scalar rot_x = (x - w / 2) * scale_x, rot_y = (y - h / 2) * scale_y;

    /* rotate the offset vector */

    vector d = direction;
    assert(d.type == SPH);

    d.sph.elevation -= rot_y;
    d.sph.azimuth += rot_x;

    vect_to_rect(&d);
    return d;
}

void render_lines(unsigned char *fb, int w, int h,
                  const struct scene_t *scene,
                  const struct camera_t *cam,
                  int start, int end, int bounces, int worker)
{
    scalar scale_x = tan(.5 * cam->fov_x / w), scale_y = tan(.5 * cam->fov_y / h);

    vector direction = cam->direction;
    vect_to_sph(&direction);

    for(int y = 0; y < h; ++y)
    {
        for(int x = start; x < end; ++x)
        {
            /* trace a ray from the camera into the scene */
            /* figure out how to rotate the offset vector to suit the
             * current pixel */

            /* rot in range of [-fov / 2, fov / 2) */
            scalar rot_x = (x - w / 2) * scale_x, rot_y = (y - h / 2) * scale_y;

            vector d = ray_to_pixel(cam->origin, direction, x, y, w, h, cam);

            /* cam->origin and d now form the camera ray */

            struct rgb_t color = trace_ray(scene, cam->origin, d, bounces, NULL);

#ifdef PPMOUT
            fb[y * w * 3 + 3 * x] = color.r;
            fb[y * w * 3 + 3 * x + 1] = color.g;
            fb[y * w * 3 + 3 * x + 2] = color.b;
#else
            fb[y * w * 3 + 3 * x] = color.b;
            fb[y * w * 3 + 3 * x + 1] = color.g;
            fb[y * w * 3 + 3 * x + 2] = color.r;
#endif

        }
#ifdef PPMOUT
        printf("Worker %d: %d%% (%d/%d)\n", worker, 100 * (y+1) / h, y+1, h);
#endif
    }
}

struct renderinfo_t {
    unsigned char *fb;
    int w, h;
    const struct scene_t *scene;
    const struct camera_t *cam;
    int start, end;
    int bounces;
    int worker;
};

void *thread(void *ptr)
{
    struct renderinfo_t *info = ptr;
    render_lines(info->fb, info->w, info->h, info->scene, info->cam, info->start, info->end, info->bounces, info->worker);
    return NULL;
}

void render_scene(unsigned char *fb, int w, int h,
                  const struct scene_t *scene,
                  const struct camera_t *cam, int n_threads, int n_bounces)
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
        info[i].start = w * i / n_threads;
        info[i].end = w * (i + 1) / n_threads;
        info[i].bounces = n_bounces;
        info[i].worker = i;
        pthread_create(threads + i, NULL, thread, info + i);
    }

    for(int i = 0; i < n_threads; ++i)
        pthread_join(threads[i], NULL);

    free(info);
}

scalar rand_norm(void)
{
    return rand() / (scalar)RAND_MAX;
}

void preprocess_scene(const struct scene_t *scene)
{
    for(int i = 0; i < scene->n_objects; ++i)
    {
        preprocess_object(scene->objects + i);
    }
}

int main()
{
    struct scene_t scene;
    scene.bg.r = 0x87;
    scene.bg.g = 0xce;
    scene.bg.b = 0xeb;
    scene.ambient = .2;

    struct object_t sph[8];

#if 1
    sph[0].type = SPHERE;
    sph[0].sphere.center = (vector) { RECT, {1, 1, 0 } };
    sph[0].sphere.radius = 1;
    sph[0].color = (struct rgb_t){0, 0, 0xff};
    sph[0].specularity = 0xf0;

    sph[1].type = SPHERE;
    sph[1].sphere.center = (vector) { RECT, {-1, 1, 0 } };
    sph[1].sphere.radius = 1;
    sph[1].color = (struct rgb_t){0xff, 0, 0};
    sph[1].specularity = 40;

    sph[2].type = SPHERE;
    sph[2].sphere.center = (vector) { RECT, {-3, 1, 0 } };
    sph[2].sphere.radius = 1;
    sph[2].color = (struct rgb_t){0xff, 0xff, 0xff};
    sph[2].specularity = 0xf0;

    sph[3].type = PLANE;
    sph[3].plane.point = (vector) { RECT, {0, 0, 0 } };
    sph[3].plane.normal = (vector) { RECT, {0, 1, 0 } };
    sph[3].color = (struct rgb_t) {0, 0xff, 0};
    sph[3].specularity = 0;
#endif

    sph[4].type = TRI;
    sph[4].tri.points[0] = (vector) { RECT, {5, 0, 0 } };
    sph[4].tri.points[1] = (vector) { RECT, {5, 5, 0 } };
    sph[4].tri.points[2] = (vector) { RECT, {0, 5, 0 } };
    sph[4].color = (struct rgb_t) {0xff, 0, 0};
    sph[4].specularity = 0x30;

    /* distribute evenly in a sphere of r = 1 */
    struct light_t lights[N_LIGHTS];
    for(int i = 0; i < N_LIGHTS; ++i)
    {
        lights[i].position = vect_add((vector) { RECT, {5, 10, -5} },
                                      (vector) { SPH, { .sph.r = rand_norm(),
                                                  .sph.elevation = 2 * M_PI * (rand_norm() - .5),
                                                  .sph.azimuth = 4 * M_PI * (rand_norm() - .5) } } );
        lights[i].intensity = 200 / N_LIGHTS;
    }

    scene.objects = sph;
    scene.n_objects = 5;
    scene.lights = lights;
    scene.n_lights = N_LIGHTS;

    struct camera_t cam;
    cam.origin = (vector){ RECT, {0, 1, -5} };
    cam.direction = (vector){ RECT, {1, 0, 0} };
    cam.fov_x = M_PI;
    cam.fov_y = M_PI * HEIGHT / WIDTH;

    unsigned char *fb = malloc(WIDTH * HEIGHT * 3);

    preprocess_scene(&scene);

#ifdef PPMOUT
    render_scene(fb, WIDTH, HEIGHT, &scene, &cam, 2, MAX_BOUNCES);
    FILE *f = fopen("test.ppm", "w");
    fprintf(f, "P6\n%d %d\n%d\n", WIDTH, HEIGHT, 255);
    fwrite(fb, WIDTH * HEIGHT, 3, f);
    fclose(f);
    free(fb);
    return 0;

#else
    /* auto-adjusting */
    int bounces = MIN_BOUNCES;

    SDL_Init(SDL_INIT_VIDEO);
    SDL_Surface *screen = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE);
    SDL_EnableKeyRepeat(500, 50);

    int ts = SDL_GetTicks();

    while(1)
    {
#ifdef MOUSELOOK
        /* mouse look */
        int x, y;
        unsigned mouse = SDL_GetMouseState(&x, &y);
        x -= WIDTH / 2;
        y -= HEIGHT / 2;
        vect_to_sph(&cam.direction);
        cam.direction.sph.azimuth += M_PI/10 * SIGN(x)*SQR((scalar)x / WIDTH);
        cam.direction.sph.elevation += M_PI/10 * SIGN(y)*SQR((scalar)y / HEIGHT);
        if(mouse & SDL_BUTTON(1))
        {
            vector d = ray_to_pixel(cam.origin, cam.direction, x + WIDTH/2, y + HEIGHT/2, WIDTH, HEIGHT, &cam);
            scalar dist;
            struct object_t *obj = scene_intersections(&scene, cam.origin, d, &dist, NULL);
            if(obj)
            {
                obj->color = (struct rgb_t) { 0xff, 0, 0xff };
                printf("Clicked object at %d, %d\n", x, y);
            }
        }
#endif

        render_scene(fb, WIDTH, HEIGHT, &scene, &cam, 2, bounces);
        memcpy(screen->pixels, fb, WIDTH * HEIGHT * 3);
        SDL_UpdateRect(screen, 0, 0, 0, 0);

        int now = SDL_GetTicks();
        int dt = now - ts;

        if(dt < TARGET_MS && bounces < MAX_BOUNCES)
        {
            /* too fast! */
            bounces *= 2;
            if(bounces > MAX_BOUNCES)
                bounces = MAX_BOUNCES;
        }
        else if(dt > TARGET_MS && bounces > MIN_BOUNCES)
        {
            bounces /= 2;
            if(bounces < MIN_BOUNCES)
                bounces = MIN_BOUNCES;
        }

        ts = now;

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
                {
                    vector tmp = cam.direction;
                    vect_to_sph(&tmp);
                    tmp.sph.azimuth -= M_PI/2;
                    cam.origin = vect_add(cam.origin, vect_mul(tmp, MOVE_FACTOR));
                    break;
                }
                case SDLK_d:
                {
                    vector tmp = cam.direction;
                    vect_to_sph(&tmp);
                    tmp.sph.azimuth += M_PI/2;
                    cam.origin = vect_add(cam.origin, vect_mul(tmp, MOVE_FACTOR));
                    break;
                }
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
