#include <assert.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <time.h>
#include <termios.h>

#ifndef SUBPIXEL
#define SUBPIXEL 4
#endif

#ifndef MAX_HITS
#define MAX_HITS 48
#endif

#ifndef FRAMES
#define FRAMES 100
#endif

#ifndef STARS
#define STARS 64
#endif

struct col {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
};

struct v3 {
	float x;
	float y;
	float z;
};
const struct v3 ZERO = {0, 0, 0};

struct v4 {
	float x; /* r */
	float y; /* i */
	float z; /* j */
	float w; /* k */
};

struct mat4 {
	struct v4 col[4];
} IDENTITY = {{
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0},
	{0.0, 0.0, 0.0, 1.0},
}};

struct hitpt {
	struct v3 pos;
	struct v3 norm;
};

struct g_sphere {
	float rad;
	struct v3 pos;
};

struct g_line {
	struct v3 org;
	struct v3 dir;
};

struct illum {
	enum {ILLUM_NONE, ILLUM_FLAT, ILLUM_DIFFUSE} mode;
	struct v3 dir;
	float pow;
};

struct geom {
	enum {G_SPHERE} kind;
	union {
		struct g_sphere sphere;
	} u;
	struct col col;
	struct illum illum;
};

struct hit {
	struct geom *geom;
	int hits;
	struct hitpt close;
	struct hitpt far;
};

struct scene {
	enum {S_LIST, S_GEOM} kind;
	union {
		struct scene *list;
		struct geom geom;
	} u;
	struct scene *next;
};

struct v4 v32v4(struct v3 v) {
	struct v4 ret;
	memcpy(&ret, &v, sizeof(v));
	ret.w = 1.0;
	return ret;
}

struct v3 v42v3(struct v4 v) {
	return (struct v3){v.x/v.w, v.y/v.w, v.z/v.w};
}

float dot(struct v3 a, struct v3 b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

float dot4(struct v4 a, struct v4 b) {
	return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

struct v3 scale(struct v3 a, float u) {
	return (struct v3){a.x*u, a.y*u, a.z*u};
}

struct v3 add(struct v3 a, struct v3 b) {
	return (struct v3){a.x+b.x, a.y+b.y, a.z+b.z};
}

struct v3 neg(struct v3 a) {
	return (struct v3){-a.x, -a.y, -a.z};
}

int is_zero(struct v3 v) {
	return v.x == 0 && v.y == 0 && v.z == 0;
}

struct v3 normalize(struct v3 v) {
	if(is_zero(v)) {
		return ZERO;
	}
	return scale(v, 1.0/sqrt(dot(v, v)));
}

void printmat4(FILE *s, struct mat4 m) {
#define sym "%.03f"
#define row sym " " sym " " sym " " sym
#define rowval(n) m.col[n].x, m.col[n].y, m.col[n].z, m.col[n].w
	fprintf(stderr, row "\n" row "\n" row "\n" row "\n",
			rowval(0), rowval(1), rowval(2), rowval(3));
#undef sym
#undef row
#undef rowval
}

struct v4 mat4mulv4(struct mat4 mat, struct v4 v) {
	return (struct v4){
		dot4(v, mat.col[0]),
		dot4(v, mat.col[1]),
		dot4(v, mat.col[2]),
		dot4(v, mat.col[3]),
	};
}
#define transform(m, v) (v42v3(mat4mulv4((m), v32v4((v)))))

struct mat4 transpose(struct mat4 m) {
	return (struct mat4){{
#define access(f) {m.col[0].f, m.col[1].f, m.col[2].f, m.col[3].f}
		access(x),
		access(y),
		access(z),
		access(w),
	}};
#undef access
}

struct mat4 mat4mulmat4(struct mat4 a, struct mat4 b) {
	b = transpose(b);
	return (struct mat4){{
			mat4mulv4(b, a.col[0]),
			mat4mulv4(b, a.col[1]),
			mat4mulv4(b, a.col[2]),
			mat4mulv4(b, a.col[3]),
	}};
}

struct mat4 mat4trans(struct v3 t) {
	return (struct mat4){{
		{1.0, 0.0, 0.0, t.x},
		{0.0, 1.0, 0.0, t.y},
		{0.0, 0.0, 1.0, t.z},
		{0.0, 0.0, 0.0, 1.0},
	}};
}

struct v4 qrot(float rad, struct v3 axis) {
	struct v3 a = normalize(axis);
	float s = sin(rad / 2.0);
	return (struct v4) {
		cos(rad / 2.0),
		s * a.x,
		s * a.y,
		s * a.z,
	};
}

struct mat4 mat4rot(struct v4 q) {
	/* assume q is unit because I can't be bothered to compute s */
	float a = q.y*q.z + q.w*q.x;
	float b = q.y*q.w + q.z*q.x;
	float c = q.z*q.w + q.y*q.x;
	return (struct mat4){{
		{1.0-2.0*(q.z*q.z + q.w*q.w), 2.0*a, 2.0*b, 0.0},
		{2.0*a, 1.0-2.0*(q.y*q.y + q.w*q.w), 2.0*c, 0.0},
		{2.0*b, 2.0*c, 1.0-2.0*(q.y*q.y + q.z*q.z), 0.0},
		{0.0, 0.0, 0.0, 1.0},
	}};
}

float sqr(float a) { return a*a; }

struct col mix(struct col from, struct col to, float u) {
	float iu;

	u = u > 1.0 ? 1.0 : u;
	u = u < 0.0 ? 0.0 : u;
	iu = 1.0 - u;

	return (struct col){
		from.r*iu + to.r*u,
		from.g*iu + to.g*u,
		from.b*iu + to.b*u,
		from.a*iu + to.a*u,
	};
}

struct v3 col2v3(struct col c, float *a) {
	*a = c.a/255.0;
	return (struct v3){c.r/255.0, c.g/255.0, c.b/255.0};
}

struct col v32col(struct v3 v, float a) {
	return (struct col){
		v.x * 255,
		v.y * 255,
		v.z * 255,
		a * 255,
	};
}

int hit_sphere(struct g_sphere *sph, struct g_line *line, struct hitpt *close, struct hitpt *far) {
	struct v3 od = add(line->org, neg(sph->pos));
	float b = dot(line->dir, od);
	float discrim = sqr(b) - (dot(od, od) - sqr(sph->rad));
	if(discrim < 0) {
		return 0;
	} else {
		close->pos = add(line->org, scale(line->dir, (-b) - sqrt(discrim)));
		close->norm = normalize(add(close->pos, neg(sph->pos)));
		far->pos = add(line->org, scale(line->dir, (-b) + sqrt(discrim)));
		far->norm = normalize(add(far->pos, neg(sph->pos)));
		return discrim == 0 ? 1 : 2;
	}
}

int hit_geom(struct geom *g, struct g_line *line, struct hitpt *close, struct hitpt *far) {
	switch(g->kind) {
		case G_SPHERE:
			return hit_sphere(&g->u.sphere, line, close, far);
			break;
	}
	assert(0);
}

int all_hits(struct scene *s, struct g_line *line, struct hit *hits, size_t sz_hits) {
	int count = 0, sub;
	struct hitpt close, far;

	while(s && sz_hits > 0) {
		switch(s->kind) {
			case S_LIST:
				sub = all_hits(s->u.list, line, hits, sz_hits);
				sz_hits -= sub;
				hits += sub;
				count += sub;
				break;

			case S_GEOM:
				sub = hit_geom(&s->u.geom, line, &close, &far);
				if(sub) {
					hits->geom = &s->u.geom;
					hits->hits = sub;
					hits->close = close;
					hits->far = far;
					hits++;
					sz_hits--;
					count++;
				}
				break;

			default:
				assert(0);
				break;
		}
		s = s->next;
	}
	return count;
}

int _sort_by_dist(struct hit *a, struct hit *b, struct v3 *orig) {
	struct v3 da = add(a->close.pos, neg(*orig));
	struct v3 db = add(b->close.pos, neg(*orig));
	float dista2 = dot(da, da), distb2 = dot(db, db);
	return dista2 < distb2 ? -1 : (dista2 == distb2 ? 0 : 1);
}

struct col query(struct scene *s, struct g_line *line, struct col bkgd, struct hit *hits, size_t sz_hits, int *num_hits) {
	int count = all_hits(s, line, hits, sz_hits), i;
	struct col ret = bkgd, frag;
	struct v3 orig = line->org;
	struct illum il;
	float d;
	int na;

	if(num_hits) *num_hits = count;

	qsort_r(hits, count, sizeof(*hits), (int(*)(const void*,const void*,void*))_sort_by_dist, &orig);

	for(i = count - 1; i >= 0; i--) {
		frag = hits[i].geom->col;
		il = hits[i].geom->illum;
		switch(il.mode) {
			case ILLUM_NONE:
				break;
			case ILLUM_FLAT:
				if(dot(hits[0].close.norm, il.dir) < 0)
					frag.a = 0;
				break;
			case ILLUM_DIFFUSE:
				d = dot(hits[0].close.norm, il.dir);
				if(d < 0) d = 0;
				frag.a *= powf(d, il.pow);
				break;
		}
		// fprintf(stderr, "m %hhu,%hhu,%hhu,%hhu + %hhu,%hhu,%hhu,%hhu = ", ret.r, ret.g, ret.b, ret.a, frag.r, frag.g, frag.b, frag.a);
		ret = mix(ret, frag, (float)frag.a/255.0);
		na = (int)ret.a + (int)frag.a;
		ret.a = (uint8_t)(na >= 0xff ? 0xff : na);
		// fprintf(stderr, "%hhu,%hhu,%hhu,%hhu\n", ret.r, ret.b, ret.g, ret.a);
	}

	// fprintf(stderr, "q %hhu,%hhu,%hhu,%hhu\n", ret.r, ret.g, ret.b, ret.a);
	return ret;
}

struct col query_sp(struct scene *s, struct g_line *line, struct col bkgd, struct hit *hits, size_t sz_hits, int *num_hits, int subpixel, struct v3 off_u, struct v3 off_v) {
	float a, b;
	struct v3 total = col2v3(bkgd, &a), samp;
	struct col scol;
	float u = 1.0 / sqr(subpixel);
	struct v3 du = scale(off_u, 1.0/subpixel);
	struct v3 dv = scale(off_v, 1.0/subpixel);
	int iu, iv, count;
	struct g_line off_line = *line;

	if(num_hits) *num_hits = 0;

	for(iu = 0; iu < subpixel; iu++) {
		for(iv = 0; iv < subpixel; iv++) {
			off_line.org = add(
					line->org,
					add(
						scale(du, iu),
						scale(dv, iv)
					)
			);
			scol = query(s, &off_line, bkgd, hits, sz_hits, &count);
			samp = col2v3(scol, &b);
			// fprintf(stderr, "s %hhu,%hhu,%hhu,%hhu;%f,%f,%f,%f\n", scol.r, scol.g, scol.b, scol.a, samp.x, samp.y, samp.z, b);
			a += u*b;
			total = add(total, scale(samp, u));
			if(num_hits) *num_hits += count;
		}
	}

	// fprintf(stderr, "t %f,%f,%f,%f\n", total.x, total.y, total.z, a);
	return v32col(total, a);
}

struct scene ROOT = {
	.kind = S_GEOM,
	.u = {
		.geom = {
			.kind = G_SPHERE,
			.u = {
				.sphere = {
					.rad = 0.5,
					.pos = {0.0, 0.0, -1.0},
				},
			},
			.col = {255, 255, 255, 255},
			.illum = {
				.mode = ILLUM_DIFFUSE,
				.dir = {1, 0, 0},
				.pow = 1.0/2.0,
			},
		},
	},
	/* .next = &(struct scene){
		.kind = S_GEOM,
		.u = {
			.geom = {
				.kind = G_SPHERE,
					.u = {
						.sphere = {
							.rad = 0.5,
							.pos = {0.15, 0.0, -0.5},
						},
					},
				.col = {0, 0, 0, 255},
				.illum = {
					.mode = ILLUM_NONE,
					.dir = {0, 0, 0},
				},
			},
		},
		.next = NULL,
	},
	*/
	.next = NULL,
};

void _matrix_test() {
	struct mat4 mtrans = mat4trans((struct v3){1.0, 2.0, 3.0});
	struct mat4 mrot = mat4rot(qrot(M_PI/2.0, (struct v3){0.0, 1.0, 0.0}));
	struct mat4 multest1 = mat4mulmat4(IDENTITY, mtrans);
	struct mat4 multest2 = mat4mulmat4(mtrans, IDENTITY);
	struct mat4 mmul1 = mat4mulmat4(mtrans, mrot);
	struct mat4 mmul2 = mat4mulmat4(mrot, mtrans);
	struct v3 pt = {1.0, 0.0, 0.0}, out;

	fprintf(stderr, "Identity:\n");
	printmat4(stderr, IDENTITY);
	fprintf(stderr, "Translation:\n");
	printmat4(stderr, mtrans);
	fprintf(stderr, "Rotation:\n");
	printmat4(stderr, mrot);
	fprintf(stderr, "Identity * Translation:\n");
	printmat4(stderr, multest1);
	fprintf(stderr, "Translation * Identity:\n");
	printmat4(stderr, multest2);
	fprintf(stderr, "Translation * Rotation:\n");
	printmat4(stderr, mmul1);
	fprintf(stderr, "Rotation * Translation:\n");
	printmat4(stderr, mmul2);

	fprintf(stderr, "Point: %f,%f,%f\n", pt.x, pt.y, pt.z);
	out = transform(mtrans, pt);
	fprintf(stderr, "Translation * Point: %f,%f,%f\n", out.x, out.y, out.z);
	out = transform(mrot, pt);
	fprintf(stderr, "Rotation * Point: %f,%f,%f\n", out.x, out.y, out.z);
	out = transform(mmul1, pt);
	fprintf(stderr, "(Translation * Rotation) * Point: %f,%f,%f\n", out.x, out.y, out.z);
	out = transform(mmul2, pt);
	fprintf(stderr, "(Rotation * Translation) * Point: %f,%f,%f\n", out.x, out.y, out.z);
}

int main() {
	int tty;
	struct winsize ws;
	int row, col, count, frame, star;
	struct col pixel, bkgd = {0, 0, 0, 0};
	struct g_line ray;
	struct scene *sc = &ROOT, *cur;
	struct hit hits[MAX_HITS];
	float arat, cfac, rfac;
	float *fade_pwr = NULL, *fade_ofs = NULL;

	/* struct geom *occluder = &sc->next->u.geom; */
	struct geom *moon = &sc->u.geom;
	struct scene *last = sc;
	struct scene *stars = NULL, **star_cur = &stars;

	char *sbuf, *sptr;
	size_t ssz, spz;

	struct v3 du, dv;

	struct mat4 transf = mat4mulmat4(
		mat4mulmat4(
			mat4trans((struct v3){0.0, 0.0, -1.0}),
			mat4rot(qrot(-0.01, (struct v3){0.0, 1.0, 0.0}))
		),
		mat4trans((struct v3){0.0, 0.0, 1.0})
	);

#ifdef MATRIX_TEST
	_matrix_test();
	return 0;
#endif

	srand48(time(NULL));
	srand(time(NULL));

	if((tty = open("/dev/tty", O_RDWR)) < 0) {
		perror("Open tty (is there a controlling term?)");
		return 1;
	}

	if(ioctl(tty, TIOCGWINSZ, &ws) < 0) {
		perror("TIOCGWINSZ");
		return 1;
	}

	ssz = ws.ws_row * ws.ws_col * 32;
	setvbuf(stdout, NULL, _IOFBF, ssz);
	sbuf = malloc(sizeof(char) * ssz);
#define scrprintf(...) do {\
	int chr = snprintf(sptr, spz, __VA_ARGS__);\
	if(chr >= 0) {\
		chr = chr > spz ? spz : chr;\
		sptr += chr;\
		spz -= chr;\
	}\
} while(0);

	arat = (float)ws.ws_row / (float)ws.ws_col;
	if(arat > 1.0) {
		cfac = 1.0;
		rfac = 0.5 / arat;
	} else {
		rfac = 1.0;
		cfac = 0.5 / arat;
	}

	du = (struct v3){
		cfac/ws.ws_col,
		0.0,
		0.0,
	};
	dv = (struct v3){
		0.0,
		rfac/ws.ws_row,
		0.0,
	};

	fade_pwr = malloc(sizeof(float) * STARS);
	fade_ofs = malloc(sizeof(float) * STARS);
	sc->u.geom.col = (struct col){rand()%256, rand()%256, rand()%256, 255};

	for(star = 0; star < STARS; star++) {
		fade_pwr[star] = 1.0 + drand48();
		fade_ofs[star] = 0.25 * drand48();
		*star_cur = malloc(sizeof(struct scene));
		**star_cur = (struct scene){
			.kind = S_GEOM,
			.u = {
				.geom = {
					.kind = G_SPHERE,
					.u = {
						.sphere = {
							.rad = 0.01 + 0.05 * drand48(),
							.pos = {cfac * (2.0 * drand48() - 1.0), rfac * (2.0 * drand48() - 1.0), -2.0 * drand48()},
						},
					},
					.col = {rand()%256, rand()%256, rand()%256, 255},
					.illum = {
						.mode = ILLUM_NONE,
						.dir = ZERO,
						.pow = 0,
					},
				},
			},
			.next = NULL,
		};
		star_cur = &(*star_cur)->next;
	}
	last->next = stars;

	puts("\x1b[2J");
	fflush(stdout);

	for(frame = 0; frame < FRAMES; frame++) {
		sptr = sbuf;
		spz = ssz;
		memset(sbuf, 0, ssz);
		scrprintf("\x1b[H");
		/* occluder->u.sphere.pos.x = 2.0 * powf((FRAMES - frame - 1)/(float)FRAMES, 4.0); */
		moon->illum.dir = (struct v3){
			-sin(M_PI*frame/(float)FRAMES),
			0,
			cos(M_PI*frame/(float)FRAMES),
		};
		for(star = 0, cur = stars; cur; cur = cur->next, star++) {
			float tb = (FRAMES - frame - 1)/((1.0 - fade_ofs[star]) * FRAMES);
			if(tb < 0.0) tb = 0.0;
			if(tb > 1.0) tb = 1.0;
			cur->u.geom.col.a = (uint8_t)(255.0 * powf(tb, fade_pwr[star]));
			cur->u.geom.u.sphere.pos = transform(transf, cur->u.geom.u.sphere.pos);
		}

		for(row = 1; row < ws.ws_row; row++) {
			for(col = 0; col < ws.ws_col; col++) {
				ray.org = (struct v3){
					cfac*(2.0 * (float)col/(float)ws.ws_col - 1.0),
					rfac*(2.0 * (float)row/(float)ws.ws_row - 1.0),
					1.0,
				};
				ray.dir = (struct v3){0.0, 0.0, -1.0};
				pixel = query_sp(sc, &ray, bkgd, hits, MAX_HITS, &count, SUBPIXEL, du, dv);
#ifdef HITCOUNT
				scrprintf("\x1b[38;2;%hhu;%hhu;%hhum%1d", pixel.r, pixel.g, pixel.b, count % 10);
#else
				scrprintf("\x1b[48;2;%hhu;%hhu;%hhum ", pixel.r, pixel.g, pixel.b);
#endif
			}
		}
		puts(sbuf);
		fflush(stdout);
	}
	puts("\x1b[H");
	fflush(stdout);
	return 0;
}
