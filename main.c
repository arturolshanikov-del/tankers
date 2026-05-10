#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lodepng.h"

/* ---------- I/O wrappers ---------- */

static unsigned char* load_png(const char* filename, unsigned int* w, unsigned int* h)
{
    unsigned char* image = NULL;
    unsigned err = lodepng_decode32_file(&image, w, h, filename);
    if (err) {
        fprintf(stderr, "lodepng decode error %u: %s\n", err, lodepng_error_text(err));
        return NULL;
    }
    return image;
}

static int save_png(const char* filename, const unsigned char* rgba, unsigned w, unsigned h)
{
    unsigned err = lodepng_encode32_file(filename, rgba, w, h);
    if (err) {
        fprintf(stderr, "lodepng encode error %u: %s\n", err, lodepng_error_text(err));
        return 0;
    }
    return 1;
}

/* ---------- preprocessing ---------- */

/* Rec.601 luminance */
static void to_grayscale(const unsigned char* rgba, unsigned char* gray, int n)
{
    for (int i = 0; i < n; i++) {
        unsigned r = rgba[4 * i + 0];
        unsigned g = rgba[4 * i + 1];
        unsigned b = rgba[4 * i + 2];
        gray[i] = (unsigned char)((299u * r + 587u * g + 114u * b) / 1000u);
    }
}

/* 5x5 Gaussian blur (sigma ~ 1.0). Edge pixels normalised by actual weight sum. */
static void gaussian_5x5(const unsigned char* src, unsigned char* dst, int w, int h)
{
    static const int K[5][5] = {
        { 1,  4,  6,  4, 1},
        { 4, 16, 24, 16, 4},
        { 6, 24, 36, 24, 6},
        { 4, 16, 24, 16, 4},
        { 1,  4,  6,  4, 1}
    };
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int acc = 0, total = 0;
            for (int dy = -2; dy <= 2; dy++) {
                int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -2; dx <= 2; dx++) {
                    int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    int k = K[dy + 2][dx + 2];
                    acc   += k * src[yy * w + xx];
                    total += k;
                }
            }
            dst[y * w + x] = (unsigned char)(acc / (total ? total : 1));
        }
    }
}

/* ---------- thresholding ---------- */

/* Adaptive threshold: mean + k * stddev computed only over pixels where
   roi[i] is non-zero. Restricting statistics to the water region prevents
   the bright basemap from saturating the threshold. */
static unsigned char compute_threshold(const unsigned char* gray,
                                       const unsigned char* roi,
                                       int n, double k)
{
    double sum = 0.0;
    long   cnt = 0;
    for (int i = 0; i < n; i++) {
        if (!roi[i]) continue;
        sum += gray[i];
        cnt++;
    }
    if (cnt == 0) return 255;
    double mean = sum / (double)cnt;
    double var = 0.0;
    for (int i = 0; i < n; i++) {
        if (!roi[i]) continue;
        double d = (double)gray[i] - mean;
        var += d * d;
    }
    var /= (double)cnt;
    double t = mean + k * sqrt(var);
    if (t > 255.0) t = 255.0;
    if (t < 0.0)   t = 0.0;
    fprintf(stderr, "ROI mean=%.2f sigma=%.2f threshold=%.0f over %ld pixels\n",
            mean, sqrt(var), t, cnt);
    return (unsigned char)t;
}

static void apply_threshold(const unsigned char* gray, const unsigned char* roi,
                            unsigned char* mask, int n, unsigned char t)
{
    for (int i = 0; i < n; i++) {
        mask[i] = (roi[i] && gray[i] >= t) ? 255 : 0;
    }
}

/* ---------- region of interest (single polygon, point-in-polygon) ---------- */

typedef struct { int x, y; } Pt;

static int in_polygon(int x, int y, const Pt* poly, int np)
{
    int inside = 0;
    for (int i = 0, j = np - 1; i < np; j = i++) {
        if ((poly[i].y > y) != (poly[j].y > y)) {
            double xint = (double)(poly[j].x - poly[i].x) * (y - poly[i].y)
                          / (double)(poly[j].y - poly[i].y) + poly[i].x;
            if ((double)x < xint) inside = !inside;
        }
    }
    return inside;
}

static void build_roi(unsigned char* roi, int w, int h, const Pt* poly, int np)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            roi[y * w + x] = in_polygon(x, y, poly, np) ? 255 : 0;
        }
    }
}

/* ---------- connected components: iterative BFS, 8-connectivity ---------- */

static int count_components(unsigned char* mask, int w, int h, int min_size, int max_size)
{
    int n = w * h;
    unsigned char* visited = (unsigned char*)calloc((size_t)n, 1);
    int*           queue   = (int*)malloc(sizeof(int) * (size_t)n);
    if (!visited || !queue) {
        fprintf(stderr, "Out of memory in count_components\n");
        free(visited); free(queue);
        return -1;
    }
    static const int dx8[8] = { -1,  0,  1, -1,  1, -1,  0,  1 };
    static const int dy8[8] = { -1, -1, -1,  0,  0,  1,  1,  1 };

    int count = 0;
    for (int seed = 0; seed < n; seed++) {
        if (visited[seed] || !mask[seed]) continue;
        int qh = 0, qt = 0;
        queue[qt++] = seed;
        visited[seed] = 1;
        while (qh < qt) {
            int p  = queue[qh++];
            int px = p % w;
            int py = p / w;
            for (int d = 0; d < 8; d++) {
                int nx = px + dx8[d];
                int ny = py + dy8[d];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int neighbour = ny * w + nx;
                if (!visited[neighbour] && mask[neighbour]) {
                    visited[neighbour] = 1;
                    queue[qt++] = neighbour;
                }
            }
        }
        int sz = qt;
        if (sz >= min_size && sz <= max_size) {
            count++;
        } else {
            for (int k = 0; k < sz; k++) mask[queue[k]] = 0;
        }
    }
    free(queue);
    free(visited);
    return count;
}

/* ---------- main ---------- */

int main(int argc, char** argv)
{
    const char* in_name  = (argc > 1) ? argv[1] : "tankers.png";
    int    min_size      = (argc > 2) ? atoi(argv[2]) : 4;
    int    max_size      = (argc > 3) ? atoi(argv[3]) : 800;
    double k_sigma       = (argc > 4) ? atof(argv[4]) : 3.5;

    unsigned int W, H;
    unsigned char* rgba = load_png(in_name, &W, &H);
    if (!rgba) {
        fprintf(stderr, "Could not open %s\n", in_name);
        return 1;
    }
    int N = (int)(W * H);
    fprintf(stderr, "Loaded %s: %ux%u (%d pixels)\n", in_name, W, H, N);

    unsigned char* gray    = (unsigned char*)malloc((size_t)N);
    unsigned char* blurred = (unsigned char*)malloc((size_t)N);
    unsigned char* mask    = (unsigned char*)malloc((size_t)N);
    unsigned char* roi     = (unsigned char*)malloc((size_t)N);
    if (!gray || !blurred || !mask || !roi) {
        fprintf(stderr, "Out of memory\n");
        return 2;
    }

    to_grayscale(rgba, gray, N);
    gaussian_5x5(gray, blurred, (int)W, (int)H);

    /* Strait of Hormuz polygon for the upscaled (4x) image (5120x2588).
       Vertices traverse the dark-water area clockwise, stair-stepping
       around the curving coastline. Adjust if your image size or
       framing differs. */
    static const Pt poly[] = {
        { 2153,   27 }, { 2887,   27 }, { 2887, 1189 }, { 3733, 1189 },
        { 3733, 1273 }, { 4521, 1273 }, { 4521, 1395 }, { 4535, 1395 },
        { 4535, 1611 }, { 4581, 1611 }, { 4581, 2239 }, { 4681, 2239 },
        { 4681, 2415 }, { 4705, 2415 }, { 4705, 2587 }, { 2393, 2587 },
        { 2393, 2415 }, { 2261, 2415 }, { 2261, 2239 }, { 2127, 2239 },
        { 2127, 1611 }, { 2033, 1611 }, { 2033, 1395 }, { 1969, 1395 },
        { 1969, 1373 }, { 2199, 1373 }, { 2199, 1273 }, { 2025, 1273 },
        { 2025, 1189 }, { 2153, 1189 }
    };
    int npoly = (int)(sizeof(poly) / sizeof(poly[0]));
    build_roi(roi, (int)W, (int)H, poly, npoly);

    unsigned char t = compute_threshold(blurred, roi, N, k_sigma);
    apply_threshold(blurred, roi, mask, N, t);

    int n_tankers = count_components(mask, (int)W, (int)H, min_size, max_size);
    if (n_tankers < 0) {
        free(gray); free(blurred); free(mask); free(rgba);
        return 3;
    }
    printf("Tankers found: %d\n", n_tankers);

    /* Diagnostic outputs. */
    unsigned char* viz = (unsigned char*)malloc((size_t)N * 4);
    if (viz) {
        for (int i = 0; i < N; i++) {
            viz[4 * i + 0] = blurred[i];
            viz[4 * i + 1] = blurred[i];
            viz[4 * i + 2] = blurred[i];
            viz[4 * i + 3] = 255;
        }
        save_png("01_grayscale.png", viz, W, H);

        for (int i = 0; i < N; i++) {
            viz[4 * i + 0] = mask[i];
            viz[4 * i + 1] = mask[i];
            viz[4 * i + 2] = mask[i];
            viz[4 * i + 3] = 255;
        }
        save_png("02_mask.png", viz, W, H);

        memcpy(viz, rgba, (size_t)N * 4);
        for (int i = 0; i < N; i++) {
            if (mask[i]) {
                viz[4 * i + 0] = 255;
                viz[4 * i + 1] =  60;
                viz[4 * i + 2] =  60;
                viz[4 * i + 3] = 255;
            }
        }
        save_png("03_detections.png", viz, W, H);
        free(viz);
    }

    free(roi);
    free(mask);
    free(blurred);
    free(gray);
    free(rgba);
    return 0;
}
