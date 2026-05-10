#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lodepng.h"

unsigned char* load_png(const char* filename, unsigned int* width, unsigned int* height)
{
    unsigned char* image = NULL;
    int error = lodepng_decode32_file(&image, width, height, filename);
    if (error != 0) {
        printf("error %u: %s\n", error, lodepng_error_text(error));
    }
    return image;
}

void to_gray(unsigned char *picture, unsigned char *gray, int size)
{
    for (int i = 0; i < size; i++) {
        int sum = picture[i * 4] + picture[i * 4 + 1] + picture[i * 4 + 2];
        gray[i] = sum / 3;
    }
}

void threshold(unsigned char *gray, int size, int min_value)
{
    for (int i = 0; i < size; i++) {
        if (gray[i] < min_value) {
            gray[i] = 0;
        } else {
            gray[i] = 255;
        }
    }
}

void fill_strait(unsigned char *gray, unsigned char *roi, int width)
{
    int rects[8][4] = {
        {2153,   27, 2887, 1189},
        {2025, 1189, 3733, 1273},
        {1969, 1273, 4521, 1395},
        {2033, 1395, 4535, 1611},
        {2127, 1611, 4581, 2239},
        {2261, 2239, 4681, 2415},
        {2393, 2415, 4705, 2587},
        {1149,   39, 1423,  435}
    };
    for (int r = 0; r < 8; r++) {
        int x1 = rects[r][0];
        int y1 = rects[r][1];
        int x2 = rects[r][2];
        int y2 = rects[r][3];
        for (int y = y1; y < y2; y++) {
            for (int x = x1; x < x2; x++) {
                if (gray[y * width + x] != 0) {
                    roi[y * width + x] = 255;
                }
            }
        }
    }
}

void dfs(unsigned char *roi, unsigned char *visited, int i, int size, int width)
{
    if (i < 0 || i >= size) return;
    if (visited[i] || roi[i] == 0) return;
    visited[i] = 1;
    dfs(roi, visited, i + 1,        size, width);
    dfs(roi, visited, i - 1,        size, width);
    dfs(roi, visited, i + width,    size, width);
    dfs(roi, visited, i - width,    size, width);
}

int main(void)
{
    const char *filename = "tankers.png";
    unsigned int width, height;
    unsigned char *picture = load_png(filename, &width, &height);
    if (picture == NULL) {
        printf("Cannot read file %s\n", filename);
        return -1;
    }

    int size = width * height;
    unsigned char *gray    = (unsigned char*)calloc(size, sizeof(unsigned char));
    unsigned char *roi     = (unsigned char*)calloc(size, sizeof(unsigned char));
    unsigned char *visited = (unsigned char*)calloc(size, sizeof(unsigned char));

    to_gray(picture, gray, size);
    threshold(gray, size, 90);
    fill_strait(gray, roi, width);

    int counter = 0;
    for (int i = 0; i < size; i++) {
        if (!visited[i] && roi[i] != 0) {
            dfs(roi, visited, i, size, width);
            counter++;
        }
    }
    printf("Tankers found: %d\n", counter);

    free(gray);
    free(roi);
    free(visited);
    free(picture);
    return 0;
}
