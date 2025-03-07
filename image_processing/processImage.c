#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "processImage.h"

// Image Processing Parameters
#define WIDTH  640  // Image width (ESP32-CAM default)
#define HEIGHT 480  // Image height
#define EDGE_THRESHOLD 50  // Threshold for edge detection
#define BLOB_MIN_SIZE 10   // Minimum blob size to be considered a raindrop

void process_image(camera_fb_t *fb) {
    uint8_t *grayscale_image = fb->buf;  // Use grayscale image
    uint8_t edges[WIDTH * HEIGHT] = {0};  // Store detected edges
    printf("Starting image processing \n");

    // Step 1: Edge Detection (Calculate edge_count)
    detect_edges(grayscale_image, edges, WIDTH, HEIGHT);
    int edge_count = 0;
    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        if (edges[i] > 0) {
            edge_count++;  // Count the number of edge pixels
        }
    }

    // Step 2: Blob Detection (Count Raindrop Blobs)
    int blob_count = count_blobs(edges, WIDTH, HEIGHT);

    // Step 3: Blurriness Detection
    float blur_level = measure_blurriness(grayscale_image, WIDTH, HEIGHT);

    // Step 4: Estimate Rain Intensity
    float rain_intensity = estimate_rain_intensity(edge_count, blob_count, blur_level);

    // Print the result
    printf("Edges: %d, Blobs: %d, Blur Level: %.2f, Estimated Rain Intensity: %.2f\n",
           edge_count, blob_count, blur_level, rain_intensity);
}


// Edge detection using Sobel filter
void detect_edges(uint8_t *image, uint8_t *edges, int width, int height) {
    int gx, gy, magnitude;
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;
            
            // Sobel operator for edge detection
            gx = -image[idx - width - 1] + image[idx - width + 1] 
                 -2 * image[idx - 1] + 2 * image[idx + 1] 
                 -image[idx + width - 1] + image[idx + width + 1];

            gy = -image[idx - width - 1] - 2 * image[idx - width] - image[idx - width + 1] 
                 +image[idx + width - 1] + 2 * image[idx + width] + image[idx + width + 1];

            magnitude = sqrt(gx * gx + gy * gy);
            edges[idx] = (magnitude > EDGE_THRESHOLD) ? 255 : 0;
        }
    }
}

// Count blobs using a simple connected components algorithm
int count_blobs(uint8_t *edges, int width, int height) {
    int blob_count = 0;
    uint8_t visited[WIDTH * HEIGHT] = {0};

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;

            if (edges[idx] == 255 && !visited[idx]) {
                // New blob found
                blob_count++;

                // Flood fill algorithm (iterative)
                int stack[WIDTH * HEIGHT];
                int stack_size = 0;
                stack[stack_size++] = idx;

                int blob_size = 0;
                while (stack_size > 0) {
                    int current = stack[--stack_size];
                    if (visited[current]) continue;

                    visited[current] = 1;
                    blob_size++;

                    // Check 8 neighbors
                    int neighbors[] = { -1, 1, -width, width, -width-1, -width+1, width-1, width+1 };
                    for (int i = 0; i < 8; i++) {
                        int neighbor_idx = current + neighbors[i];
                        if (edges[neighbor_idx] == 255 && !visited[neighbor_idx]) {
                            stack[stack_size++] = neighbor_idx;
                        }
                    }
                }

                // Ignore small blobs (noise)
                if (blob_size < BLOB_MIN_SIZE) blob_count--;
            }
        }
    }

    return blob_count;
}

// Blurriness detection using variance of Laplacian filter
float measure_blurriness(uint8_t *image, int width, int height) {
    int sum = 0, sum_sq = 0, count = 0;

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;

            // Laplacian kernel for blur detection
            int laplacian = 4 * image[idx] 
                            - image[idx - 1] - image[idx + 1] 
                            - image[idx - width] - image[idx + width];

            sum += laplacian;
            sum_sq += laplacian * laplacian;
            count++;
        }
    }

    // Variance of Laplacian is a common blur metric
    float mean = sum / (float)count;
    float variance = (sum_sq / (float)count) - (mean * mean);
    
    return variance;
}

// Compute rain intensity based on detected features
float estimate_rain_intensity(int edge_count, int blob_count, float blur_level) {
    // Normalize values (assuming max values)
    float edge_factor = edge_count / 1000.0;  // Normalize by max edge count
    float blob_factor = blob_count / 100.0;   // Normalize by max blob count
    float blur_factor = blur_level / 100.0;   // Normalize blur level

    // Weighted sum (weights can be adjusted)
    return (0.5 * edge_factor) + (0.3 * blob_factor) + (0.2 * blur_factor);
}
