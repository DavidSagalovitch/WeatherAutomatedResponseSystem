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

typedef struct {
    int x;
    int y;
} Point;

int count_blobs(uint8_t *edges, int width, int height) {
    int blob_count = 0;

    // Allocate visited map dynamically
    uint8_t *visited = (uint8_t*)calloc(width * height, sizeof(uint8_t));
    if (!visited) {
        printf("Memory allocation failed for visited[].\n");
        return 0;
    }

    // Allocate initial queue size
    int queue_capacity = 50000;  // Start small
    Point *queue = (Point*)malloc(queue_capacity * sizeof(Point));
    if (!queue) {
        printf("Memory allocation failed for queue[].\n");
        free(visited);
        return 0;
    }

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;

            if (edges[idx] == 255 && !visited[idx]) {
                // New blob found
                blob_count++;

                int queue_start = 0, queue_end = 0;
                queue[queue_end++] = (Point){x, y};
                int blob_size = 0;

                while (queue_start < queue_end) {
                    if (queue_end >= queue_capacity - 10) { 
                        // 🔹 Dynamically resize queue if needed
                        queue_capacity *= 2;
                        queue = (Point*)realloc(queue, queue_capacity * sizeof(Point));
                        if (!queue) {
                            printf("Warning: Memory limit reached, stopping further processing.\n");
                            break;  // Avoid freezing
                        }
                    }

                    Point p = queue[queue_start++];
                    int current_idx = p.y * width + p.x;

                    if (visited[current_idx]) continue;
                    visited[current_idx] = 1;
                    blob_size++;

                    // 8-Neighborhood Connectivity
                    int dx[] = {-1, 1,  0,  0, -1, -1,  1,  1};
                    int dy[] = { 0, 0, -1,  1, -1,  1, -1,  1};

                    for (int i = 0; i < 8; i++) {
                        int nx = p.x + dx[i];
                        int ny = p.y + dy[i];
                        int neighbor_idx = ny * width + nx;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (edges[neighbor_idx] == 255 && !visited[neighbor_idx]) {
                                queue[queue_end++] = (Point){nx, ny};
                            }
                        }
                    }
                }

                // Ignore small blobs (noise)
                if (blob_size < BLOB_MIN_SIZE) blob_count--;
            }
        }
    }

    // Free allocated memory
    free(visited);
    free(queue);

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
