#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "processImage.h"

// Image Processing Parameters
#define WIDTH  640  // Image width (ESP32-CAM default)
#define HEIGHT 480  // Image height
#define EDGE_THRESHOLD 50  // Threshold for edge detection
#define BLOB_MIN_SIZE 10   // Minimum blob size to be considered a raindrop

void process_image(camera_fb_t *fb) {
    uint8_t *grayscale_image = fb->buf;  // Use grayscale image
    uint8_t edges[WIDTH * HEIGHT] = {0};  // Store detected edges

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

    int brightness = detect_brightness(grayscale_image, WIDTH, HEIGHT);

    // Step 4: Estimate Rain Intensity
    float rain_intensity = estimate_rain_intensity(edge_count, blob_count, blur_level, brightness);

    // Print the result
    printf("Brightness: %d, Edges: %d, Blobs: %d, Blur Level: %.2f, Estimated Rain Intensity: %.2f\n",
        brightness, edge_count, blob_count, blur_level, rain_intensity);
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

float measure_blurriness(uint8_t *image, int width, int height) {
    int64_t sum = 0;
    int64_t sum_sq = 0;  // Use int64_t to prevent overflow
    int count = 0;

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int idx = y * width + x;

            // Laplacian kernel for blur detection
            int laplacian = 4 * image[idx] 
                            - image[idx - 1] - image[idx + 1] 
                            - image[idx - width] - image[idx + width];

            sum += laplacian;
            sum_sq += (int64_t)laplacian * laplacian;  // Prevent overflow
            count++;
        }
    }

    // Prevent division by zero
    if (count == 0) return 0;

    // Convert to double for higher precision
    double mean = sum / (double)count;
    double variance = (sum_sq / (double)count) - (mean * mean);

    // If variance is negative due to precision errors, take absolute value
    return (float)fabs(variance);
}

// Function to determine if the scene is day or night
int detect_brightness(uint8_t *image, int width, int height) {
    int edge_count = 0;
    int brightness_sum = 0;
    
    // Loop through the image
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            brightness_sum += image[idx];  // Sum pixel brightness (0-255)
        }
    }

    // Calculate average brightness
    float avg_brightness = brightness_sum / (float)(width * height);

    return avg_brightness;
}

float estimate_rain_intensity(int edge_count, int blob_count, float blur_level, int brightness) {
    // Normalize features
    float blob_factor = log(1 + blob_count) / log(1000.0);
    float blur_factor = log(1 + fabs(blur_level)) / log(50000.0);

    // Normalize brightness to range [0,1] (assuming 0-255 scale)
    float brightness_weight = brightness / 255.0;  

    // Use brightness as a smooth weighting factor
    float blob_weight = brightness_weight;          // More brightness → blobs matter more
    float blur_weight = 1.0 - brightness_weight;    // Less brightness → blur matters more

    // Compute intensity
    float intensity = (blob_weight * blob_count/100) + (blur_weight * blur_factor/1000);

    // Scale to 0-100 range
    intensity *= 50;

    // Ensure within bounds
    if (intensity > 100) intensity = 100;
    if (intensity < 0) intensity = 0;

    return intensity;
}
