#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "processImage.h"


// Define RAW image file path
#define IMAGE_PATH "rain1.raw"  // Change to your RAW file
#define IMAGE_WIDTH  640  // Must match the RAW image resolution
#define IMAGE_HEIGHT 480 // Must match the RAW image resolution

// Function prototype
uint8_t* load_raw_image(const char *filename, int width, int height);

int main() {
    // Step 1: Load RAW image into a grayscale buffer
    uint8_t *grayscale_image = load_raw_image(IMAGE_PATH, IMAGE_WIDTH, IMAGE_HEIGHT);
    if (!grayscale_image) {
        printf("Failed to load RAW image.\n");
        return 1;
    }

    printf("Loaded RAW Image: %dx%d\n", IMAGE_WIDTH, IMAGE_HEIGHT);

    // Step 2: Create a mock camera_fb_t struct
    camera_fb_t fb;
    fb.buf = grayscale_image;  // Point to raw pixel data
    fb.len = IMAGE_WIDTH * IMAGE_HEIGHT;
    fb.width = IMAGE_WIDTH;
    fb.height = IMAGE_HEIGHT;

    // Step 3: Process the image
    process_image(&fb);

    // Step 4: Cleanup
    free(grayscale_image);
    return 0;
}

// Function to load a RAW grayscale image from disk
uint8_t* load_raw_image(const char *filename, int width, int height) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Cannot open %s\n", filename);
        return NULL;
    }

    // Allocate memory for raw image buffer
    uint8_t *buffer = (uint8_t*)malloc(width * height);
    if (!buffer) {
        printf("Error: Could not allocate memory for image\n");
        fclose(file);
        return NULL;
    }

    // Read pixel data directly
    size_t bytesRead = fread(buffer, 1, width * height, file);
    fclose(file);

    if (bytesRead != width * height) {
        printf("Warning: Incomplete image data read.\n");
    }

    return buffer;
}
