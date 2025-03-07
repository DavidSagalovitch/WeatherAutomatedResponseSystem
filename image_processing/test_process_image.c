#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "processImage.h"


// Define RAW image file path
#define IMAGE_PATH "clearday.raw"  // Change to your RAW file
#define IMAGE_WIDTH  640  // Must match the RAW image resolution
#define IMAGE_HEIGHT 480 // Must match the RAW image resolution

// Function prototypes
uint8_t* load_raw_image(const char *filename, int width, int height);
void run_multiple_tests(const char *filenames[], int num_files);

int main() {
    // List of test images
    const char *test_images[] = {
        "clearday.raw",
        "clearnight.raw",
        "rain1.raw",
    };

    int num_files = sizeof(test_images) / sizeof(test_images[0]);

    // Run all test images
    run_multiple_tests(test_images, num_files);

    return 0;
}

// Function to process multiple RAW images
void run_multiple_tests(const char *filenames[], int num_files) {
    for (int i = 0; i < num_files; i++) {
        printf("\n====================\n");
        printf("Processing Image: %s\n", filenames[i]);
        printf("====================\n");

        // Load RAW image
        uint8_t *grayscale_image = load_raw_image(filenames[i], IMAGE_WIDTH, IMAGE_HEIGHT);
        if (!grayscale_image) {
            printf("Failed to load image: %s\n", filenames[i]);
            continue;  // Skip to next image
        }

        // Create mock camera framebuffer struct
        camera_fb_t fb;
        fb.buf = grayscale_image;
        fb.len = IMAGE_WIDTH * IMAGE_HEIGHT;
        fb.width = IMAGE_WIDTH;
        fb.height = IMAGE_HEIGHT;

        // Process the image
        process_image(&fb);

        // Free allocated memory
        free(grayscale_image);
    }
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
