#ifndef CAMERA_SETUP_H
#define CAMERA_SETUP_H

void setupCamera(void);
void captureImage(void);

typedef struct {
    uint8_t *buf;     // Pointer to the image buffer
    size_t len;       // Length of the buffer in bytes
    size_t width;     // Image width in pixels
    size_t height;    // Image height in pixels
    uint8_t bad;      // Image quality indicator (not commonly used)
} camera_fb_t;

#endif // CAMERA_SETUP_H
