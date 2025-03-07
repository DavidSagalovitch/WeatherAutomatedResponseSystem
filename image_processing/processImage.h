#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t *buf;     // Pointer to the image buffer
    size_t len;       // Length of the buffer in bytes
    size_t width;     // Image width in pixels
    size_t height;    // Image height in pixels
} camera_fb_t;


// Function prototypes
void process_image(camera_fb_t *fb);
void detect_edges(uint8_t *image, uint8_t *edges, int width, int height);
int count_blobs(uint8_t *edges, int width, int height);
float measure_blurriness(uint8_t *image, int width, int height);
float estimate_rain_intensity(int edge_count, int blob_count, float blur_level);