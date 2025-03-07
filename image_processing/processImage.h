#include <stdint.h>

// Function prototypes
void detect_edges(uint8_t *image, uint8_t *edges, int width, int height);
int count_blobs(uint8_t *edges, int width, int height);
float measure_blurriness(uint8_t *image, int width, int height);
float estimate_rain_intensity(int edge_count, int blob_count, float blur_level);