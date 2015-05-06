// Entropy patches code

#include "entropy_patches.h"

#define RESOLUTION 250

static const int8_t log2_lookup_table[250] = {0, 8, 14, 19, 24, 28, 32, 36, 40, 43, 46, 50, 53, 55, 58, 61, 63, 66, 68, 71, 73, 75, 77, 79, 81, 83, 85, 87, 88, 90, 92, 93, 95, 96, 98, 99, 101, 102, 103, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 119, 120, 121, 122, 122, 123, 124, 124, 125, 125, 126, 126, 127, 127, 128, 128, 129, 129, 129, 130, 130, 130, 131, 131, 131, 131, 132, 132, 132, 132, 132, 132, 132, 132, 133, 133, 133, 133, 133, 133, 133, 133, 133, 132, 132, 132, 132, 132, 132, 132, 132, 131, 131, 131, 131, 131, 130, 130, 130, 129, 129, 129, 129, 128, 128, 127, 127, 127, 126, 126, 125, 125, 125, 124, 124, 123, 123, 122, 122, 121, 121, 120, 119, 119, 118, 118, 117, 116, 116, 115, 115, 114, 113, 113, 112, 111, 111, 110, 109, 108, 108, 107, 106, 105, 105, 104, 103, 102, 101, 101, 100, 99, 98, 97, 96, 95, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 64, 63, 62, 61, 60, 59, 58, 56, 55, 54, 53, 52, 50, 49, 48, 47, 46, 44, 43, 42, 41, 39, 38, 37, 35, 34, 33, 32, 30, 29, 28, 26, 25, 24, 22, 21, 20, 18, 17, 16, 14, 13, 11, 10, 9, 7, 6, 4, 3, 1};

/* Entropy patches:
 * takes input image in, which contains both the left and the right image
 * determines the entropy of patches in one of the two images.
 *
 * parameters:
 * right: whether to use the right image (1) or left image (0)
 * in: the input image(s)
 * image_width
 * image_height
 * min_x/y, max_x/y: defines a rectangle in which to take samples on a grid
 * patch_size: patch size in pixels of which to determine entropy
 * step_size: in pixels. The bigger the step size, the fewer samples
 * n_bins: number of bins in the histogram / pdf for entropy determination
 * */

uint32_t get_entropy_patches(uint8_t right, uint8_t *in, uint32_t image_width, uint32_t image_height, uint8_t min_x,
                             uint8_t max_x, uint8_t min_y, uint8_t max_y, uint8_t patch_size, uint8_t step_size, uint8_t n_bins)
{
  uint32_t total_entropy = 0;
  uint32_t n_samples = 0;
  uint32_t n_elements = patch_size * patch_size;
  // initialize probability distribution P
  q15_t P[n_bins];
  q7_t bin_size = 255 / n_bins;

  // loop over the selected image part with step_size
  uint32_t W2 = image_width * 2; // number of pixels per image line in stereo image
  uint32_t x, y, xx, yy;
  uint8_t px, bin;
  uint32_t Mx = max_x - patch_size;
  uint32_t My = max_y - patch_size;

  for (x = min_x; x < Mx; x += step_size) {
    for (y = min_y; y < My; y += step_size) {
      // set probability distribution to zero:
      arm_fill_q15(0, P, n_bins);

      // loop over the pixels in the patch:
      for (xx = x; xx < x + patch_size; xx++) {
        for (yy = y; yy < y + patch_size; yy++) {
          px = in[yy * W2 + xx * 2 + right];
          bin = px / bin_size;
          P[bin]++;
        }
      }

      // get entropy for this patch:
      total_entropy += calculate_entropy(P, n_bins, n_elements);
      n_samples++;
    }
  }

  return (uint32_t) total_entropy / n_samples;
}

uint32_t calculate_entropy(q15_t *P, uint8_t n_bins, uint32_t n_elements)
{
  uint32_t entropy = 0;
  uint32_t table_index;
  uint8_t bin;
  for (bin = 0; bin < n_bins; bin++) {
    table_index = (P[bin] * RESOLUTION) / n_elements;
    entropy += log2_lookup_table[table_index];
  }
  return entropy;
}

