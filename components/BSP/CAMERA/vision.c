#include "vision.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "tal_api.h"

#define VISION_GAUSSIAN_RADIUS 2
#define VISION_CANNY_LOW 50
#define VISION_CANNY_HIGH 150
#define VISION_MIN_AREA 500.0f
#define VISION_APPROX_EPSILON_PERCENT 1.0f
#define VISION_BORDER_MARGIN 5
#define VISION_MIN_ANGLE_DEG 70.0f
#define VISION_MAX_ANGLE_DEG 110.0f
#define VISION_MAX_EDGE_RATIO 5.0f
#define VISION_TARGET_WIDTH_MM 265.0f
#define VISION_TARGET_HEIGHT_MM 203.0f
#define VISION_CIRCLE_RADIUS_MM 60.0f
#define VISION_MAX_POLYGON_POINTS 48
#define VISION_MAX_QUAD_VERTICES 8
#define VISION_PI 3.14159265358979323846f

static uint16_t sg_width;
static uint16_t sg_height;
static uint32_t sg_pixels;
static VISION_PARAMS_T sg_params = {
    -1,
    VISION_CANNY_LOW,
    VISION_CANNY_HIGH,
    (uint32_t)VISION_MIN_AREA,
    2,
    1,
    (uint16_t)(VISION_APPROX_EPSILON_PERCENT * 100.0f),
    1,
    1,
    180,
    180,
    255,
    255,
    0,
    255,
    0,
};
#if VISION_ENABLE_BINARIZATION
static uint8_t *sg_gray;
static uint16_t *sg_blur;
static uint8_t *sg_binary;
#endif
#if VISION_ENABLE_CANNY
static int16_t *sg_gx;
static int16_t *sg_gy;
static uint16_t *sg_magnitude;
static uint8_t *sg_edges;
#endif
#if VISION_ENABLE_CANNY || VISION_ENABLE_CONTOUR
static uint32_t *sg_stack;
#endif
#if VISION_ENABLE_CONTOUR
static uint8_t *sg_visited;
static uint8_t *sg_approx_keep;
static VISION_POINT_T *sg_contour;
#endif
static bool sg_ready;
#if VISION_ENABLE_TARGET_SELECTION
static VISION_RESULT_T sg_cached_result;
static uint32_t sg_detect_frame;
static bool sg_cache_valid;
#endif

static void *vision_alloc(uint32_t size)
{
    void *ptr = tal_psram_malloc(size);

    if (NULL == ptr) {
        ptr = tal_malloc(size);
    }

    return ptr;
}

#if VISION_ENABLE_TARGET_SELECTION
static int32_t vision_round(float value)
{
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}
#endif

#if VISION_ENABLE_CANNY
static int32_t vision_abs(int32_t value)
{
    return value < 0 ? -value : value;
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static float vision_distance(VISION_POINT_T a, VISION_POINT_T b)
{
    float dx = (float)a.x - b.x;
    float dy = (float)a.y - b.y;

    return sqrtf(dx * dx + dy * dy);
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static bool vision_order_corners(const VISION_POINT_T input[4], VISION_POINT_T output[4])
{
    int min_sum = 0;
    int max_sum = 0;
    int min_diff = 0;
    int max_diff = 0;

    for (int i = 1; i < 4; i++) {
        int sum = input[i].x + input[i].y;
        int diff = input[i].y - input[i].x;
        int min_sum_value = input[min_sum].x + input[min_sum].y;
        int max_sum_value = input[max_sum].x + input[max_sum].y;
        int min_diff_value = input[min_diff].y - input[min_diff].x;
        int max_diff_value = input[max_diff].y - input[max_diff].x;

        if (sum < min_sum_value) {
            min_sum = i;
        }
        if (sum > max_sum_value) {
            max_sum = i;
        }
        if (diff < min_diff_value) {
            min_diff = i;
        }
        if (diff > max_diff_value) {
            max_diff = i;
        }
    }

    if ((min_sum == max_sum) || (min_sum == min_diff) || (min_sum == max_diff)
        || (max_sum == min_diff) || (max_sum == max_diff) || (min_diff == max_diff)) {
        return false;
    }

    output[0] = input[min_sum];
    output[1] = input[min_diff];
    output[2] = input[max_sum];
    output[3] = input[max_diff];

    return true;
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static float vision_polygon_area(const VISION_POINT_T *points, uint32_t count)
{
    int64_t twice_area = 0;

    for (uint32_t i = 0; i < count; i++) {
        uint32_t next = (i + 1) % count;
        twice_area += (int64_t)points[i].x * points[next].y
                      - (int64_t)points[next].x * points[i].y;
    }

    return fabsf((float)twice_area * 0.5f);
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static float vision_polygon_perimeter(const VISION_POINT_T *points, uint32_t count, bool closed)
{
    float perimeter = 0.0f;

    for (uint32_t i = 1; i < count; i++) {
        perimeter += vision_distance(points[i - 1], points[i]);
    }
    if (closed && (count > 2)) {
        perimeter += vision_distance(points[count - 1], points[0]);
    }

    return perimeter;
}
#endif

#if VISION_ENABLE_BINARIZATION
static void vision_gaussian_blur(void)
{
    static const uint8_t kernel[5] = {1, 4, 6, 4, 1};

    for (uint16_t y = 0; y < sg_height; y++) {
        uint32_t row = (uint32_t)y * sg_width;

        for (uint16_t x = 0; x < sg_width; x++) {
            uint32_t sum = 0;

            for (int k = -VISION_GAUSSIAN_RADIUS; k <= VISION_GAUSSIAN_RADIUS; k++) {
                int sample = (int)x + k;
                sample = sample < 0 ? 0 : (sample >= sg_width ? sg_width - 1 : sample);
                sum += (uint32_t)kernel[k + 2] * sg_gray[row + sample];
            }
            sg_blur[row + x] = (uint16_t)sum;
        }
    }

    for (uint16_t y = 0; y < sg_height; y++) {
        uint32_t row = (uint32_t)y * sg_width;

        for (uint16_t x = 0; x < sg_width; x++) {
            uint32_t sum = 0;

            for (int k = -VISION_GAUSSIAN_RADIUS; k <= VISION_GAUSSIAN_RADIUS; k++) {
                int sample = (int)y + k;
                sample = sample < 0 ? 0 : (sample >= sg_height ? sg_height - 1 : sample);
                sum += (uint32_t)kernel[k + 2] * sg_blur[(uint32_t)sample * sg_width + x];
            }
            sg_gray[row + x] = (uint8_t)((sum + 128) >> 8);
        }
    }
}

static uint8_t vision_otsu_threshold(void)
{
    uint32_t histogram[256] = {0};
    uint64_t sum = 0;
    uint64_t background_sum = 0;
    uint32_t background_weight = 0;
    double best_variance = 0.0;
    uint8_t threshold = 127;

    for (uint32_t i = 0; i < sg_pixels; i++) {
        histogram[sg_gray[i]]++;
        sum += sg_gray[i];
    }

    for (uint32_t value = 0; value < 256; value++) {
        uint64_t foreground_weight;
        double background_mean;
        double foreground_mean;
        double mean_delta;
        double variance;

        background_weight += histogram[value];
        background_sum += (uint64_t)value * histogram[value];
        foreground_weight = sg_pixels - background_weight;
        if ((0 == background_weight) || (0 == foreground_weight)) {
            continue;
        }

        background_mean = (double)background_sum / background_weight;
        foreground_mean = (double)(sum - background_sum) / foreground_weight;
        mean_delta = background_mean - foreground_mean;
        variance = (double)background_weight * foreground_weight * mean_delta * mean_delta;
        if (variance > best_variance) {
            best_variance = variance;
            threshold = (uint8_t)value;
        }
    }

    return threshold;
}

static void vision_morph_pass(uint8_t *dst, const uint8_t *src, bool max_filter)
{
    for (uint16_t y = 0; y < sg_height; y++) {
        for (uint16_t x = 0; x < sg_width; x++) {
            uint8_t value = max_filter ? 0 : 255;

            for (int dy = -1; dy <= 1; dy++) {
                int sy = (int)y + dy;
                sy = sy < 0 ? 0 : (sy >= sg_height ? sg_height - 1 : sy);
                for (int dx = -1; dx <= 1; dx++) {
                    int sx = (int)x + dx;
                    sx = sx < 0 ? 0 : (sx >= sg_width ? sg_width - 1 : sx);
                    uint8_t sample = src[(uint32_t)sy * sg_width + sx];

                    if (max_filter) {
                        value = sample > value ? sample : value;
                    } else {
                        value = sample < value ? sample : value;
                    }
                }
            }
            dst[(uint32_t)y * sg_width + x] = value;
        }
    }
}

static void vision_morphology(void)
{
    uint8_t *scratch = (uint8_t *)sg_blur;

    vision_morph_pass(scratch, sg_binary, false);
    vision_morph_pass(sg_binary, scratch, true);
    vision_morph_pass(scratch, sg_binary, true);
    vision_morph_pass(sg_binary, scratch, false);
}

static uint8_t vision_binarize(const uint8_t *yuyv, uint32_t data_len, VISION_RESULT_T *result)
{
    for (uint16_t y = 0; y < sg_height; y++) {
        for (uint16_t x = 0; x < sg_width; x++) {
            uint32_t index = (uint32_t)y * sg_width + x;
            uint32_t offset = index * 2;

            sg_gray[index] = offset < data_len ? yuyv[offset] : 0;
        }
    }

    if (sg_params.cmode) {
        for (uint16_t y = 0; y < sg_height; y++) {
            uint16_t cy = (uint16_t)((y & ~3u) + sg_params.cphase);
            uint32_t crow = (uint32_t)cy * sg_width;

            for (uint16_t x = 0; x < sg_width; x++) {
                uint32_t i = (uint32_t)y * sg_width + x;
                uint32_t base = (crow + x) / 2 * 4;
                uint8_t yy = yuyv[i * 2];
                uint8_t uu = (base + 1) < data_len ? yuyv[base + 1] : 128;
                uint8_t vv = (base + 3) < data_len ? yuyv[base + 3] : 128;
                bool paper = (uu >= sg_params.u_lo) && (uu <= sg_params.u_hi)
                             && (vv >= sg_params.v_lo) && (vv <= sg_params.v_hi)
                             && (yy >= sg_params.y_lo) && (yy <= sg_params.y_hi);

                sg_binary[i] = paper ? 255 : 0;
            }
        }
        if (sg_params.morph) {
            vision_morphology();
        }
        return 0;
    }

    vision_gaussian_blur();

    uint8_t threshold = (sg_params.threshold >= 0) ? (uint8_t)sg_params.threshold : vision_otsu_threshold();
    for (uint32_t i = 0; i < sg_pixels; i++) {
        sg_binary[i] = sg_gray[i] > threshold ? 255 : 0;
    }

    if (sg_params.cmode == 2) {
        uint32_t kept = 0;
        int64_t sum_x = 0, sum_y = 0;

        for (uint16_t y = 0; y < sg_height; y++) {
            uint16_t cy = (uint16_t)((y & ~3u) + sg_params.cphase);
            uint32_t crow = (uint32_t)cy * sg_width;

            for (uint16_t x = 0; x < sg_width; x++) {
                uint32_t i = (uint32_t)y * sg_width + x;
                uint32_t base = (crow + x) / 2 * 4;
                uint8_t uu = (base + 1) < data_len ? yuyv[base + 1] : 128;
                uint8_t vv = (base + 3) < data_len ? yuyv[base + 3] : 128;
                int16_t delta = (int16_t)uu - (int16_t)vv;
                bool dark = sg_gray[i] <= threshold;
                bool colored = (delta > 20) || (delta < -20);

                sg_binary[i] = (dark && colored) ? 255 : 0;
            }
        }
        memset(sg_visited, 0, sg_pixels);
        for (uint32_t start = 0; start < sg_pixels; start++) {
            uint32_t stack_size = 0;
            uint32_t count = 0;
            int16_t min_x, min_y, max_x, max_y;
            int64_t cx = 0, cy = 0;

            if ((0 == sg_binary[start]) || sg_visited[start]) {
                continue;
            }
            min_x = max_x = (int16_t)(start % sg_width);
            min_y = max_y = (int16_t)(start / sg_width);
            sg_stack[stack_size++] = start;
            sg_visited[start] = 1;
            while (stack_size > 0) {
                uint32_t index = sg_stack[--stack_size];
                int x = (int)(index % sg_width);
                int y = (int)(index / sg_width);

                count++;
                cx += x;
                cy += y;
                min_x = x < min_x ? (int16_t)x : min_x;
                min_y = y < min_y ? (int16_t)y : min_y;
                max_x = x > max_x ? (int16_t)x : max_x;
                max_y = y > max_y ? (int16_t)y : max_y;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        uint32_t neighbor;

                        if ((nx < 0) || (nx >= sg_width) || (ny < 0) || (ny >= sg_height)) {
                            continue;
                        }
                        neighbor = (uint32_t)ny * sg_width + nx;
                        if ((0 != sg_binary[neighbor]) && !sg_visited[neighbor]) {
                            sg_visited[neighbor] = 1;
                            sg_stack[stack_size++] = neighbor;
                        }
                    }
                }
            }
            if ((count >= 30) && ((max_x - min_x) >= 15) && ((max_y - min_y) >= 15)
                && ((max_x - min_x) <= 90) && ((max_y - min_y) <= 90)) {
                kept += count;
                sum_x += cx;
                sum_y += cy;
            }
        }
        if (kept >= 60) {
            result->target_found = true;
            result->center.x = (int16_t)(sum_x / kept);
            result->center.y = (int16_t)(sum_y / kept);
            result->area = kept;
        }
        return threshold;
    }

    if (sg_params.morph) {
        vision_morphology();
    }

    return threshold;
}
#endif

#if VISION_ENABLE_CANNY
static uint32_t vision_canny(void)
{
    uint32_t stack_size = 0;
    uint32_t edge_count = 0;

    memset(sg_magnitude, 0, sg_pixels * sizeof(*sg_magnitude));
    memset(sg_edges, 0, sg_pixels);

    for (uint16_t y = 1; y < sg_height - 1; y++) {
        for (uint16_t x = 1; x < sg_width - 1; x++) {
            uint32_t index = (uint32_t)y * sg_width + x;
            int a = sg_binary[index - sg_width - 1];
            int b = sg_binary[index - sg_width];
            int c = sg_binary[index - sg_width + 1];
            int d = sg_binary[index - 1];
            int f = sg_binary[index + 1];
            int g = sg_binary[index + sg_width - 1];
            int h = sg_binary[index + sg_width];
            int j = sg_binary[index + sg_width + 1];
            int gx = c + 2 * f + j - a - 2 * d - g;
            int gy = g + 2 * h + j - a - 2 * b - c;

            sg_gx[index] = (int16_t)gx;
            sg_gy[index] = (int16_t)gy;
            sg_magnitude[index] = (uint16_t)(vision_abs(gx) + vision_abs(gy));
        }
    }

    for (uint16_t y = 1; y < sg_height - 1; y++) {
        for (uint16_t x = 1; x < sg_width - 1; x++) {
            uint32_t index = (uint32_t)y * sg_width + x;
            uint16_t magnitude = sg_magnitude[index];
            uint16_t first;
            uint16_t second;
            int ax = vision_abs(sg_gx[index]);
            int ay = vision_abs(sg_gy[index]);

            if (0 == magnitude) {
                continue;
            }

            if (ax > 2 * ay) {
                first = sg_magnitude[index - 1];
                second = sg_magnitude[index + 1];
            } else if (ay > 2 * ax) {
                first = sg_magnitude[index - sg_width];
                second = sg_magnitude[index + sg_width];
            } else if ((sg_gx[index] > 0) == (sg_gy[index] > 0)) {
                first = sg_magnitude[index - sg_width + 1];
                second = sg_magnitude[index + sg_width - 1];
            } else {
                first = sg_magnitude[index - sg_width - 1];
                second = sg_magnitude[index + sg_width + 1];
            }

            if ((magnitude >= first) && (magnitude >= second)) {
                if (magnitude >= sg_params.canny_high) {
                    sg_edges[index] = 2;
                    sg_stack[stack_size++] = index;
                } else if (magnitude >= sg_params.canny_low) {
                    sg_edges[index] = 1;
                }
            }
        }
    }

    while (stack_size > 0) {
        uint32_t index = sg_stack[--stack_size];
        int x = (int)(index % sg_width);
        int y = (int)(index / sg_width);

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int nx = x + dx;
                int ny = y + dy;
                uint32_t neighbor;

                if ((nx < 0) || (nx >= sg_width) || (ny < 0) || (ny >= sg_height)) {
                    continue;
                }
                neighbor = (uint32_t)ny * sg_width + nx;
                if (1 == sg_edges[neighbor]) {
                    sg_edges[neighbor] = 2;
                    sg_stack[stack_size++] = neighbor;
                }
            }
        }
    }

    for (uint32_t i = 0; i < sg_pixels; i++) {
        if (2 == sg_edges[i]) {
            sg_edges[i] = 255;
            edge_count++;
        } else {
            sg_edges[i] = 0;
        }
    }

    return edge_count;
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static bool vision_border_pixel(int x, int y)
{
    if ((x < 0) || (x >= sg_width) || (y < 0) || (y >= sg_height)) {
        return false;
    }

    return sg_edges[(uint32_t)y * sg_width + x] != 0;
}

static int vision_neighbor_direction(VISION_POINT_T center, VISION_POINT_T neighbor)
{
    static const int8_t directions[8][2] = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };
    int dx = neighbor.x - center.x;
    int dy = neighbor.y - center.y;

    for (int i = 0; i < 8; i++) {
        if ((dx == directions[i][0]) && (dy == directions[i][1])) {
            return i;
        }
    }

    return -1;
}

static VISION_POINT_T vision_neighbor(VISION_POINT_T center, int direction)
{
    static const int8_t directions[8][2] = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    return (VISION_POINT_T){(int16_t)(center.x + directions[direction & 7][0]),
                            (int16_t)(center.y + directions[direction & 7][1])};
}

static uint32_t vision_trace_contour(VISION_POINT_T start, int direction)
{
    VISION_POINT_T current = start;
    VISION_POINT_T previous = {-1, -1};
    uint32_t count = 0;
    bool isolated = true;

    for (int step = 0; step < 8; step++) {
        VISION_POINT_T next = vision_neighbor(start, direction + step);
        if (vision_border_pixel(next.x, next.y)) {
            previous = vision_neighbor(next, 1);
            isolated = false;
            break;
        }
    }

    if (isolated) {
        sg_contour[0] = start;
        return 1;
    }

    while (count < sg_pixels) {
        int from_direction = vision_neighbor_direction(current, previous);
        int search_direction;
        VISION_POINT_T next = current;

        sg_contour[count++] = current;
        search_direction = (from_direction + 1) & 7;
        for (int step = 0; step < 8; step++) {
            VISION_POINT_T examined = vision_neighbor(current, search_direction + step);

            if (vision_border_pixel(examined.x, examined.y)) {
                next = examined;
                break;
            }
        }

        if ((next.x == current.x) && (next.y == current.y)) {
            break;
        }
        if ((current.x == start.x) && (current.y == start.y) && (count > 2)) {
            count--;
            break;
        }
        previous = current;
        current = next;
    }

    return count;
}

static uint32_t vision_compress_contour(uint32_t count)
{
    uint32_t output = 0;

    for (uint32_t i = 0; i < count; i++) {
        if ((output >= 2)
            && (sg_contour[output - 1].x - sg_contour[output - 2].x)
                   * (sg_contour[i].y - sg_contour[output - 1].y)
               == (sg_contour[output - 1].y - sg_contour[output - 2].y)
                      * (sg_contour[i].x - sg_contour[output - 1].x)
            && ((sg_contour[i].x - sg_contour[output - 1].x)
                    * (sg_contour[output - 1].x - sg_contour[output - 2].x)
                + (sg_contour[i].y - sg_contour[output - 1].y)
                      * (sg_contour[output - 1].y - sg_contour[output - 2].y)) >= 0) {
            sg_contour[output - 1] = sg_contour[i];
        } else {
            sg_contour[output++] = sg_contour[i];
        }
    }

    while ((output >= 3) && (sg_contour[0].x == sg_contour[output - 1].x)
           && (sg_contour[0].y == sg_contour[output - 1].y)) {
        output--;
    }

    return output;
}

static float vision_point_line_distance(VISION_POINT_T point, VISION_POINT_T start, VISION_POINT_T end)
{
    float dx = (float)end.x - start.x;
    float dy = (float)end.y - start.y;
    float denominator = sqrtf(dx * dx + dy * dy);

    if (denominator < 0.000001f) {
        return vision_distance(point, start);
    }

    return fabsf(dy * point.x - dx * point.y + end.x * start.y - end.y * start.x) / denominator;
}

static VISION_POINT_T vision_arc_point(const VISION_POINT_T *points, uint32_t count,
                                       uint32_t start, uint32_t index)
{
    return points[(start + index) % count];
}

static void vision_approx_arc(const VISION_POINT_T *points, uint32_t count, uint32_t start,
                              uint32_t first, uint32_t last, float epsilon,
                              uint8_t *keep, uint32_t keep_offset, int depth)
{
    float maximum = 0.0f;
    uint32_t split = first;
    VISION_POINT_T first_point = vision_arc_point(points, count, start, first);
    VISION_POINT_T last_point = vision_arc_point(points, count, start, last);

    if ((last <= first + 1) || (depth >= 64)) {
        return;
    }
    for (uint32_t i = first + 1; i < last; i++) {
        VISION_POINT_T point = vision_arc_point(points, count, start, i);
        float distance = vision_point_line_distance(point, first_point, last_point);

        if (distance > maximum) {
            maximum = distance;
            split = i;
        }
    }
    if (maximum > epsilon) {
        keep[(keep_offset + split) % count] = 1;
        vision_approx_arc(points, count, start, first, split, epsilon,
                          keep, keep_offset, depth + 1);
        vision_approx_arc(points, count, start, split, last, epsilon,
                          keep, keep_offset, depth + 1);
    }
}

static uint32_t vision_approx_polygon(const VISION_POINT_T *points, uint32_t count,
                                      VISION_POINT_T *output, float epsilon)
{
    uint32_t result_count = 0;
    uint32_t second = count / 2;
    int maximum_distance = -1;

    if ((count < 4) || (count > sg_pixels)) {
        return 0;
    }
    memset(sg_approx_keep, 0, count);
    sg_approx_keep[0] = 1;
    for (uint32_t i = 1; i < count; i++) {
        int dx = points[i].x - points[0].x;
        int dy = points[i].y - points[0].y;
        int distance = dx * dx + dy * dy;

        if (distance > maximum_distance) {
            maximum_distance = distance;
            second = i;
        }
    }
    if (second < 2) {
        return 0;
    }

    sg_approx_keep[second] = 1;
    vision_approx_arc(points, count, 0, 0, second, epsilon, sg_approx_keep, 0, 0);
    vision_approx_arc(points, count, second, 0, count - second,
                      epsilon, sg_approx_keep, second, 0);

    for (uint32_t i = 0; i < count; i++) {
        if (sg_approx_keep[i]) {
            if (result_count >= VISION_MAX_POLYGON_POINTS) {
                return VISION_MAX_POLYGON_POINTS + 1;
            }
            if (result_count < VISION_MAX_QUAD_VERTICES) {
                output[result_count] = points[i];
            }
            result_count++;
        }
    }

    return result_count;
}
#endif

#if VISION_ENABLE_TARGET_SELECTION
static float vision_interior_angle(VISION_POINT_T previous, VISION_POINT_T current, VISION_POINT_T next)
{
    float ax = (float)previous.x - current.x;
    float ay = (float)previous.y - current.y;
    float bx = (float)next.x - current.x;
    float by = (float)next.y - current.y;
    float denominator = sqrtf((ax * ax + ay * ay) * (bx * bx + by * by));

    if (denominator < 0.000001f) {
        return 0.0f;
    }

    float cosine = (ax * bx + ay * by) / denominator;
    cosine = cosine > 1.0f ? 1.0f : (cosine < -1.0f ? -1.0f : cosine);

    return acosf(cosine) * 180.0f / (float)VISION_PI;
}

static VISION_POINT_T vision_rectangle_center(const VISION_POINT_T corners[4])
{
    float ax = corners[2].y - corners[0].y;
    float bx = corners[0].x - corners[2].x;
    float cx = corners[2].x * corners[0].y - corners[0].x * corners[2].y;
    float ay = corners[3].y - corners[1].y;
    float by = corners[1].x - corners[3].x;
    float cy = corners[3].x * corners[1].y - corners[1].x * corners[3].y;
    float denominator = ax * by - ay * bx;
    VISION_POINT_T center;

    if (fabsf(denominator) < 0.000001f) {
        center.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x + 2) / 4;
        center.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y + 2) / 4;
    } else {
        center.x = (int16_t)vision_round((bx * cy - by * cx) / denominator);
        center.y = (int16_t)vision_round((ay * cx - ax * cy) / denominator);
    }

    return center;
}

static bool vision_score_quad(const VISION_POINT_T input[4], VISION_RESULT_T *result)
{
    VISION_POINT_T corners[4];
    float angle_deviation = 0.0f;
    float minimum_edge = 1000000.0f;
    float maximum_edge = 0.0f;
    float area = vision_polygon_area(input, 4);
    float angle_score;
    float area_score;
    float total_score;

    if (area < (float)sg_params.min_area) {
        return false;
    }
    if (!vision_order_corners(input, corners)) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        float edge;
        float angle;

        if ((corners[i].x <= sg_params.border_margin) || (corners[i].y <= sg_params.border_margin)
            || (corners[i].x >= sg_width - sg_params.border_margin - 1)
            || (corners[i].y >= sg_height - sg_params.border_margin - 1)) {
            return false;
        }
        edge = vision_distance(corners[i], corners[(i + 1) & 3]);
        minimum_edge = edge < minimum_edge ? edge : minimum_edge;
        maximum_edge = edge > maximum_edge ? edge : maximum_edge;
        angle = vision_interior_angle(corners[(i + 3) & 3], corners[i], corners[(i + 1) & 3]);
        if ((angle < VISION_MIN_ANGLE_DEG) || (angle > VISION_MAX_ANGLE_DEG)) {
            return false;
        }
        angle_deviation += fabsf(angle - 90.0f);
    }
    if ((minimum_edge < 0.000001f) || (maximum_edge / minimum_edge > VISION_MAX_EDGE_RATIO)) {
        return false;
    }

    angle_score = 100.0f - angle_deviation * 0.25f;
    area_score = area / (sg_pixels * 0.5f) * 100.0f;
    area_score = area_score > 100.0f ? 100.0f : area_score;
    total_score = 0.6f * angle_score + 0.4f * area_score;
    if (result->target_found && (total_score * 100.0f <= result->score_x100)) {
        return false;
    }

    result->target_found = true;
    result->score_x100 = vision_round(total_score * 100.0f);
    result->area = (uint32_t)vision_round(area);
    memcpy(result->corners, corners, sizeof(corners));

    return true;
}

static void vision_detect_target(VISION_RESULT_T *result)
{
    memset(sg_visited, 0, sg_pixels);

    for (uint32_t start = 0; start < sg_pixels; start++) {
        uint32_t stack_size = 0;
        int16_t min_x;
        int16_t min_y;
        int16_t max_x;
        int16_t max_y;

        if ((0 == sg_edges[start]) || sg_visited[start]) {
            continue;
        }

        min_x = max_x = (int16_t)(start % sg_width);
        min_y = max_y = (int16_t)(start / sg_width);
        sg_stack[stack_size++] = start;
        sg_visited[start] = 1;
        while (stack_size > 0) {
            uint32_t index = sg_stack[--stack_size];
            int x = (int)(index % sg_width);
            int y = (int)(index / sg_width);

            min_x = x < min_x ? (int16_t)x : min_x;
            min_y = y < min_y ? (int16_t)y : min_y;
            max_x = x > max_x ? (int16_t)x : max_x;
            max_y = y > max_y ? (int16_t)y : max_y;

            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;
                    uint32_t neighbor;

                    if ((nx < 0) || (nx >= sg_width) || (ny < 0) || (ny >= sg_height)) {
                        continue;
                    }
                    neighbor = (uint32_t)ny * sg_width + nx;
                    if ((0 != sg_edges[neighbor]) && !sg_visited[neighbor]) {
                        sg_visited[neighbor] = 1;
                        sg_stack[stack_size++] = neighbor;
                    }
                }
            }
        }

        if ((uint32_t)(max_x - min_x + 1) * (max_y - min_y + 1) < sg_params.min_area) {
            continue;
        }

        uint32_t count = vision_trace_contour((VISION_POINT_T){(int16_t)(start % sg_width),
                                                               (int16_t)(start / sg_width)}, 6);
        if (count < 4) {
            continue;
        }
        count = vision_compress_contour(count);
        VISION_POINT_T polygon[VISION_MAX_QUAD_VERTICES];
        float perimeter = vision_polygon_perimeter(sg_contour, count, true);
        uint32_t vertices = vision_approx_polygon(sg_contour, count, polygon,
                                                  perimeter * sg_params.eps_x100 * 0.0001f);
        if ((vertices >= 4) && (vertices <= VISION_MAX_QUAD_VERTICES)) {
            for (uint32_t a = 0; a < vertices - 3; a++) {
                for (uint32_t b = a + 1; b < vertices - 2; b++) {
                    for (uint32_t c = b + 1; c < vertices - 1; c++) {
                        for (uint32_t d = c + 1; d < vertices; d++) {
                            VISION_POINT_T quad[4] = {polygon[a], polygon[b], polygon[c], polygon[d]};

                            vision_score_quad(quad, result);
                        }
                    }
                }
            }
        }
    }
}
#endif

#if VISION_ENABLE_PERSPECTIVE
static bool vision_homography(const VISION_POINT_T corners[4], float matrix[9])
{
    const float physical[4][2] = {
        {0.0f, 0.0f},
        {VISION_TARGET_WIDTH_MM, 0.0f},
        {VISION_TARGET_WIDTH_MM, VISION_TARGET_HEIGHT_MM},
        {0.0f, VISION_TARGET_HEIGHT_MM}
    };
    float a[8][9] = {{0}};

    for (int i = 0; i < 4; i++) {
        float x = physical[i][0];
        float y = physical[i][1];
        float u = corners[i].x;
        float v = corners[i].y;

        a[2 * i][0] = x;
        a[2 * i][1] = y;
        a[2 * i][2] = 1.0f;
        a[2 * i][6] = -u * x;
        a[2 * i][7] = -u * y;
        a[2 * i][8] = u;
        a[2 * i + 1][3] = x;
        a[2 * i + 1][4] = y;
        a[2 * i + 1][5] = 1.0f;
        a[2 * i + 1][6] = -v * x;
        a[2 * i + 1][7] = -v * y;
        a[2 * i + 1][8] = v;
    }

    for (int column = 0; column < 8; column++) {
        int pivot = column;

        for (int row = column + 1; row < 8; row++) {
            if (fabsf(a[row][column]) > fabsf(a[pivot][column])) {
                pivot = row;
            }
        }
        if (fabsf(a[pivot][column]) < 0.000001f) {
            return false;
        }
        if (pivot != column) {
            for (int k = column; k < 9; k++) {
                float tmp = a[column][k];
                a[column][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }

        for (int row = column + 1; row < 8; row++) {
            float factor = a[row][column] / a[column][column];
            for (int k = column + 1; k < 9; k++) {
                a[row][k] -= factor * a[column][k];
            }
        }
    }

    for (int row = 7; row >= 0; row--) {
        float value = a[row][8];
        for (int k = row + 1; k < 8; k++) {
            value -= a[row][k] * matrix[k];
        }
        matrix[row] = value / a[row][row];
    }
    matrix[8] = 1.0f;

    return true;
}

static VISION_POINT_T vision_project(const float matrix[9], float x, float y)
{
    float denominator = matrix[6] * x + matrix[7] * y + matrix[8];
    VISION_POINT_T point = {0, 0};

    if (fabsf(denominator) < 0.000001f) {
        return point;
    }
    point.x = (int16_t)vision_round((matrix[0] * x + matrix[1] * y + matrix[2]) / denominator);
    point.y = (int16_t)vision_round((matrix[3] * x + matrix[4] * y + matrix[5]) / denominator);

    return point;
}

static void vision_project_circle(VISION_RESULT_T *result)
{
    float matrix[9];

    if (!vision_homography(result->corners, matrix)) {
        return;
    }

    float center_x = 0.0f;
    float center_y = 0.0f;
    for (int i = 0; i < VISION_CIRCLE_POINT_COUNT; i++) {
        float angle = 2.0f * (float)VISION_PI * i / VISION_CIRCLE_POINT_COUNT;
        VISION_POINT_T point = vision_project(matrix,
                                              VISION_TARGET_WIDTH_MM * 0.5f
                                                  + VISION_CIRCLE_RADIUS_MM * cosf(angle),
                                              VISION_TARGET_HEIGHT_MM * 0.5f
                                                  + VISION_CIRCLE_RADIUS_MM * sinf(angle));
        result->circle[i] = point;
        center_x += point.x;
        center_y += point.y;
    }

    center_x /= VISION_CIRCLE_POINT_COUNT;
    center_y /= VISION_CIRCLE_POINT_COUNT;
    int offset_x = result->center.x - vision_round(center_x);
    int offset_y = result->center.y - vision_round(center_y);
    for (int i = 0; i < VISION_CIRCLE_POINT_COUNT; i++) {
        result->circle[i].x += offset_x;
        result->circle[i].y += offset_y;
    }
    result->projection_valid = true;
}
#endif

bool vision_init(uint16_t width, uint16_t height)
{
    if ((width < 3) || (height < 3)) {
        return false;
    }

    sg_width = width;
    sg_height = height;
    sg_pixels = (uint32_t)width * height;
    sg_ready = true;
#if VISION_ENABLE_BINARIZATION
    sg_gray = vision_alloc(sg_pixels);
    sg_blur = vision_alloc(sg_pixels * sizeof(*sg_blur));
    sg_binary = vision_alloc(sg_pixels);
    sg_ready = sg_ready && (NULL != sg_gray) && (NULL != sg_blur) && (NULL != sg_binary);
#endif
#if VISION_ENABLE_CANNY
    sg_gx = vision_alloc(sg_pixels * sizeof(*sg_gx));
    sg_gy = vision_alloc(sg_pixels * sizeof(*sg_gy));
    sg_magnitude = vision_alloc(sg_pixels * sizeof(*sg_magnitude));
    sg_edges = vision_alloc(sg_pixels);
    sg_ready = sg_ready && (NULL != sg_gx) && (NULL != sg_gy)
               && (NULL != sg_magnitude) && (NULL != sg_edges);
#endif
#if VISION_ENABLE_CANNY || VISION_ENABLE_CONTOUR
    sg_stack = vision_alloc(sg_pixels * sizeof(*sg_stack));
    sg_ready = sg_ready && (NULL != sg_stack);
#endif
#if VISION_ENABLE_CONTOUR
    sg_visited = vision_alloc(sg_pixels);
    sg_approx_keep = vision_alloc(sg_pixels);
    sg_contour = vision_alloc(sg_pixels * sizeof(*sg_contour));
    sg_ready = sg_ready && (NULL != sg_visited) && (NULL != sg_approx_keep)
               && (NULL != sg_contour);
#endif

    return sg_ready;
}

bool vision_process(const uint8_t *yuyv, uint32_t data_len, VISION_RESULT_T *result)
{
    if (!sg_ready || (NULL == yuyv) || (NULL == result) || (data_len < sg_pixels * 2)) {
        return false;
    }

    memset(result, 0, sizeof(*result));
    result->center = (VISION_POINT_T){-1, -1};
#if VISION_ENABLE_BINARIZATION
    if (!sg_params.proc) {
        for (uint32_t i = 0; i < sg_pixels; i++) {
            sg_gray[i] = (i * 2) < data_len ? yuyv[i * 2] : 0;
        }
        return true;
    }
    result->threshold = vision_binarize(yuyv, data_len, result);
#endif
#if VISION_ENABLE_CANNY && VISION_ENABLE_TARGET_SELECTION
    if (sg_params.cmode != 2) {
        bool detect_now = !sg_cache_valid || (VISION_DETECT_INTERVAL_FRAMES <= 1)
                          || ((sg_detect_frame % VISION_DETECT_INTERVAL_FRAMES) == 0);

        sg_detect_frame++;
        if (detect_now) {
            result->edge_count = vision_canny();
            vision_detect_target(result);
            if (result->target_found) {
                result->center = vision_rectangle_center(result->corners);
#if VISION_ENABLE_PERSPECTIVE
                vision_project_circle(result);
#endif
            } else if (sg_params.cmode == 1) {
                uint32_t best_count = 0;
                int64_t best_x = 0, best_y = 0;

                memset(sg_visited, 0, sg_pixels);
                for (uint32_t start = 0; start < sg_pixels; start++) {
                    uint32_t stack_size = 0;
                    uint32_t count = 0;
                    int16_t min_x, min_y, max_x, max_y;
                    int64_t cx = 0, cy = 0;

                    if ((0 == sg_binary[start]) || sg_visited[start]) {
                        continue;
                    }
                    min_x = max_x = (int16_t)(start % sg_width);
                    min_y = max_y = (int16_t)(start / sg_width);
                    sg_stack[stack_size++] = start;
                    sg_visited[start] = 1;
                    while (stack_size > 0) {
                        uint32_t index = sg_stack[--stack_size];
                        int x = (int)(index % sg_width);
                        int y = (int)(index / sg_width);

                        count++;
                        cx += x;
                        cy += y;
                        min_x = x < min_x ? (int16_t)x : min_x;
                        min_y = y < min_y ? (int16_t)y : min_y;
                        max_x = x > max_x ? (int16_t)x : max_x;
                        max_y = y > max_y ? (int16_t)y : max_y;
                        for (int dy = -1; dy <= 1; dy++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                int nx = x + dx;
                                int ny = y + dy;
                                uint32_t neighbor;

                                if ((nx < 0) || (nx >= sg_width) || (ny < 0) || (ny >= sg_height)) {
                                    continue;
                                }
                                neighbor = (uint32_t)ny * sg_width + nx;
                                if ((0 != sg_binary[neighbor]) && !sg_visited[neighbor]) {
                                    sg_visited[neighbor] = 1;
                                    sg_stack[stack_size++] = neighbor;
                                }
                            }
                        }
                    }
                    if ((count >= 100) && (count > best_count)
                        && (min_x > sg_params.border_margin) && (min_y > sg_params.border_margin)
                        && (max_x < sg_width - sg_params.border_margin - 1)
                        && (max_y < sg_height - sg_params.border_margin - 1)) {
                        best_count = count;
                        best_x = cx;
                        best_y = cy;
                    }
                }
                if (best_count > 0) {
                    result->target_found = true;
                    result->center.x = (int16_t)(best_x / best_count);
                    result->center.y = (int16_t)(best_y / best_count);
                    result->area = best_count;
                }
            }
            sg_cached_result = *result;
            sg_cache_valid = true;
        } else {
            uint8_t threshold = result->threshold;

            *result = sg_cached_result;
            result->threshold = threshold;
        }
    }
#elif VISION_ENABLE_CANNY
    result->edge_count = vision_canny();
#endif

    return true;
}

const uint8_t *vision_get_binary(void)
{
#if VISION_ENABLE_BINARIZATION
    return sg_binary;
#else
    return NULL;
#endif
}

const uint8_t *vision_get_edges(void)
{
#if VISION_ENABLE_CANNY
    return sg_edges;
#else
    return NULL;
#endif
}

const VISION_PARAMS_T *vision_get_params(void)
{
    return &sg_params;
}

bool vision_set_param(const char *name, int32_t value)
{
    if (0 == strcmp(name, "thr")) {
        if ((value < -1) || (value > 255)) {
            return false;
        }
        sg_params.threshold = (int16_t)value;
    } else if (0 == strcmp(name, "canny_lo")) {
        if ((value < 0) || (value > 1020)) {
            return false;
        }
        sg_params.canny_low = (uint16_t)value;
    } else if (0 == strcmp(name, "canny_hi")) {
        if ((value < 0) || (value > 1020)) {
            return false;
        }
        sg_params.canny_high = (uint16_t)value;
    } else if (0 == strcmp(name, "min_area")) {
        if ((value < 1) || (value > (int32_t)sg_pixels)) {
            return false;
        }
        sg_params.min_area = (uint32_t)value;
    } else if (0 == strcmp(name, "margin")) {
        if ((value < 0) || (value > 20)) {
            return false;
        }
        sg_params.border_margin = (uint16_t)value;
    } else if (0 == strcmp(name, "morph")) {
        sg_params.morph = (0 != value);
    } else if (0 == strcmp(name, "eps")) {
        if ((value < 50) || (value > 1000)) {
            return false;
        }
        sg_params.eps_x100 = (uint16_t)value;
    } else if (0 == strcmp(name, "proc")) {
        sg_params.proc = (0 != value);
    } else if (0 == strcmp(name, "cmode")) {
        if ((value < 0) || (value > 2)) {
            return false;
        }
        sg_params.cmode = (uint16_t)value;
    } else if (0 == strcmp(name, "u_lo")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.u_lo = (uint16_t)value;
    } else if (0 == strcmp(name, "v_lo")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.v_lo = (uint16_t)value;
    } else if (0 == strcmp(name, "u_hi")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.u_hi = (uint16_t)value;
    } else if (0 == strcmp(name, "v_hi")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.v_hi = (uint16_t)value;
    } else if (0 == strcmp(name, "cphase")) {
        if ((value < 0) || (value > 3)) {
            return false;
        }
        sg_params.cphase = (uint16_t)value;
    } else if (0 == strcmp(name, "y_lo")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.y_lo = (uint16_t)value;
    } else if (0 == strcmp(name, "y_hi")) {
        if ((value < 0) || (value > 255)) {
            return false;
        }
        sg_params.y_hi = (uint16_t)value;
    } else {
        return false;
    }

    return true;
}

void vision_dump_params(void)
{
    PR_NOTICE("vision params thr:%d canny_lo:%u canny_hi:%u min_area:%u margin:%u morph:%u eps:%u proc:%u",
              sg_params.threshold, sg_params.canny_low, sg_params.canny_high, sg_params.min_area,
              sg_params.border_margin, sg_params.morph, sg_params.eps_x100, sg_params.proc);
    PR_NOTICE("vision chroma cmode:%u u:[%u,%u] v:[%u,%u] y:[%u,%u] cphase:%u", sg_params.cmode, sg_params.u_lo,
              sg_params.u_hi, sg_params.v_lo, sg_params.v_hi, sg_params.y_lo, sg_params.y_hi, sg_params.cphase);
}

const uint8_t *vision_get_gray(void)
{
#if VISION_ENABLE_BINARIZATION
    return sg_gray;
#else
    return NULL;
#endif
}
