#ifndef IMAGE_ANALYZER_H
#define IMAGE_ANALYZER_H

#include "esp_camera.h"
#include <stdint.h>

// Structure to hold image quality metrics
struct ImageQualityMetrics {
    float brightness;        // Average luminance (0-255)
    float contrast;          // Standard deviation of luminance
    float noiseLevel;        // Estimated noise (variance in uniform regions)
    float overexposure;      // Percentage of overexposed pixels (0-100)
    float underexposure;     // Percentage of underexposed pixels (0-100)
    float sharpness;         // Edge strength measure
    float qualityScore;      // Composite quality score (0-100)
    bool isDark;             // True if image is too dark
    bool isBright;           // True if image is too bright
};

// Histogram structure (lightweight, 256 bins)
struct Histogram {
    uint32_t bins[256];      // Luminance histogram
    uint32_t totalPixels;    // Total number of pixels sampled
};

class ImageAnalyzer {
public:
    ImageAnalyzer();

    // Main analysis function
    ImageQualityMetrics analyze(camera_fb_t* fb);

    // Individual metric calculations
    Histogram generateHistogram(camera_fb_t* fb);
    float calculateBrightness(const Histogram& hist);
    float calculateContrast(const Histogram& hist, float brightness);
    float calculateNoiseLevel(camera_fb_t* fb);
    float calculateOverexposure(const Histogram& hist);
    float calculateUnderexposure(const Histogram& hist);
    float calculateSharpness(camera_fb_t* fb);

    // Composite quality scoring
    float calculateQualityScore(const ImageQualityMetrics& metrics);

    // Helper to print metrics
    void printMetrics(const ImageQualityMetrics& metrics);

private:
    // JPEG decoding helper - extract luminance from JPEG
    // For ESP32 memory constraints, we'll sample pixels instead of full decode
    void extractLuminanceSamples(camera_fb_t* fb, uint8_t* samples, int numSamples);

    // Configuration
    static const int SAMPLE_SIZE = 1000;  // Number of pixels to sample for analysis
    static const uint8_t OVEREXPOSED_THRESHOLD = 250;
    static const uint8_t UNDEREXPOSED_THRESHOLD = 5;
    static const float DARK_THRESHOLD;
    static const float BRIGHT_THRESHOLD;
};

#endif // IMAGE_ANALYZER_H
