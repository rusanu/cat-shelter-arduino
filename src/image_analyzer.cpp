#include "image_analyzer.h"
#include <Arduino.h>
#include <math.h>

// Static constants
const float ImageAnalyzer::DARK_THRESHOLD = 40.0f;
const float ImageAnalyzer::BRIGHT_THRESHOLD = 215.0f;

ImageAnalyzer::ImageAnalyzer() {
    // Constructor
}

ImageQualityMetrics ImageAnalyzer::analyze(camera_fb_t* fb) {
    ImageQualityMetrics metrics;

    if (!fb || fb->len == 0) {
        Serial.println("Invalid frame buffer for analysis");
        memset(&metrics, 0, sizeof(metrics));
        return metrics;
    }

    Serial.println("Analyzing image quality...");

    // Generate histogram
    Histogram hist = generateHistogram(fb);

    // Calculate individual metrics
    metrics.brightness = calculateBrightness(hist);
    metrics.contrast = calculateContrast(hist, metrics.brightness);
    metrics.noiseLevel = calculateNoiseLevel(fb);
    metrics.overexposure = calculateOverexposure(hist);
    metrics.underexposure = calculateUnderexposure(hist);
    metrics.sharpness = calculateSharpness(fb);

    // Determine if image is too dark or bright
    metrics.isDark = (metrics.brightness < DARK_THRESHOLD);
    metrics.isBright = (metrics.brightness > BRIGHT_THRESHOLD);

    // Calculate composite quality score
    metrics.qualityScore = calculateQualityScore(metrics);

    return metrics;
}

Histogram ImageAnalyzer::generateHistogram(camera_fb_t* fb) {
    Histogram hist;
    memset(&hist, 0, sizeof(hist));

    // For JPEG images, we'll extract luminance samples
    // This is a lightweight approach for ESP32
    uint8_t samples[SAMPLE_SIZE];
    extractLuminanceSamples(fb, samples, SAMPLE_SIZE);

    // Build histogram from samples
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        hist.bins[samples[i]]++;
    }
    hist.totalPixels = SAMPLE_SIZE;

    return hist;
}

void ImageAnalyzer::extractLuminanceSamples(camera_fb_t* fb, uint8_t* samples, int numSamples) {
    // For JPEG images on ESP32, full decoding is expensive
    // We'll use a simple sampling strategy: read bytes from JPEG data
    // and use them as approximate luminance values
    // This is not perfect but gives us useful metrics for auto-tuning

    if (!fb || !samples || numSamples <= 0) return;

    // Skip JPEG header (typically first ~600 bytes contain metadata)
    size_t skipBytes = min((size_t)600, fb->len / 4);
    size_t dataLen = fb->len - skipBytes;
    uint8_t* data = fb->buf + skipBytes;

    // Sample uniformly across the image data
    int step = max(1, (int)(dataLen / numSamples));

    for (int i = 0; i < numSamples && (i * step) < dataLen; i++) {
        samples[i] = data[i * step];
    }
}

float ImageAnalyzer::calculateBrightness(const Histogram& hist) {
    if (hist.totalPixels == 0) return 0.0f;

    uint64_t sum = 0;
    for (int i = 0; i < 256; i++) {
        sum += i * hist.bins[i];
    }

    return (float)sum / hist.totalPixels;
}

float ImageAnalyzer::calculateContrast(const Histogram& hist, float brightness) {
    if (hist.totalPixels == 0) return 0.0f;

    // Calculate standard deviation
    float variance = 0.0f;
    for (int i = 0; i < 256; i++) {
        float diff = i - brightness;
        variance += diff * diff * hist.bins[i];
    }
    variance /= hist.totalPixels;

    return sqrt(variance);
}

float ImageAnalyzer::calculateNoiseLevel(camera_fb_t* fb) {
    // Estimate noise by analyzing variance in sampled regions
    // Higher variance indicates more noise
    uint8_t samples[SAMPLE_SIZE];
    extractLuminanceSamples(fb, samples, SAMPLE_SIZE);

    // Calculate variance of samples
    float mean = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        mean += samples[i];
    }
    mean /= SAMPLE_SIZE;

    float variance = 0.0f;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        float diff = samples[i] - mean;
        variance += diff * diff;
    }
    variance /= SAMPLE_SIZE;

    // Normalize to 0-100 range (typical noise variance is 0-5000)
    return min(100.0f, sqrt(variance));
}

float ImageAnalyzer::calculateOverexposure(const Histogram& hist) {
    if (hist.totalPixels == 0) return 0.0f;

    uint32_t overexposedPixels = 0;
    for (int i = OVEREXPOSED_THRESHOLD; i < 256; i++) {
        overexposedPixels += hist.bins[i];
    }

    return (100.0f * overexposedPixels) / hist.totalPixels;
}

float ImageAnalyzer::calculateUnderexposure(const Histogram& hist) {
    if (hist.totalPixels == 0) return 0.0f;

    uint32_t underexposedPixels = 0;
    for (int i = 0; i <= UNDEREXPOSED_THRESHOLD; i++) {
        underexposedPixels += hist.bins[i];
    }

    return (100.0f * underexposedPixels) / hist.totalPixels;
}

float ImageAnalyzer::calculateSharpness(camera_fb_t* fb) {
    // Estimate sharpness using gradient magnitude on samples
    // Sharp images have stronger edges (higher gradients)
    uint8_t samples[SAMPLE_SIZE];
    extractLuminanceSamples(fb, samples, SAMPLE_SIZE);

    float gradientSum = 0.0f;
    int gradientCount = 0;

    // Calculate gradient magnitude using adjacent samples
    for (int i = 1; i < SAMPLE_SIZE; i++) {
        float gradient = abs(samples[i] - samples[i-1]);
        gradientSum += gradient;
        gradientCount++;
    }

    if (gradientCount == 0) return 0.0f;

    // Average gradient magnitude (normalize to 0-100)
    float avgGradient = gradientSum / gradientCount;
    return min(100.0f, avgGradient * 2.0f);  // Scale factor for better range
}

float ImageAnalyzer::calculateQualityScore(const ImageQualityMetrics& metrics) {
    // Composite quality score based on multiple factors
    // Perfect score: bright enough, good contrast, low noise, minimal clipping

    float score = 100.0f;

    // Penalize for too dark or too bright
    if (metrics.brightness < DARK_THRESHOLD) {
        score -= (DARK_THRESHOLD - metrics.brightness) * 1.5f;
    } else if (metrics.brightness > BRIGHT_THRESHOLD) {
        score -= (metrics.brightness - BRIGHT_THRESHOLD) * 1.5f;
    }

    // Penalize for low contrast
    if (metrics.contrast < 30.0f) {
        score -= (30.0f - metrics.contrast);
    }

    // Penalize for high noise
    score -= metrics.noiseLevel * 0.5f;

    // Penalize for overexposure
    score -= metrics.overexposure * 2.0f;

    // Penalize for underexposure
    score -= metrics.underexposure * 1.5f;

    // Bonus for good sharpness
    if (metrics.sharpness > 20.0f) {
        score += min(10.0f, (metrics.sharpness - 20.0f) * 0.2f);
    }

    // Clamp to 0-100 range
    return max(0.0f, min(100.0f, score));
}

void ImageAnalyzer::printMetrics(const ImageQualityMetrics& metrics) {
    Serial.println("=== Image Quality Metrics ===");
    Serial.printf("Brightness:     %.2f (target: 80-180)\n", metrics.brightness);
    Serial.printf("Contrast:       %.2f (target: >30)\n", metrics.contrast);
    Serial.printf("Noise Level:    %.2f (target: <20)\n", metrics.noiseLevel);
    Serial.printf("Overexposure:   %.2f%% (target: <5%%)\n", metrics.overexposure);
    Serial.printf("Underexposure:  %.2f%% (target: <10%%)\n", metrics.underexposure);
    Serial.printf("Sharpness:      %.2f (target: >20)\n", metrics.sharpness);
    Serial.printf("Quality Score:  %.2f/100\n", metrics.qualityScore);
    Serial.printf("Status: %s\n", metrics.isDark ? "TOO DARK" :
                                   metrics.isBright ? "TOO BRIGHT" : "OK");
    Serial.println("============================");
}
