#ifndef CAMERA_OPTIMIZER_H
#define CAMERA_OPTIMIZER_H

#include "esp_camera.h"
#include "image_analyzer.h"

// Camera parameter settings structure
struct CameraSettings {
    int brightness;       // -2 to 2
    int contrast;         // -2 to 2
    int saturation;       // -2 to 2
    int sharpness;        // -2 to 2
    int agcGain;          // 0 to 30 (auto gain control)
    int aecValue;         // 0 to 1200 (auto exposure control)
    bool flashEnabled;    // Flash LED on/off
    bool awbGain;         // Auto white balance gain
    bool gainCtrl;        // Gain control enable
    bool exposureCtrl;    // Exposure control enable
};

// Optimization result
struct OptimizationResult {
    CameraSettings settings;
    ImageQualityMetrics metrics;
    int iterations;
    bool converged;
    float improvementPercent;
};

class CameraOptimizer {
public:
    CameraOptimizer();

    // Main optimization function - auto-tune camera settings
    OptimizationResult optimize(int maxIterations = 10);

    // Get/Set current camera settings
    CameraSettings getCurrentSettings();
    bool applySettings(const CameraSettings& settings);

    // Reset to default settings
    void resetToDefaults();

    // Manual adjustment helpers
    void adjustForDarkness(CameraSettings& settings);
    void adjustForBrightness(CameraSettings& settings);
    void adjustForLowContrast(CameraSettings& settings);
    void adjustForNoise(CameraSettings& settings);

    // Configuration
    void setConvergenceThreshold(float threshold);
    void setQualityTarget(float target);

private:
    ImageAnalyzer analyzer;
    CameraSettings currentSettings;

    float convergenceThreshold;   // Minimum improvement to continue (default 2.0%)
    float qualityTarget;          // Target quality score (default 75.0)

    // Hill-climbing optimization
    OptimizationResult hillClimbing(int maxIterations);

    // Evaluate current settings
    ImageQualityMetrics evaluateSettings();

    // Generate neighbor settings (small adjustments)
    CameraSettings generateNeighbor(const CameraSettings& current);

    // Parameter adjustment helpers
    int clampBrightness(int value);
    int clampContrast(int value);
    int clampGain(int value);
    int clampExposure(int value);

    // Determine primary issue from metrics
    enum IssueType {
        ISSUE_TOO_DARK,
        ISSUE_TOO_BRIGHT,
        ISSUE_LOW_CONTRAST,
        ISSUE_HIGH_NOISE,
        ISSUE_NONE
    };
    IssueType identifyMainIssue(const ImageQualityMetrics& metrics);

    // Helper to print settings
    void printSettings(const CameraSettings& settings);
};

#endif // CAMERA_OPTIMIZER_H
