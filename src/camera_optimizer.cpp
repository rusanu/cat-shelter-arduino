#include "camera_optimizer.h"
#include <Arduino.h>

extern camera_fb_t* capturePhoto();
void releasePhoto(camera_fb_t* fb);

CameraOptimizer::CameraOptimizer() {
    convergenceThreshold = 2.0f;  // 2% improvement threshold
    qualityTarget = 75.0f;         // Target quality score
    resetToDefaults();
}

void CameraOptimizer::resetToDefaults() {
    currentSettings.brightness = 0;
    currentSettings.contrast = 0;
    currentSettings.saturation = 0;
    currentSettings.sharpness = 0;
    currentSettings.agcGain = 0;
    currentSettings.aecValue = 300;
    currentSettings.flashEnabled = false;
    currentSettings.awbGain = true;
    currentSettings.gainCtrl = true;
    currentSettings.exposureCtrl = true;
}

CameraSettings CameraOptimizer::getCurrentSettings() {
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        // Read current settings from sensor
        // Note: Not all sensors support reading back settings, so we keep our own copy
        return currentSettings;
    }
    return currentSettings;
}

bool CameraOptimizer::applySettings(const CameraSettings& settings) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("Failed to get sensor for applying settings");
        return false;
    }

    // Apply settings to sensor
    s->set_brightness(s, settings.brightness);
    s->set_contrast(s, settings.contrast);
    s->set_saturation(s, settings.saturation);
    s->set_sharpness(s, settings.sharpness);
    s->set_agc_gain(s, settings.agcGain);
    s->set_aec_value(s, settings.aecValue);
    s->set_awb_gain(s, settings.awbGain ? 1 : 0);
    s->set_gain_ctrl(s, settings.gainCtrl ? 1 : 0);
    s->set_exposure_ctrl(s, settings.exposureCtrl ? 1 : 0);

    // Apply flash setting
    // setFlash(settings.flashEnabled);

    // Store current settings
    currentSettings = settings;

    // Wait for sensor to stabilize after settings change
    delay(100);

    return true;
}

OptimizationResult CameraOptimizer::optimize(int maxIterations) {
    Serial.println("\n=== Starting Camera Auto-Tuning ===");
    Serial.printf("Max iterations: %d\n", maxIterations);
    Serial.printf("Quality target: %.1f\n", qualityTarget);
    Serial.printf("Convergence threshold: %.1f%%\n", convergenceThreshold);

    // Apply initial settings
    applySettings(currentSettings);

    // Run hill-climbing optimization
    OptimizationResult result = hillClimbing(maxIterations);

    Serial.println("\n=== Optimization Complete ===");
    Serial.printf("Final quality score: %.2f/100\n", result.metrics.qualityScore);
    Serial.printf("Iterations: %d\n", result.iterations);
    Serial.printf("Converged: %s\n", result.converged ? "Yes" : "No");
    Serial.printf("Improvement: %.1f%%\n", result.improvementPercent);

    return result;
}

OptimizationResult CameraOptimizer::hillClimbing(int maxIterations) {
    OptimizationResult result;
    result.settings = currentSettings;
    result.iterations = 0;
    result.converged = false;

    // Evaluate initial settings
    ImageQualityMetrics initialMetrics = evaluateSettings();
    float bestScore = initialMetrics.qualityScore;
    CameraSettings bestSettings = currentSettings;

    Serial.printf("\nInitial quality score: %.2f/100\n", bestScore);
    analyzer.printMetrics(initialMetrics);

    // Hill-climbing loop
    for (int iter = 0; iter < maxIterations; iter++) {
        result.iterations++;
        Serial.printf("\n--- Iteration %d/%d ---\n", iter + 1, maxIterations);

        // Identify main issue and adjust accordingly
        IssueType issue = identifyMainIssue(initialMetrics);
        CameraSettings newSettings = bestSettings;

        switch (issue) {
            case ISSUE_TOO_DARK:
                Serial.println("Issue: Image too dark");
                adjustForDarkness(newSettings);
                break;
            case ISSUE_TOO_BRIGHT:
                Serial.println("Issue: Image too bright");
                adjustForBrightness(newSettings);
                break;
            case ISSUE_LOW_CONTRAST:
                Serial.println("Issue: Low contrast");
                adjustForLowContrast(newSettings);
                break;
            case ISSUE_HIGH_NOISE:
                Serial.println("Issue: High noise");
                adjustForNoise(newSettings);
                break;
            case ISSUE_NONE:
                Serial.println("No major issues detected, trying neighbor settings");
                newSettings = generateNeighbor(bestSettings);
                break;
        }

        // Apply and evaluate new settings
        applySettings(newSettings);
        ImageQualityMetrics newMetrics = evaluateSettings();
        float newScore = newMetrics.qualityScore;

        Serial.printf("New quality score: %.2f/100 (was %.2f)\n", newScore, bestScore);

        // Check if we improved
        float improvement = ((newScore - bestScore) / bestScore) * 100.0f;
        if (newScore > bestScore) {
            Serial.printf("Improvement: +%.1f%%\n", improvement);
            bestScore = newScore;
            bestSettings = newSettings;
            initialMetrics = newMetrics;

            // Check if we reached target
            if (bestScore >= qualityTarget) {
                Serial.println("Target quality reached!");
                result.converged = true;
                break;
            }

            // Check if improvement is too small
            if (improvement < convergenceThreshold) {
                Serial.println("Improvement below threshold, stopping");
                result.converged = true;
                break;
            }
        } else {
            Serial.printf("No improvement (%.1f%%)\n", improvement);
            // Revert to best settings
            applySettings(bestSettings);
        }

        // Discard first 2 frames after settings change (sensor warm-up)
        camera_fb_t* fb1 = capturePhoto();
        if (fb1) releasePhoto(fb1);
        delay(50);
        camera_fb_t* fb2 = capturePhoto();
        if (fb2) releasePhoto(fb2);
        delay(50);
    }

    // Store result
    result.settings = bestSettings;
    result.metrics = initialMetrics;
    result.improvementPercent = ((bestScore - initialMetrics.qualityScore) /
                                  max(0.1f, initialMetrics.qualityScore)) * 100.0f;

    // Apply final best settings
    applySettings(bestSettings);

    return result;
}

ImageQualityMetrics CameraOptimizer::evaluateSettings() {
    // Discard first frame after settings change
    camera_fb_t* fb0 = capturePhoto();
    if (fb0) releasePhoto(fb0);
    delay(100);

    // Capture and analyze
    camera_fb_t* fb = capturePhoto();
    if (!fb) {
        ImageQualityMetrics emptyMetrics;
        memset(&emptyMetrics, 0, sizeof(emptyMetrics));
        return emptyMetrics;
    }

    ImageQualityMetrics metrics = analyzer.analyze(fb);
    releasePhoto(fb);

    return metrics;
}

CameraOptimizer::IssueType CameraOptimizer::identifyMainIssue(const ImageQualityMetrics& metrics) {
    // Prioritize issues: darkness/brightness first, then contrast, then noise
    if (metrics.isDark || metrics.underexposure > 15.0f) {
        return ISSUE_TOO_DARK;
    }
    if (metrics.isBright || metrics.overexposure > 10.0f) {
        return ISSUE_TOO_BRIGHT;
    }
    if (metrics.contrast < 30.0f) {
        return ISSUE_LOW_CONTRAST;
    }
    if (metrics.noiseLevel > 30.0f) {
        return ISSUE_HIGH_NOISE;
    }
    return ISSUE_NONE;
}

void CameraOptimizer::adjustForDarkness(CameraSettings& settings) {
    // Increase brightness: enable flash, increase exposure, increase gain
    if (!settings.flashEnabled && settings.brightness < 80) {
        settings.flashEnabled = true;
        Serial.println("  -> Enabling flash");
    }
    if (settings.aecValue < 1200) {
        settings.aecValue = min(1200, settings.aecValue + 200);
        Serial.printf("  -> Increasing exposure to %d\n", settings.aecValue);
    }
    if (settings.brightness < 2) {
        settings.brightness++;
        Serial.printf("  -> Increasing brightness to %d\n", settings.brightness);
    }
    if (settings.agcGain < 20) {
        settings.agcGain = min(20, settings.agcGain + 5);
        Serial.printf("  -> Increasing gain to %d\n", settings.agcGain);
    }
}

void CameraOptimizer::adjustForBrightness(CameraSettings& settings) {
    // Decrease brightness: disable flash, decrease exposure, decrease gain
    if (settings.flashEnabled) {
        settings.flashEnabled = false;
        Serial.println("  -> Disabling flash");
    }
    if (settings.aecValue > 100) {
        settings.aecValue = max(100, settings.aecValue - 200);
        Serial.printf("  -> Decreasing exposure to %d\n", settings.aecValue);
    }
    if (settings.brightness > -2) {
        settings.brightness--;
        Serial.printf("  -> Decreasing brightness to %d\n", settings.brightness);
    }
    if (settings.agcGain > 0) {
        settings.agcGain = max(0, settings.agcGain - 5);
        Serial.printf("  -> Decreasing gain to %d\n", settings.agcGain);
    }
}

void CameraOptimizer::adjustForLowContrast(CameraSettings& settings) {
    if (settings.contrast < 2) {
        settings.contrast++;
        Serial.printf("  -> Increasing contrast to %d\n", settings.contrast);
    }
    if (settings.sharpness < 2) {
        settings.sharpness++;
        Serial.printf("  -> Increasing sharpness to %d\n", settings.sharpness);
    }
}

void CameraOptimizer::adjustForNoise(CameraSettings& settings) {
    // Reduce noise: decrease gain, increase exposure time instead
    if (settings.agcGain > 0) {
        settings.agcGain = max(0, settings.agcGain - 3);
        Serial.printf("  -> Decreasing gain to %d\n", settings.agcGain);
    }
    if (settings.aecValue < 800) {
        settings.aecValue = min(800, settings.aecValue + 100);
        Serial.printf("  -> Increasing exposure to %d\n", settings.aecValue);
    }
}

CameraSettings CameraOptimizer::generateNeighbor(const CameraSettings& current) {
    // Generate a small random adjustment
    CameraSettings neighbor = current;

    int adjustment = random(-1, 2);  // -1, 0, or 1
    int param = random(0, 4);  // Which parameter to adjust

    switch (param) {
        case 0:
            neighbor.brightness = clampBrightness(current.brightness + adjustment);
            break;
        case 1:
            neighbor.contrast = clampContrast(current.contrast + adjustment);
            break;
        case 2:
            neighbor.agcGain = clampGain(current.agcGain + adjustment * 3);
            break;
        case 3:
            neighbor.aecValue = clampExposure(current.aecValue + adjustment * 50);
            break;
    }

    return neighbor;
}

int CameraOptimizer::clampBrightness(int value) {
    return max(-2, min(2, value));
}

int CameraOptimizer::clampContrast(int value) {
    return max(-2, min(2, value));
}

int CameraOptimizer::clampGain(int value) {
    return max(0, min(30, value));
}

int CameraOptimizer::clampExposure(int value) {
    return max(0, min(1200, value));
}

void CameraOptimizer::setConvergenceThreshold(float threshold) {
    convergenceThreshold = threshold;
}

void CameraOptimizer::setQualityTarget(float target) {
    qualityTarget = target;
}

void CameraOptimizer::printSettings(const CameraSettings& settings) {
    Serial.println("=== Camera Settings ===");
    Serial.printf("Brightness:  %d\n", settings.brightness);
    Serial.printf("Contrast:    %d\n", settings.contrast);
    Serial.printf("Saturation:  %d\n", settings.saturation);
    Serial.printf("Sharpness:   %d\n", settings.sharpness);
    Serial.printf("AGC Gain:    %d\n", settings.agcGain);
    Serial.printf("AEC Value:   %d\n", settings.aecValue);
    Serial.printf("Flash:       %s\n", settings.flashEnabled ? "ON" : "OFF");
    Serial.println("=======================");
}
