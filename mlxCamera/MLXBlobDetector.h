#ifndef MLX_BLOB_DETECTOR_H
#define MLX_BLOB_DETECTOR_H

#include <stdint.h>

class MLXBlobDetector {
public:
    static const uint8_t kW = 32, kH = 24;
    static const uint16_t kPixels = kW * kH;
    static const uint8_t kMaxBlobs = 8;

    struct Blob {
        uint16_t area;
        float cx, cy;        // heat-weighted centroid, in pixels
        float peak;          // max (temp - background)
        uint8_t peakX, peakY;
    };

    void learnBackground(const float* temps);   // call for N frames with nobody present
    void update(const float* temps);            // threshold -> blobs -> background update
    uint8_t blobCount() const;
    const Blob& blob(uint8_t i) const;

private:
    void threshold(const float* temps);         // fills mask_
    void findBlobs(const float* temps);         // flood fill over mask_, fills blobs_
    void updateBackground(const float* temps);  // only where mask_ == 0

    float bg_[kPixels];
    uint8_t mask_[kPixels];
    uint8_t labels_[kPixels];
    uint16_t queue_[kPixels];                   // flood-fill work queue
    Blob blobs_[kMaxBlobs];
    uint8_t nBlobs_ = 0;
    float bgAlpha_ = 0.01f;
    float diffThreshold_ = 2.0f;
    uint16_t minArea_ = 4;
};

#endif