// stdin: [uint32 kare_no][1280x720 mono8] tekrarlari. Her tespit: kare id hamming margin mesafe kenar_px egim_derece
// (node ile ayni ayarlar: add_family = 2 bit duzeltme, decimate 1, refine_edges, sharpening 1.0)
#include <apriltag/apriltag.h>
#include <apriltag/tag25h9.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/matd.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <vector>
int main() {
    const int W = 1280, H = 720; const double fx = 647.50, cx = 646.49, cy = 354.55, size = 0.26;
    apriltag_family_t *tf = tag25h9_create(); apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, 2);
    td->quad_decimate = 1.0f; td->quad_sigma = 0.0f; td->refine_edges = 1; td->decode_sharpening = 1.0; td->nthreads = 4;
    std::vector<uint8_t> buf(W * H); uint32_t fi;
    while (fread(&fi, 4, 1, stdin) == 1 && fread(buf.data(), 1, W * H, stdin) == (size_t)(W * H)) {
        image_u8_t im{W, H, W, buf.data()};
        zarray_t *d = apriltag_detector_detect(td, &im);
        for (int i = 0; i < zarray_size(d); ++i) {
            apriltag_detection_t *det; zarray_get(d, i, &det);
            apriltag_detection_info_t info{det, size, fx, fx, cx, cy}; apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);
            double t0 = MATD_EL(pose.t, 0, 0), t1 = MATD_EL(pose.t, 1, 0), t2 = MATD_EL(pose.t, 2, 0), s = 0;
            double tilt = std::acos(std::fabs(MATD_EL(pose.R, 2, 2))) * 180.0 / M_PI;   // tag normali - optik eksen
            for (int k = 0; k < 4; ++k) s += std::hypot(det->p[k][0] - det->p[(k + 1) % 4][0], det->p[k][1] - det->p[(k + 1) % 4][1]);
            printf("%u %d %d %.1f %.3f %.1f %.1f\n", fi, det->id, det->hamming, det->decision_margin,
                   std::sqrt(t0*t0 + t1*t1 + t2*t2), s / 4, tilt);
            free(pose.R); free(pose.t);
        }
        apriltag_detections_destroy(d);
    }
}
