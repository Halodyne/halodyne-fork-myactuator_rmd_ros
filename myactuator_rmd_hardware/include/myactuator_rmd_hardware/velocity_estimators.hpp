#ifndef MYACTUATOR_RMD_HARDWARE__VELOCITY_ESTIMATORS
#define MYACTUATOR_RMD_HARDWARE__VELOCITY_ESTIMATORS
#pragma once

#include <atomic>
#include <cmath>


namespace myactuator_rmd_hardware {

  /// 1-Euro filter for velocity estimation from position measurements.
  ///
  /// Applies the 1-Euro adaptive low-pass filter (Casiez et al., CHI 2012) to
  /// the position signal, then differentiates the filtered output to obtain
  /// velocity. The adaptive cutoff provides smooth output at rest/low speed
  /// and responsive tracking during fast motion.
  ///
  /// Parameters:
  ///   min_cutoff: Minimum cutoff frequency (Hz). Lower = smoother at rest.
  ///   beta:       Speed coefficient. Higher = more responsive during motion.
  ///   d_cutoff:   Cutoff frequency for derivative estimation (Hz).
  ///
  /// Parameters are atomic so they can be updated at runtime from another thread.
  class OneEuroEstimator {
    public:
      OneEuroEstimator(double dt, double min_cutoff, double beta, double d_cutoff) noexcept
      : dt_{dt}, min_cutoff_{min_cutoff}, beta_{beta}, d_cutoff_{d_cutoff} {}

      void setMinCutoff(double min_cutoff) noexcept { min_cutoff_.store(min_cutoff, std::memory_order_relaxed); }
      void setBeta(double beta) noexcept { beta_.store(beta, std::memory_order_relaxed); }
      void setDCutoff(double d_cutoff) noexcept { d_cutoff_.store(d_cutoff, std::memory_order_relaxed); }

      double update(double z) noexcept {
        auto const min_cutoff = min_cutoff_.load(std::memory_order_relaxed);
        auto const beta = beta_.load(std::memory_order_relaxed);
        auto const d_cutoff = d_cutoff_.load(std::memory_order_relaxed);

        if (!initialized_) {
          x_prev_ = z;
          x_raw_prev_ = z;
          dx_prev_ = 0.0;
          filtered_prev_ = z;
          initialized_ = true;
          return 0.0;
        }

        // Raw derivative from consecutive raw samples
        double const dx = (z - x_raw_prev_) / dt_;

        // Filter the derivative with fixed cutoff
        double const a_d = alpha(d_cutoff);
        double const dx_hat = a_d * dx + (1.0 - a_d) * dx_prev_;

        // Adaptive cutoff for position filter
        double const fc = min_cutoff + beta * std::abs(dx_hat);

        // Filter the position
        double const a_s = alpha(fc);
        double const x_hat = a_s * z + (1.0 - a_s) * x_prev_;

        // Velocity = derivative of filtered position
        double const vel = (x_hat - filtered_prev_) / dt_;

        // Store state
        x_prev_ = x_hat;
        x_raw_prev_ = z;
        dx_prev_ = dx_hat;
        filtered_prev_ = x_hat;

        return vel;
      }

    private:
      double alpha(double cutoff) const noexcept {
        double const tau = 1.0 / (2.0 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / dt_);
      }

      double dt_;
      std::atomic<double> min_cutoff_;
      std::atomic<double> beta_;
      std::atomic<double> d_cutoff_;
      double x_prev_{0.0};
      double x_raw_prev_{0.0};
      double dx_prev_{0.0};
      double filtered_prev_{0.0};
      bool initialized_{false};
  };

}  // namespace myactuator_rmd_hardware

#endif  // MYACTUATOR_RMD_HARDWARE__VELOCITY_ESTIMATORS
