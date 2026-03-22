#pragma once

class LowPassFilter {
public:
  explicit LowPassFilter(double alpha = 0.3) : alpha_(alpha) {}

  double update(double x) {
    v_ = alpha_ * x + (1.0 - alpha_) * v_;
    return v_;
  }

  void reset(double v = 0.0) { v_ = v; }

private:
  double alpha_;
  double v_{0.0};
};
