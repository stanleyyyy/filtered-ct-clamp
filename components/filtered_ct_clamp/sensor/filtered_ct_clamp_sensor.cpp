#include "filtered_ct_clamp_sensor.h"

#include "esphome/core/log.h"
#include <cinttypes>
#include <cmath>

namespace esphome {
namespace filtered_ct_clamp {

static const char *const TAG = "ct_clamp_filtered";

void CTClampFilteredSensor::dump_config() {
  LOG_SENSOR("", "CT Clamp Filtered Sensor", this);
  ESP_LOGCONFIG(TAG, "  Sample Duration: %.2fs", this->sample_duration_ / 1e3f);
  LOG_UPDATE_INTERVAL(this);
}

void CTClampFilteredSensor::update() {
  // Update only starts the sampling phase, in loop() the actual sampling is happening.

  // Request a high loop() execution interval during sampling phase.
  this->high_freq_.start();

  // Set timeout for ending sampling phase
  this->set_timeout("read", this->sample_duration_, [this]() {
    this->is_sampling_ = false;
    this->high_freq_.stop();

    if (this->num_samples_ == 0) {
      // Shouldn't happen, but let's not crash if it does.
      ESP_LOGD(TAG, "'%s' - zero samples retrieved, returning NAN", this->name_.c_str());
      this->publish_state(NAN);
      return;
    }

    const float rms_ac_dc_squared = this->sample_squared_sum_ / this->num_samples_;
    const float rms_dc = this->sample_sum_ / this->num_samples_;
    const float rms_diff = (rms_ac_dc_squared > rms_dc * rms_dc) ? (rms_ac_dc_squared - rms_dc * rms_dc) : 0; // we have to be sure the final value is above 0 to avoid sqrt issues!
    const float rms_ac = std::sqrt(rms_diff);

    // disable check for minimum value as it seems integration integration in HA can't handle 0 properly
    // instead of providing 0A, we will give it 10x smaller value
    if (rms_ac < MIN_CURRENT) {
      ESP_LOGD(TAG, "'%s' - Raw AC Value: %.3f A after %" PRIu32 " different samples (%" PRIu32 " SPS). AC value is too low, forcing to %.3f A", this->name_.c_str(), rms_ac, this->num_samples_, 1000 * this->num_samples_ / this->sample_duration_, rms_ac / 10);
      this->publish_state(rms_ac / 10);
    }
    else
    {
      ESP_LOGD(TAG, "'%s' - Raw AC Value: %.3f A after %" PRIu32 " different samples (%" PRIu32 " SPS)", this->name_.c_str(), rms_ac, this->num_samples_, 1000 * this->num_samples_ / this->sample_duration_);
      this->publish_state(rms_ac);
    }
  });

  // Set sampling values
  this->last_value_ = 0.0;
  this->num_samples_ = 0;
  this->sample_sum_ = 0.0f;
  this->sample_squared_sum_ = 0.0f;
  this->is_sampling_ = true;
}

void CTClampFilteredSensor::loop() {
  if (!this->is_sampling_)
    return;

  if (!this->source_)
    return;

  // Perform a single sample
  float value = this->source_->sample();
  if (std::isnan(value))
    return;

  // Assuming a sine wave, avoid requesting values faster than the ADC can provide them
  if (this->last_value_ == value)
    return;

  this->last_value_ = value;

  // apply low-pass filter
  value = lowPassFilter(value);

  this->num_samples_++;
  this->sample_sum_ += value;
  this->sample_squared_sum_ += value * value;
}

double CTClampFilteredSensor::lowPassFilter(double input) {
  static double RC = 1.0 / (CUTOFF * 2 * 3.1415);
  static double dt = 1.0 / SAMPLE_RATE;
  static double alpha = dt / (RC + dt);
  static double g_prevOutput = 0;

  double output = g_prevOutput + (alpha * (input - g_prevOutput));
  g_prevOutput = output;
  return output;
}

}  // namespace filtered_ct_clamp
}  // namespace esphome
