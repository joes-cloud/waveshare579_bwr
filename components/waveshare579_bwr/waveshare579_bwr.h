#pragma once

#include "esphome/components/display/display_buffer.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace waveshare579_bwr {

class Waveshare579BWR
    : public display::DisplayBuffer,
      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                            spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  void set_dc_pin(GPIOPin *pin) { this->dc_pin_ = pin; }
  void set_busy_pin(GPIOPin *pin) { this->busy_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_full_update_every(uint32_t full_update_every) { this->full_update_every_ = full_update_every; }
  void set_fast_refresh(bool fast_refresh) { this->fast_refresh_ = fast_refresh; }

  void setup() override;
  void update() override;
  void dump_config() override;
  void fill(Color color) override;
  void display();
  void partial_refresh(int x, int y, int width, int height);

  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  int get_width_internal() override { return WIDTH; }
  int get_height_internal() override { return HEIGHT; }
  display::DisplayType get_display_type() override {
    return display::DisplayType::DISPLAY_TYPE_COLOR;
  }

 protected:
  static constexpr int WIDTH = 792;
  static constexpr int HEIGHT = 272;
  static constexpr size_t BYTES_PER_ROW = WIDTH / 8;
  static constexpr size_t PLANE_SIZE = BYTES_PER_ROW * HEIGHT;

  size_t get_buffer_length_() { return 3 * PLANE_SIZE; }
  void draw_absolute_pixel_internal(int x, int y, Color color) override;

  void reset_();
  bool wait_busy_();
  void send_command_(uint8_t command);
  void send_data_(uint8_t data);
  void init_display_();
  void set_ram_master_();
  void set_ram_slave_();
  void set_window_master_(int byte_start, int byte_end, int y_start, int y_end);
  void set_window_slave_(int byte_start, int byte_end, int y_start, int y_end);
  void write_half_(uint8_t command, const uint8_t *plane, bool master);
  void write_window_(uint8_t command, const uint8_t *plane, int byte_start, int byte_end,
                     int y_start, int y_end);
  void prepare_partial_basemap_();
  void refresh_automatically_();
  uint32_t red_plane_hash_() const;

  GPIOPin *dc_pin_{nullptr};
  GPIOPin *busy_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  bool refresh_handled_{false};
  bool partial_basemap_ready_{false};
  bool fast_refresh_{false};
  uint32_t partial_refresh_count_{0};
  uint32_t full_update_every_{5};
  uint32_t last_red_hash_{0};
};

}  // namespace waveshare579_bwr
}  // namespace esphome
