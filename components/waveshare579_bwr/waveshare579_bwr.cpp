#include "waveshare579_bwr.h"

#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waveshare579_bwr {

static const char *const TAG = "waveshare579_bwr";
static constexpr size_t HALF_BYTES_PER_ROW = 50;

void Waveshare579BWR::setup() {
  this->dc_pin_->setup();
  this->busy_pin_->setup();
  this->reset_pin_->setup();
  this->spi_setup();

  this->init_internal_(this->get_buffer_length_());
  if (this->buffer_ == nullptr) {
    ESP_LOGE(TAG, "Framebuffer allocation failed (53856 bytes required)");
    this->mark_failed();
    return;
  }

  // Both planes use 1 for paper white. The second plane starts at PLANE_SIZE.
  std::memset(this->buffer_, 0xFF, this->get_buffer_length_());
  this->init_display_();
}

void Waveshare579BWR::dump_config() {
  ESP_LOGCONFIG(TAG, "Waveshare 5.79in B/W/R e-paper:");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", WIDTH, HEIGHT);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}

void Waveshare579BWR::update() { this->do_update_(); }

void Waveshare579BWR::fill(Color color) {
  if (this->get_clipping().is_set()) {
    display::Display::fill(color);
    return;
  }
  const bool red = color.r > 127 && color.r > color.g * 2 && color.r > color.b * 2;
  const bool black = !red && (uint16_t(color.r) + color.g + color.b < 384);
  std::memset(this->buffer_, black ? 0x00 : 0xFF, PLANE_SIZE);
  std::memset(this->buffer_ + PLANE_SIZE, red ? 0x00 : 0xFF, PLANE_SIZE);
}

void Waveshare579BWR::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
    return;

  const size_t pos = size_t(y) * BYTES_PER_ROW + size_t(x / 8);
  const uint8_t mask = uint8_t(0x80 >> (x & 7));
  uint8_t *black_plane = this->buffer_;
  uint8_t *red_plane = this->buffer_ + PLANE_SIZE;

  const bool red = color.r > 127 && color.r > color.g * 2 && color.r > color.b * 2;
  const bool black = !red && (uint16_t(color.r) + color.g + color.b < 384);

  // White=(1,1), black=(0,1), red=(1,0).
  black_plane[pos] = black ? (black_plane[pos] & ~mask) : (black_plane[pos] | mask);
  red_plane[pos] = red ? (red_plane[pos] & ~mask) : (red_plane[pos] | mask);
}

void Waveshare579BWR::display() {
  if (this->is_failed())
    return;

  const uint8_t *black_plane = this->buffer_;
  const uint8_t *red_plane = this->buffer_ + PLANE_SIZE;

  this->set_ram_slave_();
  this->write_half_(0xA4, black_plane, false);
  this->set_ram_master_();
  this->write_half_(0x24, black_plane, true);
  this->set_ram_slave_();
  this->write_half_(0xA6, red_plane, false);
  this->set_ram_master_();
  this->write_half_(0x26, red_plane, true);

  this->send_command_(0x22);
  this->send_data_(0xF7);
  this->send_command_(0x20);
  this->wait_busy_();
}

void Waveshare579BWR::write_half_(uint8_t command, const uint8_t *plane, bool master) {
  this->send_command_(command);
  this->dc_pin_->digital_write(true);
  this->enable();
  for (size_t y = 0; y < HEIGHT; y++) {
    const size_t row = y * BYTES_PER_ROW;
    const size_t first = master ? HALF_BYTES_PER_ROW - 1 : 0;
    for (size_t b = 0; b < HALF_BYTES_PER_ROW; b++)
      this->write_byte(plane[row + first + b]);
    if ((y & 31) == 0)
      App.feed_wdt();
  }
  this->disable();
}

void Waveshare579BWR::reset_() {
  this->reset_pin_->digital_write(true);
  delay(10);
  this->reset_pin_->digital_write(false);
  delay(10);
  this->reset_pin_->digital_write(true);
  delay(10);
  this->wait_busy_();
}

bool Waveshare579BWR::wait_busy_() {
  const uint32_t started = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - started > 60000) {
      ESP_LOGE(TAG, "BUSY timeout after 60 seconds");
      this->status_set_warning();
      return false;
    }
    App.feed_wdt();
    delay(10);
  }
  this->status_clear_warning();
  return true;
}

void Waveshare579BWR::send_command_(uint8_t command) {
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(command);
  this->disable();
}

void Waveshare579BWR::send_data_(uint8_t data) {
  this->dc_pin_->digital_write(true);
  this->enable();
  this->write_byte(data);
  this->disable();
}

void Waveshare579BWR::set_ram_master_() {
  this->send_command_(0x11);
  this->send_data_(0x02);  // X decrement, Y increment
  this->send_command_(0x44);
  this->send_data_(0x31);
  this->send_data_(0x00);
  this->send_command_(0x45);
  this->send_data_(0x00);
  this->send_data_(0x00);
  this->send_data_(0x0F);
  this->send_data_(0x01);
  this->send_command_(0x4E);
  this->send_data_(0x31);
  this->send_command_(0x4F);
  this->send_data_(0x00);
  this->send_data_(0x00);
}

void Waveshare579BWR::set_ram_slave_() {
  this->send_command_(0x91);
  this->send_data_(0x03);  // X increment, Y increment
  this->send_command_(0xC4);
  this->send_data_(0x00);
  this->send_data_(0x31);
  this->send_command_(0xC5);
  this->send_data_(0x00);
  this->send_data_(0x00);
  this->send_data_(0x0F);
  this->send_data_(0x01);
  this->send_command_(0xCE);
  this->send_data_(0x00);
  this->send_command_(0xCF);
  this->send_data_(0x00);
  this->send_data_(0x00);
}

void Waveshare579BWR::init_display_() {
  this->reset_();
  this->send_command_(0x12);  // Software reset
  delay(10);
  this->wait_busy_();
  this->send_command_(0x18);  // Internal temperature sensor
  this->send_data_(0x80);
  this->send_command_(0x22);  // Load temperature and three-color OTP LUT
  this->send_data_(0xB1);
  this->send_command_(0x20);
  delay(10);
  this->wait_busy_();
  this->send_command_(0x1A);  // Reference temperature used by vendor code
  this->send_data_(0x64);
  this->send_data_(0x00);
  this->send_command_(0x22);
  this->send_data_(0xB1);
  this->send_command_(0x20);
  delay(10);
  this->wait_busy_();
  ESP_LOGI(TAG, "Initialization complete");
}

}  // namespace waveshare579_bwr
}  // namespace esphome
