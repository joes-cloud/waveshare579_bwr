#include "waveshare579_bwr.h"

#include <algorithm>
#include <cinttypes>
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
    ESP_LOGE(TAG, "Framebuffer allocation failed (80784 bytes required)");
    this->mark_failed();
    return;
  }

  // Black plane: 1=white, 0=black. Red plane: 0=white, 1=red.
  std::memset(this->buffer_, 0xFF, PLANE_SIZE);
  std::memset(this->buffer_ + PLANE_SIZE, 0x00, PLANE_SIZE);
  std::memset(this->buffer_ + 2 * PLANE_SIZE, 0x00, PLANE_SIZE);
  // A display with update_interval: never must still render once after boot.
  this->set_timeout(100, [this]() { this->update(); });
}

void Waveshare579BWR::dump_config() {
  ESP_LOGCONFIG(TAG, "Waveshare 5.79in B/W/R e-paper:");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", WIDTH, HEIGHT);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  ESP_LOGCONFIG(TAG, "  Full refresh after %" PRIu32 " partial refreshes", this->full_update_every_);
  ESP_LOGCONFIG(TAG, "  Full refresh mode: %s", this->fast_refresh_ ? "fast" : "normal");
  LOG_UPDATE_INTERVAL(this);
}

void Waveshare579BWR::update() {
  ESP_LOGD(TAG, "Rendering framebuffer");
  this->refresh_handled_ = false;
  this->do_update_();
  if (!this->refresh_handled_)
    this->refresh_automatically_();
}

uint32_t Waveshare579BWR::red_plane_hash_() const {
  // FNV-1a is sufficient here: this is change detection, not authentication.
  uint32_t hash = 2166136261UL;
  const uint8_t *red_plane = this->buffer_ + PLANE_SIZE;
  for (size_t i = 0; i < PLANE_SIZE; i++) {
    hash ^= red_plane[i];
    hash *= 16777619UL;
  }
  return hash;
}

void Waveshare579BWR::refresh_automatically_() {
  if (!this->partial_basemap_ready_) {
    this->display();
    return;
  }

  const uint32_t red_hash = this->red_plane_hash_();
  if (red_hash != this->last_red_hash_) {
    ESP_LOGI(TAG, "Red plane changed; using full refresh");
    this->display();
    return;
  }

  const uint8_t *black_plane = this->buffer_;
  const uint8_t *previous_black = this->buffer_ + 2 * PLANE_SIZE;
  int min_byte = BYTES_PER_ROW;
  int max_byte = -1;
  int min_y = HEIGHT;
  int max_y = -1;

  for (int y = 0; y < HEIGHT; y++) {
    const size_t row = size_t(y) * BYTES_PER_ROW;
    for (int byte = 0; byte < BYTES_PER_ROW; byte++) {
      if (black_plane[row + byte] != previous_black[row + byte]) {
        min_byte = std::min(min_byte, byte);
        max_byte = std::max(max_byte, byte);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }
    }
  }

  if (max_byte < 0) {
    ESP_LOGD(TAG, "Framebuffer unchanged; skipping refresh");
    this->refresh_handled_ = true;
    return;
  }

  this->partial_refresh(min_byte * 8, min_y, (max_byte - min_byte + 1) * 8, max_y - min_y + 1);
}

void Waveshare579BWR::fill(Color color) {
  if (this->get_clipping().is_set()) {
    display::Display::fill(color);
    return;
  }
  const bool red = color.r > 127 && color.r > color.g * 2 && color.r > color.b * 2;
  const bool black = !red && (uint16_t(color.r) + color.g + color.b < 384);
  std::memset(this->buffer_, black ? 0x00 : 0xFF, PLANE_SIZE);
  std::memset(this->buffer_ + PLANE_SIZE, red ? 0xFF : 0x00, PLANE_SIZE);
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

  // Controller-native encoding: white=(1,0), black=(0,0), red=(1,1).
  black_plane[pos] = black ? (black_plane[pos] & ~mask) : (black_plane[pos] | mask);
  red_plane[pos] = red ? (red_plane[pos] | mask) : (red_plane[pos] & ~mask);
}

void Waveshare579BWR::display() {
  if (this->is_failed())
    return;

  const uint8_t *black_plane = this->buffer_;
  const uint8_t *red_plane = this->buffer_ + PLANE_SIZE;

  ESP_LOGI(TAG, "Writing black and red planes for %s refresh",
           this->fast_refresh_ ? "fast" : "normal");

  // The vendor requires re-initialization before every full-screen update.
  // Partial updates intentionally keep the existing controller state.
  this->init_display_();

  this->set_ram_slave_();
  this->write_half_(0xA4, black_plane, false);
  this->set_ram_master_();
  this->write_half_(0x24, black_plane, true);
  this->set_ram_slave_();
  this->write_half_(0xA6, red_plane, false);
  this->set_ram_master_();
  this->write_half_(0x26, red_plane, true);

  this->send_command_(0x22);
  this->send_data_(this->fast_refresh_ ? 0xC7 : 0xF7);
  this->send_command_(0x20);
  if (this->wait_busy_())
    ESP_LOGI(TAG, "Full refresh complete");

  // The SSD1683 uses the second RAM bank as the B/W reference image during
  // partial refresh. The visible red pixels have already been driven by now,
  // so replacing that RAM with the current black plane does not alter them.
  this->prepare_partial_basemap_();
  std::memcpy(this->buffer_ + 2 * PLANE_SIZE, black_plane, PLANE_SIZE);
  this->last_red_hash_ = this->red_plane_hash_();
  this->partial_basemap_ready_ = true;
  this->partial_refresh_count_ = 0;
  this->refresh_handled_ = true;
}

void Waveshare579BWR::partial_refresh(int x, int y, int width, int height) {
  this->refresh_handled_ = true;
  if (this->is_failed() || width <= 0 || height <= 0)
    return;

  if (!this->partial_basemap_ready_ || this->partial_refresh_count_ >= this->full_update_every_) {
    ESP_LOGI(TAG, "Partial baseline unavailable or refresh limit reached; using full refresh");
    this->display();
    return;
  }

  int x_end = x + width - 1;
  int y_end = y + height - 1;
  x = std::max(0, x);
  y = std::max(0, y);
  x_end = std::min(WIDTH - 1, x_end);
  y_end = std::min(HEIGHT - 1, y_end);
  if (x > x_end || y > y_end)
    return;

  // SSD1683 window coordinates are byte based. Extend to whole bytes.
  const int first_byte = x / 8;
  const int last_byte = x_end / 8;
  const uint8_t *black_plane = this->buffer_;

  ESP_LOGI(TAG, "Partial B/W refresh: x=%d, y=%d, w=%d, h=%d", x, y, x_end - x + 1,
           y_end - y + 1);

  // Power analog blocks for the partial update and keep the border unchanged.
  this->send_command_(0x3C);
  this->send_data_(0x80);
  this->send_command_(0x22);
  this->send_data_(0xC0);
  this->send_command_(0x20);
  if (!this->wait_busy_())
    return;

  if (first_byte <= 49) {
    const int slave_start = first_byte;
    const int slave_end = std::min(49, last_byte);
    this->set_window_slave_(slave_start, slave_end, y, y_end);
    this->write_window_(0xA4, black_plane, slave_start, slave_end, y, y_end);
  }

  if (last_byte >= 49) {
    const int master_start = std::max(49, first_byte);
    const int master_end = last_byte;
    this->set_window_master_(master_start, master_end, y, y_end);
    this->write_window_(0x24, black_plane, master_start, master_end, y, y_end);
  }

  this->send_command_(0x22);
  this->send_data_(0x1C);
  this->send_command_(0x20);
  if (this->wait_busy_()) {
    this->partial_refresh_count_++;
    uint8_t *previous_black = this->buffer_ + 2 * PLANE_SIZE;
    for (int row = y; row <= y_end; row++) {
      const size_t offset = size_t(row) * BYTES_PER_ROW + first_byte;
      std::memcpy(previous_black + offset, black_plane + offset, last_byte - first_byte + 1);
    }
    this->last_red_hash_ = this->red_plane_hash_();
    ESP_LOGI(TAG, "Partial refresh complete (%" PRIu32 "/%" PRIu32 ")", this->partial_refresh_count_,
             this->full_update_every_);
  }
}

void Waveshare579BWR::prepare_partial_basemap_() {
  const uint8_t *black_plane = this->buffer_;
  this->set_ram_slave_();
  this->write_half_(0xA6, black_plane, false);
  this->set_ram_master_();
  this->write_half_(0x26, black_plane, true);
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

void Waveshare579BWR::write_window_(uint8_t command, const uint8_t *plane, int byte_start,
                                    int byte_end, int y_start, int y_end) {
  this->send_command_(command);
  this->dc_pin_->digital_write(true);
  this->enable();
  for (int row = y_start; row <= y_end; row++) {
    for (int byte = byte_start; byte <= byte_end; byte++)
      this->write_byte(plane[size_t(row) * BYTES_PER_ROW + byte]);
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

void Waveshare579BWR::set_window_master_(int byte_start, int byte_end, int y_start, int y_end) {
  // Master X runs backwards; framebuffer byte 98 maps to controller address 0.
  const int address_start = 98 - byte_start;
  const int address_end = 98 - byte_end;
  this->send_command_(0x11);
  this->send_data_(0x02);
  this->send_command_(0x44);
  this->send_data_(address_start);
  this->send_data_(address_end);
  this->send_command_(0x45);
  this->send_data_(y_start & 0xFF);
  this->send_data_((y_start >> 8) & 0x01);
  this->send_data_(y_end & 0xFF);
  this->send_data_((y_end >> 8) & 0x01);
  this->send_command_(0x4E);
  this->send_data_(address_start);
  this->send_command_(0x4F);
  this->send_data_(y_start & 0xFF);
  this->send_data_((y_start >> 8) & 0x01);
}

void Waveshare579BWR::set_window_slave_(int byte_start, int byte_end, int y_start, int y_end) {
  this->send_command_(0x91);
  this->send_data_(0x03);
  this->send_command_(0xC4);
  this->send_data_(byte_start);
  this->send_data_(byte_end);
  this->send_command_(0xC5);
  this->send_data_(y_start & 0xFF);
  this->send_data_((y_start >> 8) & 0x01);
  this->send_data_(y_end & 0xFF);
  this->send_data_((y_end >> 8) & 0x01);
  this->send_command_(0xCE);
  this->send_data_(byte_start);
  this->send_command_(0xCF);
  this->send_data_(y_start & 0xFF);
  this->send_data_((y_start >> 8) & 0x01);
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
  this->send_command_(0x1A);  // Reference temperature selects normal/fast OTP waveform
  this->send_data_(this->fast_refresh_ ? 0x5A : 0x64);
  this->send_data_(0x00);
  this->send_command_(0x22);
  this->send_data_(0x91);
  this->send_command_(0x20);
  delay(10);
  this->wait_busy_();
  ESP_LOGI(TAG, "%s refresh initialization complete", this->fast_refresh_ ? "Fast" : "Normal");
}

}  // namespace waveshare579_bwr
}  // namespace esphome
