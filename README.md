# ESP LCD Touch XPT2046 Driver

XPT2046 resistive touch controller driver for ESP-IDF.

## Features

- **SPI interface support** using `spi_device_handle_t` for proper device management
- **Single touch point detection** with 12-bit ADC resolution
- **Configurable calibration** (X/Y min/max ranges)
- **Sample averaging** using best-2-of-3 algorithm for noise reduction
- **Pressure detection** with configurable Z threshold (default: 400)
- **Coordinate transformation** (swap XY, mirror X/Y)
- **NULL-safe API** - prevents crashes when reading coordinates
- **Polling mode support** for input-only GPIOs (no interrupt required)
- **Compatible with esp_lcd_touch API** for seamless LVGL integration

## Hardware

- **Interface**: SPI (2MHz, Mode 0, MSB first)
- **Resolution**: 12-bit ADC (0-4095 range)
- **Touch points**: Single touch
- **Interrupt**: Optional IRQ pin support (polling mode available for input-only GPIOs)
- **Power**: 3.3V compatible
- **Tested on**: ESP32-2432S028 (Cheap Yellow Display)

## Usage

### Basic Setup with SPI Device Handle

```c
#include "esp_lcd_touch_xpt2046.h"
#include "driver/spi_master.h"

// Configure SPI bus (if not already initialized)
spi_bus_config_t bus_cfg = {
    .mosi_io_num = GPIO_NUM_32,
    .miso_io_num = GPIO_NUM_39,
    .sclk_io_num = GPIO_NUM_25,
    .quadwp_io_num = GPIO_NUM_NC,
    .quadhd_io_num = GPIO_NUM_NC,
    .max_transfer_sz = 0,
};
spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED);

// Configure SPI device for XPT2046
spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = 2 * 1000 * 1000,  // 2MHz
    .mode = 0,
    .spics_io_num = GPIO_NUM_33,
    .queue_size = 1,
    .flags = SPI_DEVICE_HALFDUPLEX,
};

spi_device_handle_t spi_device;
spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device);

// Configure touch controller
esp_lcd_touch_config_t touch_cfg = {
    .x_max = 240,
    .y_max = 320,
    .rst_gpio_num = GPIO_NUM_NC,
    .int_gpio_num = GPIO_NUM_NC,  // Use GPIO_NUM_NC for polling mode
    .levels = {
        .reset = 0,
        .interrupt = 0,
    },
    .flags = {
        .swap_xy = false,
        .mirror_x = false,
        .mirror_y = false,
    },
};

esp_lcd_touch_handle_t touch_handle;
esp_lcd_touch_new_spi_xpt2046(spi_device, &touch_cfg, &touch_handle);

// Set calibration (optional, adjust to your display)
xpt2046_calibration_t cal = {
    .x_min = 200,
    .x_max = 3700,
    .y_min = 240,
    .y_max = 3800,
};
esp_lcd_touch_xpt2046_set_calibration(touch_handle, &cal);
```

### Reading Touch Data

```c
uint16_t x[1], y[1];
uint8_t points;

// Read touch coordinates
esp_lcd_touch_read_data(touch_handle);
bool pressed = esp_lcd_touch_get_coordinates(touch_handle, x, y, NULL, &points, 1);

if (pressed && points > 0) {
    printf("Touch at X=%d, Y=%d\n", x[0], y[0]);
}
```

## Configuration

### Menuconfig Options

- `LCD_TOUCH_XPT2046_ENABLE_CALIBRATION`: Enable calibration feature
- `LCD_TOUCH_XPT2046_SAMPLES`: Number of samples for averaging (1-10, default 3)
- `LCD_TOUCH_XPT2046_Z_THRESHOLD`: Touch pressure threshold (0-4095, default 400)

### Calibration

The driver supports custom calibration to map raw ADC values to screen coordinates:

```c
xpt2046_calibration_t cal = {
    .x_min = 200,   // Raw X value at screen left edge
    .x_max = 3900,  // Raw X value at screen right edge
    .y_min = 200,   // Raw Y value at screen top edge
    .y_max = 3900,  // Raw Y value at screen bottom edge
};
esp_lcd_touch_xpt2046_set_calibration(touch_handle, &cal);
```

**Calibration tips:**
- Touch corners of your display and note raw values in logs
- Default range is typically 200-3900, but varies by display
- Use `ESP_LOGI` in driver to see raw values before calibration
- Adjust `x_min`, `x_max`, `y_min`, `y_max` to match your touchscreen

## Typical Wiring (ESP32-2432S028)

| XPT2046 Pin | ESP32 GPIO | Description              |
|-------------|------------|--------------------------|
| T_CLK       | GPIO25     | SPI Clock (SCLK)        |
| T_CS        | GPIO33     | Chip Select (CS)        |
| T_DIN       | GPIO32     | MOSI (Master Out)       |
| T_DO        | GPIO39     | MISO (Master In)        |
| T_IRQ       | GPIO36     | Interrupt (optional)    |
| VCC         | 3.3V       | Power supply            |
| GND         | GND        | Ground                  |

**Notes:**
- GPIO39 and GPIO36 are **input-only** on ESP32 classic - use polling mode
- Use separate SPI bus (SPI2_HOST) if LCD is on different bus (SPI3_HOST)
- IRQ pin can be set to `GPIO_NUM_NC` for polling mode

## Advanced Features

### NULL-Safe API
The driver includes NULL pointer checks to prevent crashes:
```c
uint16_t x[1], y[1];
// Safe to call even if touch_handle is NULL or not touched
esp_lcd_touch_get_coordinates(touch_handle, x, y, NULL, &points, 1);
```

### Sample Averaging
Uses **best-2-of-3** algorithm to reduce noise:
- Takes 3 samples of X, Y, and Z coordinates
- Discards the outlier sample
- Averages the two closest values
- Configurable via Kconfig (1-10 samples)

### Pressure Detection
Measures touch pressure using Z1/Z2 resistance:
- Z threshold configurable (default: 400)
- Higher Z value = harder press
- Returns `false` if pressure below threshold

## Troubleshooting

**Touch not detected:**
- Check SPI wiring and bus configuration
- Verify CS pin is correct and not shared
- Ensure SPI bus is initialized before creating touch device
- Try increasing Z threshold if too sensitive

**Incorrect coordinates:**
- Adjust calibration values (`x_min`, `x_max`, `y_min`, `y_max`)
- Check `swap_xy`, `mirror_x`, `mirror_y` flags
- Verify `x_max` and `y_max` match your display resolution

**Noisy/jittery touch:**
- Increase sample count in Kconfig (default: 3)
- Lower Z threshold for better pressure detection
- Check for electrical noise on SPI lines

## License

Apache-2.0
