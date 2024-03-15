#include <libhal-pca/pca9685.hpp>

#include <array>
#include <cmath>

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/i2c.hpp>

namespace hal::pca {
namespace {
// NOTE: keeping this for future reference
[[maybe_unused]] static constexpr hal::byte mode1 = 0x00;
static constexpr hal::byte pwm_channel0_address = 0x06;
static constexpr hal::byte byte_per_pwm_channel = 4;
static constexpr hal::byte prescaler_address = 0xFE;
static constexpr float max_pwm_ticks = 4095.0f;
}  // namespace

pca9685::pwm_channel::pwm_channel(pca9685* p_pca9685, hal::byte p_channel)
  : m_pca9685(p_pca9685)
  , m_channel(p_channel)
{
}

void pca9685::pwm_channel::driver_frequency(hal::hertz p_frequency)
{
  m_pca9685->set_channel_frequency(p_frequency);
}

void pca9685::pwm_channel::driver_duty_cycle(float p_duty_cycle)
{
  m_pca9685->set_channel_duty_cycle(p_duty_cycle, m_channel);
}

constexpr hal::byte pwm_channel_address(hal::byte p_channel)
{
  return static_cast<hal::byte>(pwm_channel0_address +
                                (p_channel * byte_per_pwm_channel));
}

pca9685::pca9685(hal::i2c& p_i2c,
                 hal::byte p_address,
                 std::optional<pca9685::settings> p_settings)
  : m_i2c(&p_i2c)
  , m_address(p_address)
{
  pca9685::configure(p_settings.value_or(pca9685::settings{}));
}

void pca9685::configure(const pca9685::settings& p_settings)
{
  static constexpr hal::byte mode_address = 0U;

  // Mode 0 flags
  static constexpr auto enable_external_oscillator = bit_mask::from<6>();
  static constexpr auto auto_increment_address = bit_mask::from<5>();
  static constexpr auto sleep = bit_mask::from<4>();
  static constexpr auto sub1_enable = bit_mask::from<3>();
  static constexpr auto sub2_enable = bit_mask::from<2>();
  static constexpr auto sub3_enable = bit_mask::from<1>();
  static constexpr auto all_call_enable = bit_mask::from<0>();

  // Mode 1 flags
  static constexpr auto invert_logic = bit_mask::from<4>();
  static constexpr auto update_on_acknowledge = bit_mask::from<3>();
  static constexpr auto output_drive = bit_mask::from<2>();
  static constexpr auto output_enable_pin_state = bit_mask::from<1, 0>();

  auto mode1_byte = bit_value<hal::byte>(0)
                      .insert<enable_external_oscillator>(0U)
                      .insert<auto_increment_address>(1U)
                      .insert<sleep>(hal::byte(p_settings.sleep))
                      .insert<sub1_enable>(0U)
                      .insert<sub2_enable>(0U)
                      .insert<sub3_enable>(0U)
                      .insert<all_call_enable>(0U);

  auto mode2_byte =
    bit_value<hal::byte>(0)
      .insert<invert_logic>(hal::byte(p_settings.invert_outputs))
      .insert<update_on_acknowledge>(
        hal::byte(p_settings.output_changes_on_i2c_acknowledge))
      .insert<output_drive>(hal::byte(p_settings.totem_pole_output))
      .insert<output_enable_pin_state>(
        hal::value(p_settings.pin_disabled_state));

  hal::write(*m_i2c,
             m_address,
             std::array{ mode_address, mode1_byte.get(), mode2_byte.get() },
             hal::never_timeout());

  m_settings = p_settings;
}

void pca9685::set_channel_frequency(hal::hertz p_frequency)
{
  using namespace hal::literals;
  static constexpr auto internal_oscillator = 25.0_MHz;
  bool within_bounds = 24.0_Hz < p_frequency && p_frequency < 1526.0_Hz;
  if (!within_bounds) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  auto prescale_value =
    std::round(internal_oscillator / (max_pwm_ticks * p_frequency));
  auto prescale_value_byte = static_cast<hal::byte>(prescale_value - 1.0f);

  // The device must be put to sleep before it can have its prescale value
  // updated.
  auto original_settings = m_settings;
  // Ensure that the settings put the device to sleep.
  m_settings.sleep = true;
  configure(m_settings);

  hal::write(*m_i2c,
             m_address,
             std::array{ prescaler_address, prescale_value_byte },
             hal::never_timeout());

  // Configure device back to what it was before which may or may not be asleep
  configure(original_settings);
}

// NOLINTNEXTLINE
void pca9685::set_channel_duty_cycle(float p_duty_cycle, hal::byte p_channel)
{
  // The PCA9685 works by setting a HIGH point and a LOW point out of the 12-bit
  // timer cycle. This driver implements LEFT aligned PWM for all channels, and
  // thus, the point in which the pulse should go HIGH should be at the start at
  // position 0. The point in which the PWM duty cycle is pulled low is
  // calculated based on the passed in p_duty_cycle value.
  static constexpr hal::byte high_point_msb_lsb = 0U;

  auto low_point_float = std::round(max_pwm_ticks * p_duty_cycle);
  auto low_point = static_cast<std::uint16_t>(low_point_float);
  hal::byte low_point_lsb = low_point & 0xFF;
  hal::byte low_point_msb = low_point >> 8;

  hal::write(*m_i2c,
             m_address,
             std::array{ pwm_channel_address(p_channel),
                         high_point_msb_lsb,
                         high_point_msb_lsb,
                         low_point_lsb,
                         low_point_msb },
             hal::never_timeout());
}
}  // namespace hal::pca
