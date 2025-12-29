# RobotHat.jl - Julia interface for SunFounder Robot HAT
# Direct I2C/GPIO access via Linux kernel interfaces (no daemon required)
#
# Usage:
#   include("RobotHat.jl")
#   using .RobotHat

module RobotHat

export I2CDevice, PWMChannel, ADCChannel, Servo, Motor, Pin
export read_byte, write_byte, mem_read, mem_write
export freq!, pulse_width!, pulse_width_percent!
export angle!, read_voltage, speed!

# =============================================================================
# Constants
# =============================================================================

const I2C_BUS = 1
const I2C_ADDRESSES = [0x14, 0x15, 0x16]  # MCU addresses to try
const PWM_CLOCK = 72_000_000.0

# ioctl constants for I2C
const I2C_SLAVE = 0x0703
const I2C_SMBUS = 0x0720
const I2C_RDWR  = 0x0707

# I2C_SMBUS read/write
const I2C_SMBUS_READ  = 1
const I2C_SMBUS_WRITE = 0

# I2C_SMBUS transaction types
const I2C_SMBUS_BYTE      = 1
const I2C_SMBUS_BYTE_DATA = 2
const I2C_SMBUS_WORD_DATA = 3
const I2C_SMBUS_BLOCK_DATA = 5
const I2C_SMBUS_I2C_BLOCK_DATA = 8

# PWM Register Map
const REG_CHN = 0x20      # Channel registers start (P0-P19)
const REG_PSC = 0x40      # Prescaler registers (timers 0-3)
const REG_ARR = 0x44      # Period registers (timers 0-3)
const REG_PSC2 = 0x50     # Prescaler registers (timers 4-6)
const REG_ARR2 = 0x54     # Period registers (timers 4-6)

# ADC Register Map
const REG_ADC_BASE = 0x10  # ADC channel base (0x10-0x17)

# GPIO Pin Mapping (BCM numbers)
const PIN_MAP = Dict(
    "D0" => 17, "D1" => 4,  "D2" => 27, "D3" => 22,
    "D4" => 23, "D5" => 24, "D6" => 25, "D9" => 6,
    "D10" => 12, "D11" => 13, "D12" => 19, "D13" => 16,
    "D14" => 26, "D15" => 20, "D16" => 21,
    "SW" => 25, "USER" => 25, "LED" => 26,
    "RST" => 16, "MCURST" => 5,
)

# =============================================================================
# Low-level I2C via /dev/i2c-*
# =============================================================================

mutable struct I2CDevice
    fd::Cint
    address::UInt8
    retries::Int
end

"""
    I2CDevice(; addresses=I2C_ADDRESSES, bus=I2C_BUS, retries=5)

Open I2C connection to Robot HAT MCU via /dev/i2c-{bus}.
Scans addresses to find responding device.
"""
function I2CDevice(; addresses=I2C_ADDRESSES, bus=I2C_BUS, retries=5)
    devpath = "/dev/i2c-$bus"
    fd = ccall(:open, Cint, (Cstring, Cint), devpath, 2)  # O_RDWR = 2
    if fd < 0
        error("Failed to open $devpath - check permissions (try: sudo chmod 666 $devpath)")
    end

    # Try each address until one works
    for addr in addresses
        if _set_address(fd, addr)
            # Test if device responds with a simple read
            buf = Vector{UInt8}(undef, 1)
            result = ccall(:read, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), fd, buf, 1)
            if result >= 0
                return I2CDevice(fd, UInt8(addr), retries)
            end
        end
    end

    # Default to first address if none respond
    _set_address(fd, addresses[1])
    return I2CDevice(fd, UInt8(addresses[1]), retries)
end

function _set_address(fd::Cint, address::Integer)::Bool
    result = ccall(:ioctl, Cint, (Cint, Culong, Cint), fd, I2C_SLAVE, address)
    return result >= 0
end

function Base.close(dev::I2CDevice)
    if dev.fd >= 0
        ccall(:close, Cint, (Cint,), dev.fd)
        dev.fd = -1
    end
end

"""Retry wrapper for I2C operations"""
function with_retry(f, dev::I2CDevice)
    for attempt in 1:dev.retries
        try
            result = f()
            if result !== nothing && result !== false
                return result
            end
        catch e
            if attempt == dev.retries
                rethrow(e)
            end
        end
        sleep(0.001)  # Small delay before retry
    end
    return nothing
end

# --- Raw I2C read/write ---

function _raw_write(dev::I2CDevice, data::Vector{UInt8})
    n = ccall(:write, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), dev.fd, data, length(data))
    return n == length(data)
end

function _raw_read(dev::I2CDevice, count::Int)::Vector{UInt8}
    buf = Vector{UInt8}(undef, count)
    n = ccall(:read, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), dev.fd, buf, count)
    if n != count
        error("I2C read failed: expected $count bytes, got $n")
    end
    return buf
end

# --- SMBus-style operations ---

"""Write a single byte"""
function write_byte(dev::I2CDevice, data::UInt8)
    with_retry(dev) do
        _raw_write(dev, UInt8[data])
    end
end

"""Write byte to register"""
function write_byte_data(dev::I2CDevice, reg::UInt8, data::UInt8)
    with_retry(dev) do
        _raw_write(dev, UInt8[reg, data])
    end
end

"""Write 16-bit word to register (little-endian, SMBus style)"""
function write_word_data(dev::I2CDevice, reg::UInt8, data::UInt16)
    with_retry(dev) do
        _raw_write(dev, UInt8[reg, data & 0xFF, (data >> 8) & 0xFF])
    end
end

"""Write block of bytes to register"""
function write_block(dev::I2CDevice, reg::UInt8, data::Vector{UInt8})
    with_retry(dev) do
        _raw_write(dev, vcat(UInt8[reg], data))
    end
end

"""Read a single byte"""
function read_byte(dev::I2CDevice)::UInt8
    result = with_retry(dev) do
        _raw_read(dev, 1)
    end
    return result[1]
end

"""Read byte from register"""
function read_byte_data(dev::I2CDevice, reg::UInt8)::UInt8
    with_retry(dev) do
        _raw_write(dev, UInt8[reg])
    end
    result = with_retry(dev) do
        _raw_read(dev, 1)
    end
    return result[1]
end

"""Read 16-bit word from register (little-endian, SMBus style)"""
function read_word_data(dev::I2CDevice, reg::UInt8)::UInt16
    with_retry(dev) do
        _raw_write(dev, UInt8[reg])
    end
    result = with_retry(dev) do
        _raw_read(dev, 2)
    end
    return UInt16(result[1]) | (UInt16(result[2]) << 8)
end

"""Read block of bytes from register"""
function read_block(dev::I2CDevice, reg::UInt8, count::Int)::Vector{UInt8}
    with_retry(dev) do
        _raw_write(dev, UInt8[reg])
    end
    with_retry(dev) do
        _raw_read(dev, count)
    end
end

"""Write data to memory address (register)"""
function mem_write(dev::I2CDevice, memaddr::UInt8, data::Vector{UInt8})
    write_block(dev, memaddr, data)
end

"""Read data from memory address (register)"""
function mem_read(dev::I2CDevice, memaddr::UInt8, length::Int)::Vector{UInt8}
    read_block(dev, memaddr, length)
end

# =============================================================================
# GPIO via /sys/class/gpio (sysfs interface)
# =============================================================================

mutable struct Pin
    gpio::Int
    exported::Bool
    direction::Symbol  # :in or :out
end

const GPIO_PATH = "/sys/class/gpio"

"""
    Pin(pin; mode=:out)

Create GPIO pin. `pin` can be BCM number or name like "D4".
"""
function Pin(pin; mode::Symbol=:out)
    gpio = pin isa Integer ? pin : PIN_MAP[pin]

    # Export the pin if not already exported
    pin_path = joinpath(GPIO_PATH, "gpio$gpio")
    exported = isdir(pin_path)

    if !exported
        try
            write(joinpath(GPIO_PATH, "export"), string(gpio))
            sleep(0.1)  # Give kernel time to create files
            exported = true
        catch e
            @warn "Failed to export GPIO $gpio: $e"
        end
    end

    p = Pin(gpio, exported, :out)
    direction!(p, mode)
    return p
end

function direction!(pin::Pin, dir::Symbol)
    @assert dir in (:in, :out) "Direction must be :in or :out"
    pin.direction = dir
    try
        write(joinpath(GPIO_PATH, "gpio$(pin.gpio)", "direction"), dir == :in ? "in" : "out")
    catch
    end
end

"""Set pin value (0 or 1)"""
function value!(pin::Pin, val::Integer)
    if pin.direction != :out
        direction!(pin, :out)
    end
    write(joinpath(GPIO_PATH, "gpio$(pin.gpio)", "value"), val > 0 ? "1" : "0")
end

"""Read pin value"""
function value(pin::Pin)::Int
    if pin.direction != :in
        direction!(pin, :in)
    end
    s = read(joinpath(GPIO_PATH, "gpio$(pin.gpio)", "value"), String)
    return parse(Int, strip(s))
end

on!(pin::Pin) = value!(pin, 1)
off!(pin::Pin) = value!(pin, 0)

function Base.close(pin::Pin)
    if pin.exported
        try
            write(joinpath(GPIO_PATH, "unexport"), string(pin.gpio))
        catch
        end
    end
end

# =============================================================================
# PWM Channel
# =============================================================================

# Global timer state (shared across PWM channels like in Python version)
const TIMER_ARR = fill(1, 7)

mutable struct PWMChannel
    dev::I2CDevice
    channel::Int
    timer_index::Int
    prescaler::Int
    pulse_width::Int
    freq::Float64
end

"""
    PWMChannel(dev::I2CDevice, channel)

Create PWM channel (0-19 or "P0"-"P19"). Initializes at 50Hz.
"""
function PWMChannel(dev::I2CDevice, channel::Int)
    @assert 0 <= channel <= 19 "Channel must be 0-19"

    # Determine timer index
    timer_index = if channel < 16
        div(channel, 4)
    elseif channel in (16, 17)
        4
    elseif channel == 18
        5
    else
        6
    end

    pwm = PWMChannel(dev, channel, timer_index, 1, 0, 50.0)
    freq!(pwm, 50.0)
    return pwm
end

"""Parse channel string like "P0" to integer"""
function parse_channel(s::AbstractString)::Int
    if startswith(s, "P")
        parse(Int, s[2:end])
    else
        error("Channel string must start with 'P', got: $s")
    end
end

PWMChannel(dev::I2CDevice, channel::AbstractString) = PWMChannel(dev, parse_channel(channel))

function _i2c_write_16(pwm::PWMChannel, reg::UInt8, value::Int)
    # Big-endian for the Robot HAT MCU (different from SMBus!)
    value_h = UInt8((value >> 8) & 0xFF)
    value_l = UInt8(value & 0xFF)
    write_block(pwm.dev, reg, UInt8[value_h, value_l])
end

"""Set PWM frequency in Hz"""
function freq!(pwm::PWMChannel, freq::Real)
    pwm.freq = Float64(freq)

    # Find optimal prescaler/period combination (same algorithm as Python)
    st = max(1, round(Int, sqrt(PWM_CLOCK / freq)) - 5)

    best_psc, best_arr, best_error = 1, 1, Inf
    for psc in st:(st+9)
        arr = round(Int, PWM_CLOCK / freq / psc)
        error = abs(freq - PWM_CLOCK / psc / arr)
        if error < best_error
            best_psc, best_arr, best_error = psc, arr, error
        end
    end

    prescaler!(pwm, best_psc)
    period!(pwm, best_arr)
end

"""Set prescaler value"""
function prescaler!(pwm::PWMChannel, psc::Int)
    pwm.prescaler = psc
    pwm.freq = PWM_CLOCK / psc / TIMER_ARR[pwm.timer_index + 1]

    reg = if pwm.timer_index < 4
        UInt8(REG_PSC + pwm.timer_index)
    else
        UInt8(REG_PSC2 + pwm.timer_index - 4)
    end
    _i2c_write_16(pwm, reg, psc - 1)
end

"""Set period (ARR) value"""
function period!(pwm::PWMChannel, arr::Int)
    TIMER_ARR[pwm.timer_index + 1] = arr
    pwm.freq = PWM_CLOCK / pwm.prescaler / arr

    reg = if pwm.timer_index < 4
        UInt8(REG_ARR + pwm.timer_index)
    else
        UInt8(REG_ARR2 + pwm.timer_index - 4)
    end
    _i2c_write_16(pwm, reg, arr)
end

"""Set pulse width (0-65535)"""
function pulse_width!(pwm::PWMChannel, width::Int)
    pwm.pulse_width = width
    reg = UInt8(REG_CHN + pwm.channel)
    _i2c_write_16(pwm, reg, width)
end

"""Set pulse width as percentage (0-100)"""
function pulse_width_percent!(pwm::PWMChannel, percent::Real)
    arr = TIMER_ARR[pwm.timer_index + 1]
    width = round(Int, percent / 100.0 * arr)
    pulse_width!(pwm, width)
end

# =============================================================================
# ADC Channel
# =============================================================================

struct ADCChannel
    dev::I2CDevice
    channel::Int
    reg::UInt8
end

"""
    ADCChannel(dev::I2CDevice, channel)

Create ADC channel (0-7 or "A0"-"A7"). Returns 12-bit values (0-4095).
"""
function ADCChannel(dev::I2CDevice, channel::Int)
    @assert 0 <= channel <= 7 "ADC channel must be 0-7"
    chn = 7 - channel  # Channel mapping from Python
    reg = UInt8(chn | 0x10)
    ADCChannel(dev, channel, reg)
end

function ADCChannel(dev::I2CDevice, channel::AbstractString)
    if startswith(channel, "A")
        ADCChannel(dev, parse(Int, channel[2:end]))
    else
        error("ADC channel string must start with 'A', got: $channel")
    end
end

"""Read raw ADC value (0-4095)"""
function Base.read(adc::ADCChannel)::Int
    # Write channel register with 2 dummy bytes, then read 2 bytes
    write_block(adc.dev, adc.reg, UInt8[0, 0])
    sleep(0.001)  # Small delay for ADC conversion
    data = _raw_read(adc.dev, 2)
    # Big-endian: MSB first
    return (Int(data[1]) << 8) | Int(data[2])
end

"""Read ADC value as voltage (0-3.3V)"""
function read_voltage(adc::ADCChannel)::Float64
    read(adc) * 3.3 / 4095
end

# =============================================================================
# Servo
# =============================================================================

mutable struct Servo
    pwm::PWMChannel
    min_pulse::Int   # Pulse width at -90° (μs)
    max_pulse::Int   # Pulse width at +90° (μs)
    offset::Float64  # Angle offset for calibration
end

"""
    Servo(dev::I2CDevice, channel; min_pulse=500, max_pulse=2500, offset=0.0)

Create servo on PWM channel. Default pulse range 500-2500μs at 50Hz.
"""
function Servo(dev::I2CDevice, channel; min_pulse=500, max_pulse=2500, offset=0.0)
    pwm = PWMChannel(dev, channel)
    freq!(pwm, 50)  # Standard servo frequency
    Servo(pwm, min_pulse, max_pulse, offset)
end

"""Set servo angle (-90 to +90 degrees)"""
function angle!(servo::Servo, angle::Real)
    angle = clamp(angle + servo.offset, -90, 90)

    # Map angle to pulse width
    # -90° -> min_pulse, +90° -> max_pulse
    pulse_range = servo.max_pulse - servo.min_pulse
    pulse = servo.min_pulse + (angle + 90) / 180 * pulse_range

    # Convert μs to PWM value
    # At 50Hz, period = 20000μs, so pulse_width = pulse * arr / 20000
    arr = TIMER_ARR[servo.pwm.timer_index + 1]
    width = round(Int, pulse * arr / 20000)
    pulse_width!(servo.pwm, width)
end

# =============================================================================
# Motor
# =============================================================================

struct MotorMode1
    pwm::PWMChannel
    dir_pin::Pin
    reversed::Bool
end

struct MotorMode2
    pwm_a::PWMChannel
    pwm_b::PWMChannel
    reversed::Bool
end

const Motor = Union{MotorMode1, MotorMode2}

"""Create Mode 1 motor (PWM + direction pin) - for TC1508S driver"""
function Motor(dev::I2CDevice, pwm_channel, dir_pin; reversed=false)
    pwm = PWMChannel(dev, pwm_channel)
    freq!(pwm, 100)
    pin = Pin(dir_pin, mode=:out)
    MotorMode1(pwm, pin, reversed)
end

"""Create Mode 2 motor (dual PWM) - for TC618S driver"""
function Motor(dev::I2CDevice, pwm_a_channel, pwm_b_channel, ::Val{:mode2}; reversed=false)
    pwm_a = PWMChannel(dev, pwm_a_channel)
    pwm_b = PWMChannel(dev, pwm_b_channel)
    freq!(pwm_a, 100)
    freq!(pwm_b, 100)
    MotorMode2(pwm_a, pwm_b, reversed)
end

"""Set motor speed (-100 to +100)"""
function speed!(motor::MotorMode1, speed::Real)
    dir = speed > 0 ? 1 : 0
    if motor.reversed
        dir = 1 - dir
    end
    value!(motor.dir_pin, dir)
    pulse_width_percent!(motor.pwm, abs(speed))
end

function speed!(motor::MotorMode2, speed::Real)
    dir = speed > 0
    if motor.reversed
        dir = !dir
    end
    if dir
        pulse_width_percent!(motor.pwm_a, abs(speed))
        pulse_width_percent!(motor.pwm_b, 0)
    else
        pulse_width_percent!(motor.pwm_a, 0)
        pulse_width_percent!(motor.pwm_b, abs(speed))
    end
end

function Base.close(motor::MotorMode1)
    close(motor.dir_pin)
end

function Base.close(motor::MotorMode2)
    # PWM channels don't need explicit cleanup
end

# =============================================================================
# Utility Functions
# =============================================================================

"""Get firmware version from MCU"""
function firmware_version(dev::I2CDevice)
    data = mem_read(dev, UInt8(0x05), 3)
    return "$(data[1]).$(data[2]).$(data[3])"
end

"""Reset MCU via MCURST pin"""
function reset_mcu()
    pin = Pin("MCURST", mode=:out)
    value!(pin, 0)
    sleep(0.01)
    value!(pin, 1)
    sleep(0.01)
    close(pin)
end

"""Get battery voltage (via ADC channel A4, with voltage divider)"""
function battery_voltage(dev::I2CDevice)
    adc = ADCChannel(dev, "A4")
    # Voltage divider: actual = measured * 3
    read_voltage(adc) * 3
end

# =============================================================================
# Convenience: High-level initialization
# =============================================================================

"""
    RobotHat.open(; bus=1)

Initialize connection to Robot HAT. Returns I2CDevice.

Example:
    dev = RobotHat.open()
    servo = Servo(dev, "P0")
    angle!(servo, 45)
    close(dev)
"""
function open(; bus=I2C_BUS)
    I2CDevice(; bus=bus)
end

end # module
