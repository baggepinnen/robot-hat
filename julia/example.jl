# Example usage of RobotHat.jl (daemon-free version)
#
# Prerequisites:
#   1. Enable I2C: sudo raspi-config -> Interface Options -> I2C -> Enable
#   2. Set permissions: sudo chmod 666 /dev/i2c-1
#      Or add user to i2c group: sudo usermod -aG i2c $USER (then logout/login)
#   3. Run: julia example.jl

include("RobotHat.jl")
using .RobotHat

println("Opening Robot HAT...")
dev = RobotHat.open()

println("Connected to Robot HAT at address 0x$(string(dev.address, base=16))")
println("Firmware version: ", firmware_version(dev))

# --- Servo Example ---
println("\n--- Servo Test ---")
servo = Servo(dev, "P0")  # Servo on channel P0

for angle in [-45, 0, 45, 0]
    println("Moving servo to $angle°")
    angle!(servo, angle)
    sleep(0.5)
end

# --- PWM Example ---
println("\n--- PWM Test ---")
pwm = PWMChannel(dev, "P1")
freq!(pwm, 1000)  # 1kHz

for duty in [25, 50, 75, 50, 0]
    println("PWM duty cycle: $duty%")
    pulse_width_percent!(pwm, duty)
    sleep(0.3)
end

# --- ADC Example ---
println("\n--- ADC Test ---")
adc = ADCChannel(dev, "A0")

for _ in 1:5
    raw = read(adc)
    voltage = read_voltage(adc)
    println("ADC A0: raw=$raw, voltage=$(round(voltage, digits=3))V")
    sleep(0.2)
end

# --- Battery voltage ---
println("\n--- Battery ---")
println("Battery voltage: $(round(battery_voltage(dev), digits=2))V")

# --- Motor Example (Mode 1: PWM + direction pin) ---
# Uncomment if you have motors connected:
#
# println("\n--- Motor Test ---")
# motor = Motor(dev, "P12", "D4")
#
# println("Motor forward 50%")
# speed!(motor, 50)
# sleep(1)
#
# println("Motor backward 50%")
# speed!(motor, -50)
# sleep(1)
#
# println("Motor stop")
# speed!(motor, 0)
# close(motor)

# Cleanup
close(dev)
println("\nDone!")
