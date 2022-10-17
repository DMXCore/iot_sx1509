// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Device.Gpio;
using System.Device.I2c;
using System.Threading;

namespace Iot.Device.Sx1509
{
    /// <summary>
    /// A general purpose parallel I/O expansion for I2C applications.
    /// </summary>
    public class Sx1509 : IDisposable
    {
        public enum LedDriverModes : byte
        {
            Linear = 0,
            Logarithmic = 1
        }

        public enum ClockModes : byte
        {
            InternalClock2Mhz = 2,
            ExternalClock = 1
        }

        public enum PinDirections : byte
        {
            Input = 0,
            Output = 1,
            InputPullUp = 2,
            AnalogOutput = 3
        }

        private readonly int? _interruptPin;
        private readonly int? _resetPin;
        private readonly int? _oscillatorPin;
        private readonly bool _shouldDispose;
        private I2cDevice _i2cDevice;
        private GpioController? _controller = null;
        private int _clkX;

        /// <summary>
        /// Initializes new instance of Sx1509.
        /// A general purpose parallel I/O expansion for I2C applications.
        /// </summary>
        /// <param name="i2cDevice">The I2C device used for communication.</param>
        /// <param name="resetPin">The input pin number that is connected to the reset pin.</param>
        /// <param name="interruptPin">The input pin number that is connected to the interrupt (INT).</param>
        /// <param name="oscillatorPin">The input pin number that is connected to the oscillator pin.</param>
        /// <param name="gpioController"><see cref="GpioController"/> related with operations on pins</param>
        /// <param name="shouldDispose">True to dispose the Gpio Controller</param>
        public Sx1509(I2cDevice i2cDevice, int? resetPin = null, int? interruptPin = null, int? oscillatorPin = null, GpioController? gpioController = null, bool shouldDispose = true)
        {
            _i2cDevice = i2cDevice;
            _resetPin = resetPin;
            _interruptPin = interruptPin;
            _oscillatorPin = oscillatorPin;
            _shouldDispose = shouldDispose || gpioController is null;

            InitializeGpioController(gpioController);

            Init();
        }

        private static byte Constrain(byte input, byte min, byte max)
        {
            if (input < min)
            {
                return min;
            }

            if (input > max)
            {
                return max;
            }

            return input;
        }

        private void InitializeGpioController(GpioController? gpioController)
        {
            // Only need master controller if there is external pins provided.
            if (_interruptPin.HasValue)
            {
                _controller = gpioController ?? new GpioController();
                _controller.OpenPin(_interruptPin.Value, System.Device.Gpio.PinMode.Input);
            }

            if (_resetPin.HasValue)
            {
                _controller = gpioController ?? new GpioController();
                _controller.OpenPin(_resetPin.Value, System.Device.Gpio.PinMode.Output);
            }

            if (_oscillatorPin.HasValue)
            {
                _controller = gpioController ?? new GpioController();
                _controller.OpenPin(_oscillatorPin.Value, System.Device.Gpio.PinMode.Input);
            }
        }

        private byte Init()
        {
            // If the reset pin is connected
            Reset();

            // Communication test. We'll read from two registers with different
            // default values to verify communication.
            int testRegisters = 0;
            testRegisters = ReadWord(Register.REG_INTERRUPT_MASK_A); // This should return 0xFF00

            // Then read a byte that should be 0x00
            if (testRegisters == 0xFF00)
            {
                // Set the clock to a default of 2MHz using internal
                Clock(ClockModes.InternalClock2Mhz);

                return 1;
            }

            return 0;
        }

        public void Reset()
        {
            // if hardware bool is set
            if (_resetPin.HasValue)
            {
                // Check if bit 2 of REG_MISC is set
                // if so nReset will not issue a POR, we'll need to clear that bit first
                int regMisc = ReadByte(Register.REG_MISC);
                if ((regMisc & (1 << 2)) != 0)
                {
                    regMisc &= ~(1 << 2);
                    WriteByte(Register.REG_MISC, regMisc);
                }

                // Reset the SX1509, the pin is active low

                // set reset pin as output
                PinMode(_resetPin.Value, PinDirections.Output);

                // pull reset pin low
                DigitalWrite(_resetPin.Value, false);

                // Wait for the pin to settle
                Thread.Sleep(1);

                // pull reset pin back high
                DigitalWrite(_resetPin.Value, true);
            }
            else
            {
                // Software reset command sequence:
                WriteByte(Register.REG_RESET, 0x12);
                WriteByte(Register.REG_RESET, 0x34);
            }
        }

        /// <summary>
        /// Reads a byte from a register.
        /// </summary>
        /// <param name="register">The register to read.</param>
        /// <returns>The data read from the register.</returns>
        private byte ReadByte(Register register)
        {
            _i2cDevice.WriteByte((byte)register); // Set address to register first.
            byte data = _i2cDevice.ReadByte();
            return data;
        }

        /// <summary>
        /// Reads a word from a register.
        /// </summary>
        /// <param name="register">The register to read.</param>
        /// <returns>The data read from the register.</returns>
        private ushort ReadWord(Register register)
        {
            _i2cDevice.WriteByte((byte)register); // Set address to register first.
            var buf = new byte[2];
            _i2cDevice.Read(buf);

            return (ushort)(buf[0] << 8 | buf[1]);
        }

        /// <summary>
        /// Reads the pin value of the interrupt (INT).
        /// </summary>
        /// <returns>The pin value of the interrupt (INT).</returns>
        public PinValue ReadInterrupt()
        {
            if (_controller is null)
            {
                throw new Exception("Master controller has not been initialized.");
            }

            if (_interruptPin is null)
            {
                throw new Exception("INT pin has not been initialized.");
            }

            return _controller.Read((int)_interruptPin);
        }

        private byte CalculateLEDTRegister(int ms)
        {
            int regOn1, regOn2;
            double timeOn1, timeOn2;

            if (_clkX == 0)
            {
                return 0;
            }

            regOn1 = (int)((double)(ms / 1000.0) / (64.0 * 255.0 / (double)_clkX));
            regOn2 = (int)(regOn1 / 8);
            regOn1 = Constrain((byte)regOn1, 1, 15);
            regOn2 = Constrain((byte)regOn2, 16, 31);

            timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
            timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

            if (Math.Abs(timeOn1 - ms) < Math.Abs(timeOn2 - ms))
            {
                return (byte)regOn1;
            }
            else
            {
                return (byte)regOn2;
            }
        }

        private byte CalculateSlopeRegister(int ms, int onIntensity, int offIntensity)
        {
            int regSlope1, regSlope2;
            double regTime1, regTime2;

            if (_clkX == 0)
            {
                return 0;
            }

            double tFactor = ((double)onIntensity - (4.0 * (double)offIntensity)) * 255.0 / (double)_clkX;
            double timeS = ((double)ms) / 1000.0;

            regSlope1 = (int)(timeS / tFactor);
            regSlope2 = regSlope1 / 16;

            regSlope1 = Constrain((byte)regSlope1, 1, 15);
            regSlope2 = Constrain((byte)regSlope2, 16, 31);

            regTime1 = regSlope1 * tFactor * 1000.0;
            regTime2 = 16 * regTime1;

            if (Math.Abs(regTime1 - ms) < Math.Abs(regTime2 - ms))
            {
                return (byte)regSlope1;
            }
            else
            {
                return (byte)regSlope2;
            }
        }

        public void Blink(int pin, int tOn, int tOff, byte onIntensity = 255, byte offIntensity = 0)
        {
            byte onReg = CalculateLEDTRegister(tOn);
            byte offReg = CalculateLEDTRegister(tOff);

            SetupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
        }

        public void Breathe(int pin, int tOn, int tOff, int rise, int fall, byte onIntensity = 255, byte offIntensity = 0, LedDriverModes driverMode = LedDriverModes.Linear)
        {
            offIntensity = Constrain(offIntensity, 0, 7);

            byte onReg = CalculateLEDTRegister(tOn);
            byte offReg = CalculateLEDTRegister(tOff);

            byte riseTime = CalculateSlopeRegister(rise, onIntensity, offIntensity);
            byte fallTime = CalculateSlopeRegister(fall, onIntensity, offIntensity);

            SetupBlink(pin, onReg, offReg, onIntensity, offIntensity, riseTime, fallTime, driverMode);
        }

        private void SetupBlink(int pin, byte tOn, byte tOff, byte onIntensity = 255, byte offIntensity = 00, byte tRise = 0, byte tFall = 0, LedDriverModes driverMode = LedDriverModes.Linear)
        {
            LedDriverInit(pin, driverMode: driverMode);

            // Keep parameters within their limits:
            tOn &= 0x1F;  // tOn should be a 5-bit value
            tOff &= 0x1F; // tOff should be a 5-bit value
            offIntensity &= 0x07;
            // Write the time on
            // 1-15:  TON = 64 * tOn * (255/ClkX)
            // 16-31: TON = 512 * tOn * (255/ClkX)
            WriteByte(RegisterAccess.REG_T_ON[pin], tOn);

            // Write the time/intensity off register
            // 1-15:  TOFF = 64 * tOff * (255/ClkX)
            // 16-31: TOFF = 512 * tOff * (255/ClkX)
            // linear Mode - IOff = 4 * offIntensity
            // log mode - Ioff = f(4 * offIntensity)
            WriteByte(RegisterAccess.REG_OFF[pin], (tOff << 3) | offIntensity);

            // Write the on intensity:
            WriteByte(RegisterAccess.REG_I_ON[pin], onIntensity);

            // Prepare tRise and tFall
            tRise &= 0x1F; // tRise is a 5-bit value
            tFall &= 0x1F; // tFall is a 5-bit value

            // Write regTRise
            // 0: Off
            // 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
            // 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
            if (RegisterAccess.REG_T_RISE[pin] != Register.NA)
            {
                WriteByte(RegisterAccess.REG_T_RISE[pin], tRise);
            }

            // Write regTFall
            // 0: off
            // 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
            // 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
            if (RegisterAccess.REG_T_FALL[pin] != Register.NA)
            {
                WriteByte(RegisterAccess.REG_T_FALL[pin], tFall);
            }
        }

        public void Sync()
        {
            // First check if nReset functionality is set
            byte regMisc = ReadByte(Register.REG_MISC);
            if ((regMisc & 0x04) == 0)
            {
                regMisc |= (1 << 2);
                WriteByte(Register.REG_MISC, regMisc);
            }

            // Toggle nReset pin to sync LED timers
            if (_resetPin.HasValue)
            {
                // set reset pin as output
                PinMode(_resetPin.Value, PinDirections.Output);

                // pull reset pin low
                DigitalWrite(_resetPin.Value, false);

                // Wait for the pin to settle
                Thread.Sleep(1);

                // pull reset pin back high
                DigitalWrite(_resetPin.Value, true);
            }

            // Return nReset to POR functionality
            WriteByte(Register.REG_MISC, (regMisc & ~(1 << 2)));
        }

        /// <summary>
        /// Writes a byte to a register.
        /// </summary>
        /// <param name="register">The register to write.</param>
        /// <param name="data">The data to write to the register.</param>
        private void WriteByte(Register register, byte data)
        {
            byte[] writeBuffer = new byte[] { (byte)register, data };
            _i2cDevice.Write(writeBuffer);
        }

        private void WriteWord(Register register, int data)
        {
            byte[] writeBuffer = new byte[] { (byte)register, (byte)(data >> 8), (byte)data };
            _i2cDevice.Write(writeBuffer);
        }

        private void WriteByte(Register register, int data) => WriteByte(register, (byte)data);

        public void PinMode(int pin, PinDirections inOut, bool initialLevel = true)
        {
            // The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
            //  0: IO is configured as an output
            //  1: IO is configured as an input
            bool isInput;
            if ((inOut == PinDirections.Output) || (inOut == PinDirections.AnalogOutput))
            {
                int tempRegData = ReadWord(Register.REG_DATA_B);
                if (!initialLevel)
                {
                    tempRegData &= ~(1 << pin);
                    WriteWord(Register.REG_DATA_B, tempRegData);
                }

                isInput = false;
            }
            else
            {
                isInput = true;
            }

            int tempRegDir = ReadWord(Register.REG_DIR_B);
            if (isInput)
            {
                tempRegDir |= (1 << pin);
            }
            else
            {
                tempRegDir &= ~(1 << pin);
            }

            WriteWord(Register.REG_DIR_B, tempRegDir);

            // If INPUT_PULLUP was called, set up the pullup too:
            if (inOut == PinDirections.InputPullUp)
            {
                DigitalWrite(pin, true);
            }

            if (inOut == PinDirections.AnalogOutput)
            {
                LedDriverInit(pin);
            }
        }

        public void DigitalWrite(int pin, bool value)
        {
            int tempRegDir = ReadWord(Register.REG_DIR_B);

            if (((0xFFFF ^ tempRegDir) & (1 << pin)) != 0) // If the pin is an output, write high/low
            {
                int tempRegData = ReadWord(Register.REG_DATA_B);
                if (value)
                {
                    tempRegData |= (1 << pin);
                }
                else
                {
                    tempRegData &= ~(1 << pin);
                }

                WriteWord(Register.REG_DATA_B, tempRegData);
            }
            else // Otherwise the pin is an input, pull-up/down
            {
                int tempPullUp = ReadWord(Register.REG_PULL_UP_B);
                int tempPullDown = ReadWord(Register.REG_PULL_DOWN_B);

                if (value) // if HIGH, do pull-up, disable pull-down
                {
                    tempPullUp |= (1 << pin);
                    tempPullDown &= ~(1 << pin);
                    WriteWord(Register.REG_PULL_UP_B, tempPullUp);
                    WriteWord(Register.REG_PULL_DOWN_B, tempPullDown);
                }
                else // If LOW do pull-down, disable pull-up
                {
                    tempPullDown |= (1 << pin);
                    tempPullUp &= ~(1 << pin);
                    WriteWord(Register.REG_PULL_UP_B, tempPullUp);
                    WriteWord(Register.REG_PULL_DOWN_B, tempPullDown);
                }
            }
        }

        public bool DigitalRead(int pin)
        {
            int tempRegDir = ReadWord(Register.REG_DIR_B);

            if ((tempRegDir & (1 << pin)) != 0) // If the pin is an input
            {
                int tempRegData = ReadWord(Register.REG_DATA_B);
                if ((tempRegData & (1 << pin)) != 0)
                {
                    return true;
                }
            }

            return false;
        }

        public void LedDriverInit(int pin, byte freq = 1, LedDriverModes driverMode = LedDriverModes.Linear)
        {
            int tempWord;
            int tempByte;

            // Disable input buffer
            // Writing a 1 to the pin bit will disable that pins input buffer
            tempWord = ReadWord(Register.REG_INPUT_DISABLE_B);
            tempWord |= (1 << pin);
            WriteWord(Register.REG_INPUT_DISABLE_B, tempWord);

            // Disable pull-up
            // Writing a 0 to the pin bit will disable that pull-up resistor
            tempWord = ReadWord(Register.REG_PULL_UP_B);
            tempWord &= ~(1 << pin);
            WriteWord(Register.REG_PULL_UP_B, tempWord);

            // Set direction to output (REG_DIR_B)
            tempWord = ReadWord(Register.REG_DIR_B);
            tempWord &= ~(1 << pin); // 0=output
            WriteWord(Register.REG_DIR_B, tempWord);

            // Enable oscillator (REG_CLOCK)
            tempByte = ReadByte(Register.REG_CLOCK);
            tempByte |= (1 << 6);  // Internal 2MHz oscillator part 1 (set bit 6)
            tempByte &= ~(1 << 5); // Internal 2MHz oscillator part 2 (clear bit 5)
            WriteByte(Register.REG_CLOCK, tempByte);

            // Configure LED driver clock and mode (REG_MISC)
            tempByte = ReadByte(Register.REG_MISC);
            if (driverMode == LedDriverModes.Logarithmic)
            {
                tempByte |= (1 << 7); // set logarithmic mode bank B
                tempByte |= (1 << 3); // set logarithmic mode bank A
            }
            else
            {
                tempByte &= ~(1 << 7); // set linear mode bank B
                tempByte &= ~(1 << 3); // set linear mode bank A
            }

            // Use configClock to setup the clock divder
            if (_clkX == 0) // Make clckX non-zero
            {
                // _clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable
                _clkX = 2000000;

                // byte freq = (1 & 0x07) << 4; // freq should only be 3 bits from 6:4
                // tempByte |= freq;
            }

            freq = (byte)((freq & 0x7) << 4);   // mask only 3 bits and shift to bit position 6:4
            tempByte |= freq;
            WriteByte(Register.REG_MISC, tempByte);

            // Enable LED driver operation (REG_LED_DRIVER_ENABLE)
            tempWord = ReadWord(Register.REG_LED_DRIVER_ENABLE_B);
            tempWord |= (1 << pin);
            WriteWord(Register.REG_LED_DRIVER_ENABLE_B, tempWord);

            // Set REG_DATA bit low ~ LED driver started
            tempWord = ReadWord(Register.REG_DATA_B);
            tempWord &= ~(1 << pin);
            WriteWord(Register.REG_DATA_B, tempWord);
        }

        public void Clock(ClockModes oscSource = ClockModes.InternalClock2Mhz, byte oscDivider = 1, byte oscPinFunction = 0, byte oscFreqOut = 0)
        {
            // RegClock constructed as follows:
            //  6:5 - Oscillator frequency souce
            //    00: off, 01: external input, 10: internal 2MHz, 1: reserved
            //  4 - OSCIO pin function
            //    0: input, 1 ouptut
            //  3:0 - Frequency of oscout pin
            //    0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
            byte oscSourceValue = (byte)(((byte)oscSource & 0b11) << 5);        // 2-bit value, bits 6:5
            oscPinFunction = (byte)((oscPinFunction & 1) << 4); // 1-bit value bit 4
            oscFreqOut = (byte)(oscFreqOut & 0b1111);         // 4-bit value, bits 3:0
            byte regClock = (byte)(oscSourceValue | oscPinFunction | oscFreqOut);
            WriteByte(Register.REG_CLOCK, regClock);

            // Config RegMisc[6:4] with oscDivider
            // 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
            oscDivider = Sx1509.Constrain(oscDivider, 1, 7);
            _clkX = (int)(2000000.0 / (1 << (oscDivider - 1))); // Update private clock variable
            oscDivider = (byte)((oscDivider & 0b111) << 4);      // 3-bit value, bits 6:4

            byte regMisc = ReadByte(Register.REG_MISC);
            // regMisc &= ~(0b111 << 4);
            regMisc &= 0b10001111;
            regMisc |= oscDivider;
            WriteByte(Register.REG_MISC, regMisc);
        }

        public void AnalogWrite(int pin, int iOn)
        {
            // Write the on intensity of pin
            // Linear mode: Ion = iOn
            // Log mode: Ion = f(iOn)
            WriteByte(RegisterAccess.REG_I_ON[pin], iOn);
        }

        public void DebounceConfig(byte configValue)
        {
            // First make sure clock is configured
            int tempByte = ReadByte(Register.REG_MISC);
            if ((tempByte & 0x70) == 0)
            {
                tempByte |= (1 << 4); // Just default to no divider if not set
                WriteByte(Register.REG_MISC, tempByte);
            }

            tempByte = ReadByte(Register.REG_CLOCK);
            if ((tempByte & 0x60) == 0)
            {
                tempByte |= (1 << 6); // default to internal osc.
                WriteByte(Register.REG_CLOCK, tempByte);
            }

            configValue &= 0b111; // 3-bit value
            WriteByte(Register.REG_DEBOUNCE_CONFIG, configValue);
        }

        public void DebounceTime(byte time)
        {
            if (_clkX == 0)
            {
                // If clock hasn't been set up.
                Clock(ClockModes.InternalClock2Mhz, 1); // Set clock to 2MHz.
            }

            // Debounce time-to-byte map: (assuming fOsc = 2MHz)
            // 0: 0.5ms     1: 1ms
            // 2: 2ms       3: 4ms
            // 4: 8ms       5: 16ms
            // 6: 32ms      7: 64ms
            // 2^(n-1)
            int configValue = 0;
            // We'll check for the highest set bit position,
            // and use that for debounceConfig
            for (int i = 7; i >= 0; i--)
            {
                if ((time & (1 << i)) != 0)
                {
                    configValue = i + 1;
                    break;
                }
            }

            configValue = Constrain((byte)configValue, 0, 7);

            DebounceConfig((byte)configValue);
        }

        public void DebouncePin(int pin)
        {
            int debounceEnable = ReadWord(Register.REG_DEBOUNCE_ENABLE_B);

            debounceEnable |= (1 << pin);

            WriteWord(Register.REG_DEBOUNCE_ENABLE_B, debounceEnable);
        }

        /// <inheritdoc/>
        public void Dispose()
        {
            _i2cDevice?.Dispose();
            _i2cDevice = null!;
            if (_shouldDispose)
            {
                _controller?.Dispose();
                _controller = null;
            }
        }
    }
}
