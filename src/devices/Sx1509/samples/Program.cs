// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Device.Gpio;
using System.Device.I2c;
using System.Threading;
using Iot.Device.Sx1509;

const byte busId = 22;

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
int s_deviceAddress = 0x3E;
// SX1509 Pin definition:
const byte SX1509_LED_PIN_A = 4; // LED to SX1509's pin
const byte SX1509_LED_PIN_B = 5; // LED to SX1509's pin
const byte SX1509_LED_PIN_C = 6; // LED to SX1509's pin
const byte SX1509_LED_PIN_D = 7; // LED to SX1509's pin

const byte SX1509_INPUT_PIN_A = 0;

Console.WriteLine("Hello Sx1509 Sample!");

using Sx1509 sxDevice = GetSx1509Device();
//Blink(sxDevice);
//Breathe(sxDevice);
//AnalogWrite(sxDevice);
ReadInputPort(sxDevice);

Sx1509 GetSx1509Device()
{
    I2cConnectionSettings i2cConnectionSettings = new(busId, s_deviceAddress);
    I2cDevice i2cDevice = I2cDevice.Create(i2cConnectionSettings);
    return new Sx1509(i2cDevice);
}

void Breathe(Sx1509 sxDevice)
{
    // Use the internal 2MHz oscillator.
    // Set LED clock to 500kHz (2MHz / (2^(3-1)):
    sxDevice.Clock(Sx1509.ClockModes.InternalClock2Mhz, 3);

    // To breathe an LED, make sure you set it as an
    // ANALOG_OUTPUT, so we can PWM the pin:
    sxDevice.PinMode(SX1509_LED_PIN_A, Sx1509.PinDirections.AnalogOutput);
    sxDevice.PinMode(SX1509_LED_PIN_B, Sx1509.PinDirections.AnalogOutput);
    sxDevice.PinMode(SX1509_LED_PIN_C, Sx1509.PinDirections.AnalogOutput);
    sxDevice.PinMode(SX1509_LED_PIN_D, Sx1509.PinDirections.AnalogOutput);

    // Breathe an LED: 1000ms LOW, 500ms HIGH,
    // 500ms to rise from low to high
    // 250ms to fall from high to low
    sxDevice.Breathe(SX1509_LED_PIN_A, 1000, 500, 500, 250);
    Thread.Sleep(500);
    sxDevice.Breathe(SX1509_LED_PIN_B, 1000, 500, 500, 250);
    sxDevice.Breathe(SX1509_LED_PIN_C, 1000, 500, 500, 250);
    sxDevice.Breathe(SX1509_LED_PIN_D, 1000, 500, 500, 250);
    // The timing parameters are in milliseconds, but they
    // aren't 100% exact. The library will estimate to try to
    // get them as close as possible. Play with the clock
    // divider to maybe get more accurate timing.

    // Sync the output (only works when reset pin is connected)
    sxDevice.Sync();
}

void Blink(Sx1509 sxDevice)
{
    // Set up the SX1509's clock to use the internal 2MHz
    // oscillator. The second parameter divides the oscillator
    // clock to generate a slower LED clock. 4 divides the 2MHz
    // clock by 2 ^ (4-1) (8, ie. 250kHz). The divider parameter
    // can be anywhere between 1-7.
    sxDevice.Clock(Sx1509.ClockModes.InternalClock2Mhz, 4);

    sxDevice.PinMode(SX1509_LED_PIN_A, Sx1509.PinDirections.Output); // Set LED pin to OUTPUT
    sxDevice.PinMode(SX1509_LED_PIN_B, Sx1509.PinDirections.Output); // Set LED pin to OUTPUT
    sxDevice.PinMode(SX1509_LED_PIN_C, Sx1509.PinDirections.Output); // Set LED pin to OUTPUT
    sxDevice.PinMode(SX1509_LED_PIN_D, Sx1509.PinDirections.Output); // Set LED pin to OUTPUT

    // Blink the LED pin -- ~1000 ms LOW, ~500 ms HIGH:
    sxDevice.Blink(SX1509_LED_PIN_A, 100, 50);
    sxDevice.Blink(SX1509_LED_PIN_B, 200, 150);
    sxDevice.Blink(SX1509_LED_PIN_C, 300, 250);
    sxDevice.Blink(SX1509_LED_PIN_D, 400, 350);
    // The timing parameters are in milliseconds, but they
    // aren't 100% exact. The library will estimate to try to
    // get them as close as possible. Play with the clock
    // divider to maybe get more accurate timing.
}

void CycleOutputBits(Sx1509 sxDevice)
{
    sxDevice.PinMode(SX1509_LED_PIN_A, Sx1509.PinDirections.Output); // Set LED pin to OUTPUT

    while (true)
    {
        sxDevice.DigitalWrite(SX1509_LED_PIN_A, true);
        Thread.Sleep(500);

        sxDevice.DigitalWrite(SX1509_LED_PIN_A, false);
        Thread.Sleep(500);
    }
}

void AnalogWrite(Sx1509 sxDevice)
{
    sxDevice.PinMode(SX1509_LED_PIN_A, Sx1509.PinDirections.AnalogOutput); // Set LED pin to OUTPUT

    while (true)
    {
        // Ramp brightness up, from 0-255, delay 2ms in between
        // analogWrite's
        for (int brightness = 0; brightness < 256; brightness++)
        {
            // Call io.analogWrite(<pin>, <0-255>) to configure the
            // PWM duty cycle
            sxDevice.AnalogWrite(SX1509_LED_PIN_A, brightness);
            Thread.Sleep(2); // Delay 2 milliseconds
        }

        Thread.Sleep(500); // Delay half-a-second

        // Ramp brightness down, from 255-0, delay 2ms in between
        // analogWrite's
        for (int brightness = 255; brightness >= 0; brightness--)
        {
            sxDevice.AnalogWrite(SX1509_LED_PIN_A, brightness);
            Thread.Sleep(2); // Delay 2 milliseconds
        }

        Thread.Sleep(500); // Delay half-a-second
    }
}

void ReadInputPort(Sx1509 sxDevice)
{
    sxDevice.PinMode(SX1509_INPUT_PIN_A, Sx1509.PinDirections.InputPullUp); // Set LED pin to OUTPUT
    sxDevice.DebounceConfig(4);
    sxDevice.DebouncePin(SX1509_INPUT_PIN_A);

    bool oldValue = false;
    while (true)
    {
        bool data = sxDevice.DigitalRead(SX1509_INPUT_PIN_A);
        if (oldValue != data)
        {
            Console.WriteLine($"Input Port: {data}");
            oldValue = data;
        }

        Thread.Sleep(10);
    }
}
