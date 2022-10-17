// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Iot.Device.Sx1509
{
    /// <summary>
    /// I/O Expander internal registers for Bank A
    /// </summary>
    public enum Register : byte
    {
        /// <summary>
        /// RegInputDisableB Input buffer disable register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_INPUT_DISABLE_B = 0x00,

        /// <summary>
        /// RegInputDisableA Input buffer disable register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_INPUT_DISABLE_A = 0x01,

        /// <summary>
        /// RegLongSlewB Output buffer long slew register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_LONG_SLEW_B = 0x02,

        /// <summary>
        /// RegLongSlewA Output buffer long slew register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_LONG_SLEW_A = 0x03,

        /// <summary>
        /// RegLowDriveB Output buffer low drive register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_LOW_DRIVE_B = 0x04,

        /// <summary>
        /// RegLowDriveA Output buffer low drive register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_LOW_DRIVE_A = 0x05,

        /// <summary>
        /// RegPullUpB Pull_up register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_PULL_UP_B = 0x06,

        /// <summary>
        /// RegPullUpA Pull_up register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_PULL_UP_A = 0x07,

        /// <summary>
        /// RegPullDownB Pull_down register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_PULL_DOWN_B = 0x08,

        /// <summary>
        /// RegPullDownA Pull_down register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_PULL_DOWN_A = 0x09,

        /// <summary>
        /// RegOpenDrainB Open drain register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_OPEN_DRAIN_B = 0x0A,

        /// <summary>
        /// RegOpenDrainA Open drain register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_OPEN_DRAIN_A = 0x0B,

        /// <summary>
        /// RegPolarityB Polarity register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_POLARITY_B = 0x0C,

        /// <summary>
        /// RegPolarityA Polarity register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_POLARITY_A = 0x0D,

        /// <summary>
        /// RegDirB Direction register _ I/O[15_8] (Bank B) 1111 1111
        /// </summary>
        REG_DIR_B = 0x0E,

        /// <summary>
        /// RegDirA Direction register _ I/O[7_0] (Bank A) 1111 1111
        /// </summary>
        REG_DIR_A = 0x0F,

        /// <summary>
        /// RegDataB Data register _ I/O[15_8] (Bank B) 1111 1111*
        /// </summary>
        REG_DATA_B = 0x10,

        /// <summary>
        /// RegDataA Data register _ I/O[7_0] (Bank A) 1111 1111*
        /// </summary>
        REG_DATA_A = 0x11,

        /// <summary>
        /// RegInterruptMaskB Interrupt mask register _ I/O[15_8] (Bank B) 1111 1111
        /// </summary>
        REG_INTERRUPT_MASK_B = 0x12,

        /// <summary>
        /// RegInterruptMaskA Interrupt mask register _ I/O[7_0] (Bank A) 1111 1111
        /// </summary>
        REG_INTERRUPT_MASK_A = 0x13,

        /// <summary>
        /// RegSenseHighB Sense register for I/O[15:12] 0000 0000
        /// </summary>
        REG_SENSE_HIGH_B = 0x14,

        /// <summary>
        /// RegSenseLowB Sense register for I/O[11:8] 0000 0000
        /// </summary>
        REG_SENSE_LOW_B = 0x15,

        /// <summary>
        /// RegSenseHighA Sense register for I/O[7:4] 0000 0000
        /// </summary>
        REG_SENSE_HIGH_A = 0x16,

        /// <summary>
        /// RegSenseLowA Sense register for I/O[3:0] 0000 0000
        /// </summary>
        REG_SENSE_LOW_A = 0x17,

        /// <summary>
        /// RegInterruptSourceB Interrupt source register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_INTERRUPT_SOURCE_B = 0x18,

        /// <summary>
        /// RegInterruptSourceA Interrupt source register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_INTERRUPT_SOURCE_A = 0x19,

        /// <summary>
        /// RegEventStatusB Event status register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_EVENT_STATUS_B = 0x1A,

        /// <summary>
        /// RegEventStatusA Event status register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_EVENT_STATUS_A = 0x1B,

        /// <summary>
        /// RegLevelShifter1 Level shifter register 0000 0000
        /// </summary>
        REG_LEVEL_SHIFTER_1 = 0x1C,

        /// <summary>
        /// RegLevelShifter2 Level shifter register 0000 0000
        /// </summary>
        REG_LEVEL_SHIFTER_2 = 0x1D,

        /// <summary>
        /// RegClock Clock management register 0000 0000
        /// </summary>
        REG_CLOCK = 0x1E,

        /// <summary>
        /// RegMisc Miscellaneous device settings register 0000 0000
        /// </summary>
        REG_MISC = 0x1F,

        /// <summary>
        /// RegLEDDriverEnableB LED driver enable register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_LED_DRIVER_ENABLE_B = 0x20,

        /// <summary>
        /// RegLEDDriverEnableA LED driver enable register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_LED_DRIVER_ENABLE_A = 0x21,

        // Debounce and Keypad Engine

        /// <summary>
        /// RegDebounceConfig Debounce configuration register 0000 0000
        /// </summary>
        REG_DEBOUNCE_CONFIG = 0x22,

        /// <summary>
        /// RegDebounceEnableB Debounce enable register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_DEBOUNCE_ENABLE_B = 0x23,

        /// <summary>
        /// RegDebounceEnableA Debounce enable register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_DEBOUNCE_ENABLE_A = 0x24,

        /// <summary>
        /// RegKeyConfig1 Key scan configuration register 0000 0000
        /// </summary>
        REG_KEY_CONFIG_1 = 0x25,

        /// <summary>
        /// RegKeyConfig2 Key scan configuration register 0000 0000
        /// </summary>
        REG_KEY_CONFIG_2 = 0x26,

        /// <summary>
        /// RegKeyData1 Key value (column) 1111 1111
        /// </summary>
        REG_KEY_DATA_1 = 0x27,

        /// <summary>
        /// RegKeyData2 Key value (row) 1111 1111
        /// </summary>
        REG_KEY_DATA_2 = 0x28,

        // LED Driver (PWM, blinking, breathing)

        /// <summary>
        /// RegTOn0 ON time register for I/O[0] 0000 0000
        /// </summary>
        REG_T_ON_0 = 0x29,

        /// <summary>
        /// RegIOn0 ON intensity register for I/O[0] 1111 1111
        /// </summary>
        REG_I_ON_0 = 0x2A,

        /// <summary>
        /// RegOff0 OFF time/intensity register for I/O[0] 0000 0000
        /// </summary>
        REG_OFF_0 = 0x2B,

        /// <summary>
        /// RegTOn1 ON time register for I/O[1] 0000 0000
        /// </summary>
        REG_T_ON_1 = 0x2C,

        /// <summary>
        /// RegIOn1 ON intensity register for I/O[1] 1111 1111
        /// </summary>
        REG_I_ON_1 = 0x2D,

        /// <summary>
        /// RegOff1 OFF time/intensity register for I/O[1] 0000 0000
        /// </summary>
        REG_OFF_1 = 0x2E,

        /// <summary>
        /// RegTOn2 ON time register for I/O[2] 0000 0000
        /// </summary>
        REG_T_ON_2 = 0x2F,

        /// <summary>
        /// RegIOn2 ON intensity register for I/O[2] 1111 1111
        /// </summary>
        REG_I_ON_2 = 0x30,

        /// <summary>
        /// RegOff2 OFF time/intensity register for I/O[2] 0000 0000
        /// </summary>
        REG_OFF_2 = 0x31,

        /// <summary>
        /// RegTOn3 ON time register for I/O[3] 0000 0000
        /// </summary>
        REG_T_ON_3 = 0x32,

        /// <summary>
        /// RegIOn3 ON intensity register for I/O[3] 1111 1111
        /// </summary>
        REG_I_ON_3 = 0x33,

        /// <summary>
        /// RegOff3 OFF time/intensity register for I/O[3] 0000 0000
        /// </summary>
        REG_OFF_3 = 0x34,

        /// <summary>
        /// RegTOn4 ON time register for I/O[4] 0000 0000
        /// </summary>
        REG_T_ON_4 = 0x35,

        /// <summary>
        /// RegIOn4 ON intensity register for I/O[4] 1111 1111
        /// </summary>
        REG_I_ON_4 = 0x36,

        /// <summary>
        /// RegOff4 OFF time/intensity register for I/O[4] 0000 0000
        /// </summary>
        REG_OFF_4 = 0x37,

        /// <summary>
        /// RegTRise4 Fade in register for I/O[4] 0000 0000
        /// </summary>
        REG_T_RISE_4 = 0x38,

        /// <summary>
        /// RegTFall4 Fade out register for I/O[4] 0000 0000
        /// </summary>
        REG_T_FALL_4 = 0x39,

        /// <summary>
        /// RegTOn5 ON time register for I/O[5] 0000 0000
        /// </summary>
        REG_T_ON_5 = 0x3A,

        /// <summary>
        /// RegIOn5 ON intensity register for I/O[5] 1111 1111
        /// </summary>
        REG_I_ON_5 = 0x3B,

        /// <summary>
        /// RegOff5 OFF time/intensity register for I/O[5] 0000 0000
        /// </summary>
        REG_OFF_5 = 0x3C,

        /// <summary>
        /// RegTRise5 Fade in register for I/O[5] 0000 0000
        /// </summary>
        REG_T_RISE_5 = 0x3D,

        /// <summary>
        /// RegTFall5 Fade out register for I/O[5] 0000 0000
        /// </summary>
        REG_T_FALL_5 = 0x3E,

        /// <summary>
        /// RegTOn6 ON time register for I/O[6] 0000 0000
        /// </summary>
        REG_T_ON_6 = 0x3F,

        /// <summary>
        /// RegIOn6 ON intensity register for I/O[6] 1111 1111
        /// </summary>
        REG_I_ON_6 = 0x40,

        /// <summary>
        /// RegOff6 OFF time/intensity register for I/O[6] 0000 0000
        /// </summary>
        REG_OFF_6 = 0x41,

        /// <summary>
        /// RegTRise6 Fade in register for I/O[6] 0000 0000
        /// </summary>
        REG_T_RISE_6 = 0x42,

        /// <summary>
        /// RegTFall6 Fade out register for I/O[6] 0000 0000
        /// </summary>
        REG_T_FALL_6 = 0x43,

        /// <summary>
        /// RegTOn7 ON time register for I/O[7] 0000 0000
        /// </summary>
        REG_T_ON_7 = 0x44,

        /// <summary>
        /// RegIOn7 ON intensity register for I/O[7] 1111 1111
        /// </summary>
        REG_I_ON_7 = 0x45,

        /// <summary>
        /// RegOff7 OFF time/intensity register for I/O[7] 0000 0000
        /// </summary>
        REG_OFF_7 = 0x46,

        /// <summary>
        /// RegTRise7 Fade in register for I/O[7] 0000 0000
        /// </summary>
        REG_T_RISE_7 = 0x47,

        /// <summary>
        /// RegTFall7 Fade out register for I/O[7] 0000 0000
        /// </summary>
        REG_T_FALL_7 = 0x48,

        /// <summary>
        /// RegTOn8 ON time register for I/O[8] 0000 0000
        /// </summary>
        REG_T_ON_8 = 0x49,

        /// <summary>
        /// RegIOn8 ON intensity register for I/O[8] 1111 1111
        /// </summary>
        REG_I_ON_8 = 0x4A,

        /// <summary>
        /// RegOff8 OFF time/intensity register for I/O[8] 0000 0000
        /// </summary>
        REG_OFF_8 = 0x4B,

        /// <summary>
        /// RegTOn9 ON time register for I/O[9] 0000 0000
        /// </summary>
        REG_T_ON_9 = 0x4C,

        /// <summary>
        /// RegIOn9 ON intensity register for I/O[9] 1111 1111
        /// </summary>
        REG_I_ON_9 = 0x4D,

        /// <summary>
        /// RegOff9 OFF time/intensity register for I/O[9] 0000 0000
        /// </summary>
        REG_OFF_9 = 0x4E,

        /// <summary>
        /// RegTOn10 ON time register for I/O[10] 0000 0000
        /// </summary>
        REG_T_ON_10 = 0x4F,

        /// <summary>
        /// RegIOn10 ON intensity register for I/O[10] 1111 1111
        /// </summary>
        REG_I_ON_10 = 0x50,

        /// <summary>
        /// RegOff10 OFF time/intensity register for I/O[10] 0000 0000
        /// </summary>
        REG_OFF_10 = 0x51,

        /// <summary>
        /// RegTOn11 ON time register for I/O[11] 0000 0000
        /// </summary>
        REG_T_ON_11 = 0x52,

        /// <summary>
        /// RegIOn11 ON intensity register for I/O[11] 1111 1111
        /// </summary>
        REG_I_ON_11 = 0x53,

        /// <summary>
        /// RegOff11 OFF time/intensity register for I/O[11] 0000 0000
        /// </summary>
        REG_OFF_11 = 0x54,

        /// <summary>
        /// RegTOn12 ON time register for I/O[12] 0000 0000
        /// </summary>
        REG_T_ON_12 = 0x55,

        /// <summary>
        /// RegIOn12 ON intensity register for I/O[12] 1111 1111
        /// </summary>
        REG_I_ON_12 = 0x56,

        /// <summary>
        /// RegOff12 OFF time/intensity register for I/O[12] 0000 0000
        /// </summary>
        REG_OFF_12 = 0x57,

        /// <summary>
        /// RegTRise12 Fade in register for I/O[12] 0000 0000
        /// </summary>
        REG_T_RISE_12 = 0x58,

        /// <summary>
        /// RegTFall12 Fade out register for I/O[12] 0000 0000
        /// </summary>
        REG_T_FALL_12 = 0x59,

        /// <summary>
        /// RegTOn13 ON time register for I/O[13] 0000 0000
        /// </summary>
        REG_T_ON_13 = 0x5A,

        /// <summary>
        /// RegIOn13 ON intensity register for I/O[13] 1111 1111
        /// </summary>
        REG_I_ON_13 = 0x5B,

        /// <summary>
        /// RegOff13 OFF time/intensity register for I/O[13] 0000 0000
        /// </summary>
        REG_OFF_13 = 0x5C,

        /// <summary>
        /// RegTRise13 Fade in register for I/O[13] 0000 0000
        /// </summary>
        REG_T_RISE_13 = 0x5D,

        /// <summary>
        /// RegTFall13 Fade out register for I/O[13] 0000 0000
        /// </summary>
        REG_T_FALL_13 = 0x5E,

        /// <summary>
        /// RegTOn14 ON time register for I/O[14] 0000 0000
        /// </summary>
        REG_T_ON_14 = 0x5F,

        /// <summary>
        /// RegIOn14 ON intensity register for I/O[14] 1111 1111
        /// </summary>
        REG_I_ON_14 = 0x60,

        /// <summary>
        /// RegOff14 OFF time/intensity register for I/O[14] 0000 0000
        /// </summary>
        REG_OFF_14 = 0x61,

        /// <summary>
        /// RegTRise14 Fade in register for I/O[14] 0000 0000
        /// </summary>
        REG_T_RISE_14 = 0x62,

        /// <summary>
        /// RegTFall14 Fade out register for I/O[14] 0000 0000
        /// </summary>
        REG_T_FALL_14 = 0x63,

        /// <summary>
        /// RegTOn15 ON time register for I/O[15] 0000 0000
        /// </summary>
        REG_T_ON_15 = 0x64,

        /// <summary>
        /// RegIOn15 ON intensity register for I/O[15] 1111 1111
        /// </summary>
        REG_I_ON_15 = 0x65,

        /// <summary>
        /// RegOff15 OFF time/intensity register for I/O[15] 0000 0000
        /// </summary>
        REG_OFF_15 = 0x66,

        /// <summary>
        /// RegTRise15 Fade in register for I/O[15] 0000 0000
        /// </summary>
        REG_T_RISE_15 = 0x67,

        /// <summary>
        /// RegTFall15 Fade out register for I/O[15] 0000 0000
        /// </summary>
        REG_T_FALL_15 = 0x68,

        // Miscellaneous

        /// <summary>
        /// RegHighInputB High input enable register _ I/O[15_8] (Bank B) 0000 0000
        /// </summary>
        REG_HIGH_INPUT_B = 0x69,

        /// <summary>
        /// RegHighInputA High input enable register _ I/O[7_0] (Bank A) 0000 0000
        /// </summary>
        REG_HIGH_INPUT_A = 0x6A,

        // Software Reset

        /// <summary>
        /// RegReset Software reset register 0000 0000
        /// </summary>
        REG_RESET = 0x7D,

        /// <summary>
        /// RegTest1 Test register 0000 0000
        /// </summary>
        REG_TEST_1 = 0x7E,

        /// <summary>
        /// RegTest2 Test register 0000 000
        /// </summary>
        REG_TEST_2 = 0x7F,

        /// <summary>
        /// Not Applicable/Not used
        /// </summary>
        NA = 0xFF
    }

    public static class RegisterAccess
    {
        public static readonly Register[] REG_I_ON = new Register[]
        {
            Register.REG_I_ON_0,
            Register.REG_I_ON_1,
            Register.REG_I_ON_2,
            Register.REG_I_ON_3,
            Register.REG_I_ON_4,
            Register.REG_I_ON_5,
            Register.REG_I_ON_6,
            Register.REG_I_ON_7,
            Register.REG_I_ON_8,
            Register.REG_I_ON_9,
            Register.REG_I_ON_10,
            Register.REG_I_ON_11,
            Register.REG_I_ON_12,
            Register.REG_I_ON_13,
            Register.REG_I_ON_14,
            Register.REG_I_ON_15
        };

        public static readonly Register[] REG_T_ON = new Register[]
        {
            Register.REG_T_ON_0,
            Register.REG_T_ON_1,
            Register.REG_T_ON_2,
            Register.REG_T_ON_3,
            Register.REG_T_ON_4,
            Register.REG_T_ON_5,
            Register.REG_T_ON_6,
            Register.REG_T_ON_7,
            Register.REG_T_ON_8,
            Register.REG_T_ON_9,
            Register.REG_T_ON_10,
            Register.REG_T_ON_11,
            Register.REG_T_ON_12,
            Register.REG_T_ON_13,
            Register.REG_T_ON_14,
            Register.REG_T_ON_15
        };

        public static readonly Register[] REG_OFF = new Register[]
        {
            Register.REG_OFF_0,
            Register.REG_OFF_1,
            Register.REG_OFF_2,
            Register.REG_OFF_3,
            Register.REG_OFF_4,
            Register.REG_OFF_5,
            Register.REG_OFF_6,
            Register.REG_OFF_7,
            Register.REG_OFF_8,
            Register.REG_OFF_9,
            Register.REG_OFF_10,
            Register.REG_OFF_11,
            Register.REG_OFF_12,
            Register.REG_OFF_13,
            Register.REG_OFF_14,
            Register.REG_OFF_15
        };

        public static readonly Register[] REG_T_RISE = new Register[]
        {
            Register.NA,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.REG_T_RISE_4,
            Register.REG_T_RISE_5,
            Register.REG_T_RISE_6,
            Register.REG_T_RISE_7,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.REG_T_RISE_12,
            Register.REG_T_RISE_13,
            Register.REG_T_RISE_14,
            Register.REG_T_RISE_15
        };

        public static readonly Register[] REG_T_FALL = new Register[]
        {
            Register.NA,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.REG_T_FALL_4,
            Register.REG_T_FALL_5,
            Register.REG_T_FALL_6,
            Register.REG_T_FALL_7,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.NA,
            Register.REG_T_FALL_12,
            Register.REG_T_FALL_13,
            Register.REG_T_FALL_14,
            Register.REG_T_FALL_15
        };
    }
}
