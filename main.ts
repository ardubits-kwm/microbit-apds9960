/**
*  Gesture Sensor
*/

/* Direction definitions */
enum DIR {
    DIR_NONE,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_UP,
    DIR_DOWN,
    DIR_NEAR,
    DIR_FAR,
    DIR_ALL
}

/* State definitions */
enum STATE {
    NA_STATE,
    NEAR_STATE,
    FAR_STATE,
    ALL_STATE
}

enum GESTURE_TYPE {
    //% block="UP"
    UP,
    //% block="DOWN"
    DOWN,
    //% block="LEFT"
    LEFT,
    //% block="RIGHT"
    RIGHT,
    //% block="NEAR"
    NEAR,
    //% block="FAR"
    FAR
}

enum GESTURE_WORKMODE {
    //% block="LIGHT MODE"
    LIGHT_MODE,
    //% block="PROXIMITY MODE"
    PROXIMITY_MODE,
    //% block="GESTURE MODE"
    GESTURE_MODE
}

enum LIGHT_UNIT {
    //% block="LUX"
    LUX = 1,
    //% block="FC"
    FC = 2
}

//% color="#45b787" weight=8 icon="\uf12e"
//% groups='["Mode", "Gesture", "Light", "Proximity", "Optional"]'
namespace YFGesture {
    /* Gesture parameters */
    let GESTURE_THRESHOLD_OUT = 30;
    let GESTURE_SENSITIVITY_1 = 33
    let GESTURE_SENSITIVITY_2 = 18

    /* Error code for returned values */
    //ERROR = 0xFF

    /* On/Off definitions */
    let OFF = 0;
    let ON = 1;

    /* Acceptable parameters for setMode */
    let POWER = 0
    let AMBIENT_LIGHT = 1
    let PROXIMITY = 2
    let WAIT = 3
    let AMBIENT_LIGHT_INT = 4
    let PROXIMITY_INT = 5
    let GESTURE = 6
    let ALL = 7

    /* LED Drive values */
    let LED_DRIVE_100MA = 0
    let LED_DRIVE_50MA = 1
    let LED_DRIVE_25MA = 2
    let LED_DRIVE_12_5MA = 3

    /* Proximity Gain (PGAIN) values */
    let PGAIN_1X = 0
    let PGAIN_2X = 1
    let PGAIN_4X = 2
    let PGAIN_8X = 3

    /* ALS Gain (AGAIN) values */
    let AGAIN_1X = 0
    let AGAIN_4X = 1
    let AGAIN_16X = 2
    let AGAIN_64X = 3

    /* Gesture Gain (GGAIN) values */
    let GGAIN_1X = 0
    let GGAIN_2X = 1
    let GGAIN_4X = 2
    let GGAIN_8X = 3

    /* LED Boost values */
    let LED_BOOST_100 = 0
    let LED_BOOST_150 = 1
    let LED_BOOST_200 = 2
    let LED_BOOST_300 = 3

    /* Gesture wait time values */
    let GWTIME_0MS = 0
    let GWTIME_2_8MS = 1
    let GWTIME_5_6MS = 2
    let GWTIME_8_4MS = 3
    let GWTIME_14_0MS = 4
    let GWTIME_22_4MS = 5
    let GWTIME_30_8MS = 6
    let GWTIME_39_2MS = 7

    /* Default values */
    let DEFAULT_ATIME = 219                 // 103ms
    let DEFAULT_WTIME = 246                 // 27ms
    let DEFAULT_PROX_PPULSE = 0x87          // 16us, 8 pulses
    let DEFAULT_GESTURE_PPULSE = 0x89       // 16us, 10 pulses
    let DEFAULT_POFFSET_UR = 0              // 0 offset
    let DEFAULT_POFFSET_DL = 0              // 0 offset      
    let DEFAULT_CONFIG1 = 0x60              // No 12x wait (WTIME) factor
    let DEFAULT_LDRIVE = LED_DRIVE_100MA
    let DEFAULT_PGAIN = PGAIN_4X
    let DEFAULT_AGAIN = AGAIN_4X
    let DEFAULT_PILT = 0                    // Low proximity threshold
    let DEFAULT_PIHT = 50                   // High proximity threshold
    let DEFAULT_AILT = 0xFFFF               // Force interrupt for calibration
    let DEFAULT_AIHT = 0
    let DEFAULT_PERS = 0x11                 // 2 consecutive prox or ALS for int.
    let DEFAULT_CONFIG2 = 0x01              // No saturation interrupts or LED boost  
    let DEFAULT_CONFIG3 = 0                 // Enable all photodiodes, no SAI
    let DEFAULT_GPENTH = 40                 // Threshold for entering gesture mode
    let DEFAULT_GEXTH = 30                  // Threshold for exiting gesture mode    
    let DEFAULT_GCONF1 = 0x40               // 4 gesture events for int., 1 for exit
    let DEFAULT_GGAIN = GGAIN_4X
    let DEFAULT_GLDRIVE = LED_DRIVE_100MA
    let DEFAULT_GWTIME = GWTIME_2_8MS
    let DEFAULT_GOFFSET = 0                 // No offset scaling for gesture mode
    let DEFAULT_GPULSE = 0xC9               // 32us, 10 pulses
    let DEFAULT_GCONF3 = 0                  // All photodiodes active during gesture
    let DEFAULT_GIEN = 0                    // Disable gesture interrupts
    
    /* Misc parameters */
    let FIFO_PAUSE_TIME = 30      // Wait period (ms) between FIFO reads

    let motion_global: number = DIR.DIR_NONE;

    let APDS9960_I2C_ADDR = 0x39;
    let APDS9960_ID_1 = 0xAB
    let APDS9960_ID_2 = 0x9C

    let gesture_data_u_data = pins.createBuffer(32);
    let gesture_data_d_data = pins.createBuffer(32);
    let gesture_data_l_data = pins.createBuffer(32);
    let gesture_data_r_data = pins.createBuffer(32);
    let gesture_data_index: NumberFormat.UInt8BE
    let gesture_data_total_gestures: NumberFormat.UInt8BE;
    let gesture_data_in_threshold: NumberFormat.UInt8BE;
    let gesture_data_out_threshold: NumberFormat.UInt8BE;

    let gesture_ud_delta_ = 0;
    let gesture_lr_delta_ = 0;

    let gesture_ud_count_ = 0;
    let gesture_lr_count_ = 0;

    let gesture_near_count_ = 0;
    let gesture_far_count_ = 0;

    let gesture_state_ = 0;
    let gesture_motion_ = DIR.DIR_NONE;

    //% advanced=true
    //% group=Optional
    //% blockId=YFGesture_begin
    //% block="begin"
    function begin(): void {
        let id: number
        id = wireReadDataByte(APDS9960_I2C_ADDR)

        /* Set ENABLE register to 0 (disable all features) */
        // ALL, OFF
        setMode(7, 0)

        /* Set default values for ambient light and proximity registers */
        // APDS9960_ATIME, DEFAULT_ATIME
        wireWriteDataByte(0x81, 219)

        // APDS9960_WTIME, DEFAULT_WTIME
        wireWriteDataByte(0x83, 246)

        //APDS9960_PPULSE, DEFAULT_PROX_PPULSE
        wireWriteDataByte(0x8E, 0x87)

        // APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR
        wireWriteDataByte(0x9D, 0)

        // APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL
        wireWriteDataByte(0x9E, 0)

        // APDS9960_CONFIG1, DEFAULT_CONFIG1
        wireWriteDataByte(0x8D, 0x60)

        // DEFAULT_LDRIVE
        setLEDDrive(0)

        // DEFAULT_PGAIN
        setProximityGain(2)

        // DEFAULT_AGAIN
        setAmbientLightGain(0)

        // DEFAULT_PILT
        setProxIntLowThresh(0)

        // DEFAULT_PIHT
        setProxIntHighThresh(50)

        // DEFAULT_AILT
        setLightIntLowThreshold(0xFFFF)

        // DEFAULT_AIHT
        setLightIntHighThreshold(0)

        // APDS9960_PERS, DEFAULT_PERS
        wireWriteDataByte(0x8C, 0x11)

        // APDS9960_CONFIG2, DEFAULT_CONFIG2
        wireWriteDataByte(0x90, 0x01)

        // APDS9960_CONFIG3, DEFAULT_CONFIG3
        wireWriteDataByte(0x9F, 0)

        // DEFAULT_GPENTH
        setGestureEnterThresh(40)

        // DEFAULT_GEXTH
        setGestureExitThresh(30)

        // APDS9960_GCONF1, DEFAULT_GCONF1
        wireWriteDataByte(0xA2, 0x40)

        // DEFAULT_GGAIN
        setGestureGain(2)

        // DEFAULT_GLDRIVE
        setGestureLEDDrive(0)

        // DEFAULT_GWTIME
        setGestureWaitTime(1)

        // APDS9960_GOFFSET_U, DEFAULT_GOFFSET
        wireWriteDataByte(0xA4, 0)

        // APDS9960_GOFFSET_D, DEFAULT_GOFFSET
        wireWriteDataByte(0xA5, 0)

        // APDS9960_GOFFSET_L, DEFAULT_GOFFSET
        wireWriteDataByte(0xA7, 0)

        // APDS9960_GOFFSET_R, DEFAULT_GOFFSET
        wireWriteDataByte(0xA9, 0)

        // APDS9960_GPULSE, DEFAULT_GPULSE
        wireWriteDataByte(0xA6, 0xC9)

        // APDS9960_GCONF3, DEFAULT_GCONF3
        wireWriteDataByte(0xAA, 0)

        // DEFAULT_GIEN
        setGestureIntEnable(0)

    }

    //% advanced=true
    //% group=Optional
    //% blockId=YFGesture_getMode
    //% block="get mode"
    function getMode(): number {
        let enable_value: number;
        /* Read current ENABLE register */
        // APDS9960_ENABLE
        enable_value = wireReadDataByte(0x80)
        return enable_value;
    }

    //% advanced=true
    //% group=Optional
    //% blockId=YFGesture_setMode
    //% block="set mode %mode %enable"
    function setMode(mode: NumberFormat.UInt8BE, enable: NumberFormat.UInt8BE): boolean {
        let reg_val: NumberFormat.UInt8BE;

        /* Read current ENABLE register */
        reg_val = getMode();
        // ERROR value
        if (reg_val == 0xFF) {
            return false;
        }

        /* Change bit(s) in ENABLE register */
        enable = enable & 0x01;
        if (mode >= 0 && mode <= 6) {
            if (enable) {
                reg_val |= (1 << mode);
            } else {
                reg_val &= ~(1 << mode);
            }
        } else if (mode == 7) {       // ALL mode
            if (enable) {
                reg_val = 0x7F;
            } else {
                reg_val = 0x00;
            }
        }

        /* Write value back to ENABLE register */
        // APDS9960_ENABLE
        wireWriteDataByte(0x80, reg_val)

        return true;
    }

    //% group=Optional
    //% blockId=YFGesture_enablePower
    //% block="enable power"
    function enablePower() {
        setMode(0, 1)
    }

    //% group=Optional
    //% blockId=YFGesture_disablePower
    //% block="disable power"
    function disbalePower() {
        setMode(0, 0)
    }

    function enableGestureSensor(): void {
        /* Enable gesture mode
           Set ENABLE to 0 (power off)
           Set WTIME to 0xFF
           Set AUX to LED_BOOST_300
           Enable PON, WEN, PEN, GEN in ENABLE 
        */
        resetGestureParameters();
        wireWriteDataByte(0x83, 0xFF)
        //APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE
        wireWriteDataByte(0x8E, 0x89)
        // LED_BOOST_300
        setLEDBoost(3)
        setGestureIntEnable(0)
        setGestureMode(1)
        enablePower()
        // WAIT
        setMode(3, 1)
        // PROXIMITY
        setMode(2, 1)
        // GESTURE
        setMode(6, 1)
    }

    function disableGestureSensor() {
        resetGestureParameters();
        setGestureIntEnable(0)
        setGestureMode(0)
        setMode(6, 0)
    }

    //% advanced=true
    //% group=Optional
    //% blockId=YFGesture_getLEDDRive
    //% block="get LED drive"
    function getLEDDrive() {
        let val: number;

        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)

        /* Shift and mask out LED drive bits */
        val = (val >> 6) & 0b00000011;

        return val;
    }

    //% advanced=true
    //% group=Optional
    //% blockId=YFGesture_setLEDDRive
    //% block="set LED drive %drive"
    function setLEDDrive(drive: NumberFormat.UInt8BE): void {
        let val: NumberFormat.UInt8BE = 0;

        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)

        /* Set bits in register to given value */
        drive &= 0b00000011;
        drive = drive << 6;
        val &= 0b00111111;
        val |= drive;

        /* Write register value back into CONTROL register */
        // APDS9960_CONTROL, val
        wireWriteDataByte(0x8F, val)
    }

    //% advanced=true
    //% group=Gesture
    //% blockId=YFGesture_getGestureLEDDrive
    //% block="get gesture LED drive"
    function getGestureLEDDrive() {
        let val: number;

        /* Read value from GCONF2 register */
        // APDS9960_GCONF2
        val = wireReadDataByte(0xA3)

        /* Shift and mask out GLDRIVE bits */
        val = (val >> 3) & 0b00000011;

        return val;
    }

    //% group=Gesture
    //% advanced=true
    //% blockId=YFGesture_setGestureLEDDrive
    //% block="set gesture LED drive %drive"
    function setGestureLEDDrive(drive: number) {
        let val: number;

        /* Read value from GCONF2 register */
        // APDS9960_GCONF2
        val = wireReadDataByte(0xA3)
        /* Set bits in register to given value */
        drive &= 0b00000011;
        drive = drive << 3;
        val &= 0b11100111;
        val |= drive;

        /* Write register value back into GCONF2 register */
        // APDS9960_GCONF2
        wireWriteDataByte(0xA3, val);
    }

    //% advanced=true
    //% group=Gesture
    //% blockId=YFGesture_getGestureGain
    //% block="get gesture gain"
    function getGestureGain() {
        let val: number;

        /* Read value from GCONF2 register */
        // APDS9960_GCONF2
        val = wireReadDataByte(0xA3)

        /* Shift and mask out GGAIN bits */
        val = (val >> 5) & 0b00000011;

        return val;
    }

    //% advanced=true
    //% group=Gesture
    //% blockId=YFGesture_setGestureGain
    //% block="set gesture gain %gain"
    function setGestureGain(gain: number) {
        let val: number;

        /* Read value from GCONF2 register */
        // APDS9960_GCONF2
        val = wireReadDataByte(0xA3)

        /* Set bits in register to given value */
        gain &= 0b00000011;
        gain = gain << 5;
        val &= 0b10011111;
        val |= gain;

        /* Write register value back into GCONF2 register */
        // APDS9960_GCONF2
        wireWriteDataByte(0xA3, val)
    }

    //% advanced=true
    //% group=Gesture
    //% blockId=YFGesture_getGestureIntEnable
    //% block="get gesture int enable"
    function getGestureIntEnable() {
        let val = 0;

        /* Read value from GCONF4 register */
        // APDS9960_GCONF4
        val = wireReadDataByte(0xAB)

        /* Shift and mask out GIEN bit */
        val = (val >> 1) & 0b00000001;

        return val;
    }

    //% group=Gesture
    //% advanced=true
    //% blockId=YFGesture_setGestureIntEnable
    //% block="set gesture int enable %enable"
    function setGestureIntEnable(enable: number): void {
        let val = 0;

        /* Read value from GCONF4 register */
        // APDS9960_GCONF4
        val = wireReadDataByte(0xAB)
        /* Set bits in register to given value */
        enable &= 0b00000001;
        enable = enable << 1;
        val &= 0b11111101;
        val |= enable;

        /* Write register value back into GCONF4 register */
        // APDS9960_GCONF4
        wireWriteDataByte(0xAB, val)
    }

    //% group=Gesture
    //% blockId=YFGesture_isGestureAvailable
    //% block="is gesture available"
    function isGestureAvailable() {
        let val = 0;

        /* Read value from GSTATUS register */
        // APDS9960_GSTATUS
        val = wireReadDataByte(0xAF)

        /* Shift and mask out GVALID bit */
        // APDS9960_GVALID
        val &= 0b00000001;

        /* Return true/false based on GVALID bit */
        if (val == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * read gesture
     */
    //% group=Gesture
    //% blockId="YFGesture_readGesture" weight=100 blockGap=15
    //% block="read gesture"
    //% gesture.fieldEditor="gridpicker" gesture.fieldOptions.columns=4
    export function gesture() {
        let fifo_level = 0;
        let bytes_read = 0;
        let gstatus: number;
        let fifo_data: number[] = []
        let motion: number;
        let i: number;
        let mode: number = getMode() & 0b01000001

        /* Make sure that power and gesture is on and data is valid */
        if (!isGestureAvailable() || !(mode)) {
            return;
        }

        /* Keep looping as long as gesture data is valid */
        while (1) {
            /* Wait some time to collect next batch of FIFO data */
            basic.pause(FIFO_PAUSE_TIME);       // FIFO_PAUSE_TIME

            /* Get the contents of the STATUS register. Is data still valid? */
            // APDS9960_GSTATUS
            gstatus = wireReadDataByte(0xAF);

            /* If we have valid data, read in FIFO */
            if ((gstatus & 0b00000001) == 0b00000001) {

                /* Read the current FIFO level */
                // APDS9960_GFLVL
                fifo_level = wireReadDataByte(0xAE)

                /* If there's stuff in the FIFO, read it into our data block */
                if (fifo_level > 0) {
                    //APDS9960_GFIFO_U
                    fifo_data = wireReadDataBlock(0xFC, (fifo_level * 4));

                    bytes_read = fifo_data.length

                    /* If at least 1 set of data, sort the data into U/D/L/R */
                    if (fifo_data.length >= 4) {
                        for (i = 0; i < bytes_read; i += 4) {
                            gesture_data_u_data[gesture_data_index] = fifo_data[i + 0];
                            gesture_data_d_data[gesture_data_index] = fifo_data[i + 1];
                            gesture_data_l_data[gesture_data_index] = fifo_data[i + 2];
                            gesture_data_r_data[gesture_data_index] = fifo_data[i + 3];
                            gesture_data_index++;
                            gesture_data_total_gestures++;
                        }

                        /* Filter and process gesture data. Decode near/far state */
                        if (processGestureData()) {
                            if (decodeGesture()) {
                            }
                        }

                        /* Reset data */
                        gesture_data_index = 0;
                        gesture_data_total_gestures = 0;
                    }
                }
            } else {

                /* Determine best guessed gesture and clean up */
                basic.pause(FIFO_PAUSE_TIME);
                decodeGesture();
                motion = gesture_motion_;
                motion_global = gesture_motion_
                resetGestureParameters();

                if (motion == DIR.DIR_UP) {
                    control.raiseEvent(5, 5)
                } else if (motion == DIR.DIR_DOWN) {
                    control.raiseEvent(5, 6)
                } else if (motion == DIR.DIR_RIGHT) {
                    control.raiseEvent(5, 7)
                } else if (motion == DIR.DIR_LEFT) {
                    control.raiseEvent(5, 8)
                } else if (motion == DIR.DIR_NEAR) {
                    control.raiseEvent(5, 9)
                } else if (motion == DIR.DIR_FAR) {
                    control.raiseEvent(5, 10)
                }
                return;
            }
        }

    }

    /**
     * Gesture sensor detection waving the action: no, upper, lower left, right, near, far.
     * @param u type of gesture to detect. eg: GESTURE_TYPE.UP
     * @param handler code to run
     */
    //% group=Gesture
    //% blockId="YFGesture_onGesture" weight=99 blockGap=15
    //% block="detection gesture |%u"
    //% u.fieldEditor="gridpicker" u.fieldOptions.columns=3
    export function onGesture(u: GESTURE_TYPE, handler: () => void) {
        if (u == GESTURE_TYPE.UP) {
            control.onEvent(5, 5, function () {
                handler()
            })
        }
        if (u == GESTURE_TYPE.DOWN) {
            control.onEvent(5, 6, function () {
                handler()
            })
        }
        if (u == GESTURE_TYPE.LEFT) {
            control.onEvent(5, 8, function () {
                handler()
            })
        }
        if (u == GESTURE_TYPE.RIGHT) {
            control.onEvent(5, 7, function () {
                handler()
            })
        }
        if (u == GESTURE_TYPE.NEAR) {
            control.onEvent(5, 9, function () {
                handler()
            })
        }
        if (u == GESTURE_TYPE.FAR) {
            control.onEvent(5, 10, function () {
                handler()
            })
        }
    }

    /**
     * Read the value of the gesture, no gesture: 0; up: 1; right: 2; down: 3; left: 4; near: 5; far: 6
     */
    //% group=Gesture
    //% blockId="YFGesture_getGestureID" weight=10 blockGap=15
    //% block="get gesture ID"
    export function getGestureID(): number {
        let dir_id: number = 0
        if (motion_global == DIR.DIR_UP) {
            dir_id = 1
        } else if (motion_global == DIR.DIR_RIGHT) {
            dir_id = 2
        } else if (motion_global == DIR.DIR_DOWN) {
            dir_id = 3
        } else if (motion_global == DIR.DIR_LEFT) {
            dir_id = 4
        } else if (motion_global == DIR.DIR_NEAR) {
            dir_id = 5
        } else if (motion_global == DIR.DIR_FAR) {
            dir_id = 6
        } else {
            dir_id = 0
        }
        return dir_id
    }

    function decodeGesture(): boolean {
        /* Return if near or far event is detected */
        if (gesture_state_ == STATE.NEAR_STATE) {
            gesture_motion_ = DIR.DIR_NEAR;
            return true;
        } else if (gesture_state_ == STATE.FAR_STATE) {
            gesture_motion_ = DIR.DIR_FAR;
            return true;
        }

        /* Determine swipe direction */
        if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 0)) {
            gesture_motion_ = DIR.DIR_UP;
        } else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 0)) {
            gesture_motion_ = DIR.DIR_DOWN;
        } else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 1)) {
            gesture_motion_ = DIR.DIR_RIGHT;
        } else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == -1)) {
            gesture_motion_ = DIR.DIR_LEFT;
        } else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 1)) {
            if (Math.abs(gesture_ud_delta_) > Math.abs(gesture_lr_delta_)) {
                gesture_motion_ = DIR.DIR_UP;
            } else {
                gesture_motion_ = DIR.DIR_RIGHT;
            }
        } else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == -1)) {
            if (Math.abs(gesture_ud_delta_) > Math.abs(gesture_lr_delta_)) {
                gesture_motion_ = DIR.DIR_DOWN;
            } else {
                gesture_motion_ = DIR.DIR_LEFT;
            }
        } else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == -1)) {
            if (Math.abs(gesture_ud_delta_) > Math.abs(gesture_lr_delta_)) {
                gesture_motion_ = DIR.DIR_UP;
            } else {
                gesture_motion_ = DIR.DIR_LEFT;
            }
        } else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 1)) {
            if (Math.abs(gesture_ud_delta_) > Math.abs(gesture_lr_delta_)) {
                gesture_motion_ = DIR.DIR_DOWN;
            } else {
                gesture_motion_ = DIR.DIR_RIGHT;
            }
        } else {
            return false;
        }
        return true;
    }

    function processGestureData(): boolean {
        let u_first = 0;
        let d_first = 0;
        let l_first = 0;
        let r_first = 0;
        let u_last = 0;
        let d_last = 0;
        let l_last = 0;
        let r_last = 0;
        let ud_ratio_first = 0;
        let lr_ratio_first = 0;
        let ud_ratio_last = 0;
        let lr_ratio_last = 0;
        let ud_delta = 0;
        let lr_delta = 0;
        let i = 0;

        /* If we have less than 4 total gestures, that's not enough */
        if (gesture_data_total_gestures <= 4) {
            return false;
        }

        /* Check to make sure our data isn't out of bounds */
        if ((gesture_data_total_gestures <= 32) &&
            (gesture_data_total_gestures > 0)) {

            /* Find the first value in U/D/L/R above the threshold */
            for (i = 0; i < gesture_data_total_gestures; i++) {
                // GESTURE_THRESHOLD_OUT
                if ((gesture_data_u_data[i] > 10) && (gesture_data_d_data[i] > 10) &&
                    (gesture_data_l_data[i] > 10) && (gesture_data_r_data[i] > 10)) {
                    u_first = gesture_data_u_data[i];
                    d_first = gesture_data_d_data[i];
                    l_first = gesture_data_l_data[i];
                    r_first = gesture_data_r_data[i];
                    break;
                }
            }

            /* If one of the _first values is 0, then there is no good data */
            if ((u_first == 0) || (d_first == 0) || (l_first == 0) || (r_first == 0)) {
                return false;
            }

            /* Find the last value in U/D/L/R above the threshold */
            for (i = gesture_data_total_gestures - 1; i >= 0; i--) {

                if ((gesture_data_u_data[i] > 10) && (gesture_data_d_data[i] > 10) &&
                    (gesture_data_l_data[i] > 10) && (gesture_data_r_data[i] > 10)) {
                    u_last = gesture_data_u_data[i];
                    d_last = gesture_data_d_data[i];
                    l_last = gesture_data_l_data[i];
                    r_last = gesture_data_r_data[i];
                    break;
                }
            }
        }

        /* Calculate the first vs. last ratio of up/down and left/right */
        ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
        lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
        ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
        lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

        /* Determine the difference between the first and last ratios */
        ud_delta = ud_ratio_last - ud_ratio_first;
        lr_delta = lr_ratio_last - lr_ratio_first;

        /* Accumulate the UD and LR delta values */
        gesture_ud_delta_ += ud_delta;
        gesture_lr_delta_ += lr_delta;

        /* Determine U/D gesture */
        // GESTURE_SENSITIVITY_1
        if (gesture_ud_delta_ >= 50) {
            gesture_ud_count_ = 1;
        } else if (gesture_ud_delta_ <= -50) {
            gesture_ud_count_ = -1;
        } else {
            gesture_ud_count_ = 0;
        }

        /* Determine L/R gesture */
        if (gesture_lr_delta_ >= 50) {
            gesture_lr_count_ = 1;
        } else if (gesture_lr_delta_ <= -50) {
            gesture_lr_count_ = -1;
        } else {
            gesture_lr_count_ = 0;
        }

        /* Determine Near/Far gesture */
        if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 0)) {
            // GESTURE_SENSITIVITY_2
            if ((Math.abs(ud_delta) < 20) &&
                (Math.abs(lr_delta) < 20)) {

                if ((ud_delta == 0) && (lr_delta == 0)) {
                    gesture_near_count_++;
                } else if ((ud_delta != 0) || (lr_delta != 0)) {
                    gesture_far_count_++;
                }

                if ((gesture_near_count_ >= 10) && (gesture_far_count_ >= 2)) {
                    if ((ud_delta == 0) && (lr_delta == 0)) {
                        gesture_state_ = STATE.NEAR_STATE;
                    } else if ((ud_delta != 0) && (lr_delta != 0)) {
                        gesture_state_ = STATE.FAR_STATE;
                    }
                    return true;
                }
            }
        } else {
            // GESTURE_SENSITIVITY_2
            if ((Math.abs(ud_delta) < 20) && (Math.abs(lr_delta) < 20)) {

                if ((ud_delta == 0) && (lr_delta == 0)) {
                    gesture_near_count_++;
                }

                if (gesture_near_count_ >= 10) {
                    gesture_ud_count_ = 0;
                    gesture_lr_count_ = 0;
                    gesture_ud_delta_ = 0;
                    gesture_lr_delta_ = 0;
                }
            }
        }
        return false;
    }

    /**
     * Gesture enable work mode.
     * @param u mode. eg: GESTURE_WORKMODE.GESTURE_MODE
     */
    //% group="Mode"
    //% blockId=YFGesture_enable_mode weight=100 blockGap=15
    //% block="enable %u"
    export function enable_mode(u: GESTURE_WORKMODE): void {
        if (u == GESTURE_WORKMODE.LIGHT_MODE) {
            disableProximitySensor()
            disableGestureSensor()
            enableLightSensor()
        } else if (u == GESTURE_WORKMODE.PROXIMITY_MODE) {
            disableLightSensor()
            disableGestureSensor()
            enableProximitySensor()
        } else if (u == GESTURE_WORKMODE.GESTURE_MODE) {
            disableLightSensor()
            disableProximitySensor()
            enableGestureSensor()
        }
    }

    function enableProximitySensor(): void {
        /* Set default gain, LED, interrupts, enable power, and enable sensor */
        // DEFAULT_PGAIN
        setProximityGain(2)
        // DEFAULT_LDRIVE
        setLEDDrive(0)
        setProximityIntEnable(0)
        enablePower()
        setMode(2, 1)
    }

    function disableProximitySensor(): void {
        setProximityIntEnable(0)
        setMode(2, 0)
    }

    function setProximityIntEnable(enable: number): void {
        let val: number;

        /* Read value from ENABLE register */
        // APDS9960_ENABLE
        val = wireReadDataByte(0x80)

        /* Set bits in register to given value */
        enable &= 0b00000001;
        enable = enable << 5;
        val &= 0b11011111;
        val |= enable;

        /* Write register value back into ENABLE register */
        // APDS9960_ENABLE
        wireWriteDataByte(0x80, val)
    }

    //% advanced=true
    //% group="Proximity"
    //% blockId=YFGesture_getProximityGain
    //% block="get proximity gain"
    function getProximityGain(): number {
        let val = 0;

        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)

        /* Shift and mask out PDRIVE bits */
        val = (val >> 2) & 0b00000011;

        return val;
    }

    //% advanced=true
    //% group="Proximity"
    //% blockId=YFGesture_setProximityGain
    //% block="set proximity gain %drive"
    function setProximityGain(drive: NumberFormat.UInt8BE): void {
        let val: number;
        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)

        /* Set bits in register to given value */
        drive &= 0b00000011;
        drive = drive << 2;
        val &= 0b11110011;
        val |= drive;
        /* Write register value back into CONTROL register */
        // APDS9960_CONTROL
        wireWriteDataByte(0x8F, val)
    }

    /**
     * Gesture get Proximity value.
     */
    //% group="Proximity"
    //% blockId=YFGesture_getProximity weight=100 blockGap=15
    //% block="get proximity"
    export function proximity() {
        let val: number = 0;
        /* Read value from proximity data register */
        val = wireReadDataByte(0x9C)    // APDS9960_PDATA
        return val
    }

    function enableLightSensor(): void {
        /* Set default gain, interrupts, enable power, and enable sensor */
        setAmbientLightGain(0)
        setAmbientLightIntEnable(0)
        enablePower()
        // AMBIENT_LIGHT
        setMode(1, 1)
    }

    function disableLightSensor(): void {
        setAmbientLightIntEnable(0)
        // AMBIENT_LIGHT
        setMode(1, 0)
    }

    //% advanced=true
    //% group="Light"
    //% blockId=YFGesture_getAmbientLightGain
    //% block="get ambient light gain"
    function getAmbientLightGain(): number {
        let val: number;
        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)
        /* Shift and mask out ADRIVE bits */
        val &= 0b00000011;
        return val;
    }

    //% advanced=true
    //% group="Light"
    //% blockId=YFGesture_setAmbientLightGain
    //% block="set ambient light gain %drive"
    function setAmbientLightGain(drive: number): void {
        let val: number;
        /* Read value from CONTROL register */
        // APDS9960_CONTROL
        val = wireReadDataByte(0x8F)
        /* Set bits in register to given value */
        drive &= 0b00000011;
        val &= 0b11111100;
        val |= drive;
        /* Write register value back into CONTROL register */
        // APDS9960_CONTROL
        wireWriteDataByte(0x8F, val)
    }

    //% advanced=true
    //% group="Light"
    //% blockId=YFGesture_clearAmbientLightInt
    //% block="clear ambient light int"
    function clearAmbientLightInt(): void {
        let throwaway: number;
        // APDS9960_AICLEAR
        throwaway = wireReadDataByte(0xE7)
    }

    /**
     * Gesture Ambient Light.
     * @param u mode. eg: LIGHT_UNIT.LUX
     */
    //% group="Light"
    //% blockId=YFGesture_getAmbientLight weight=100 blockGap=15
    //% block="illuminance %u"
    //% u.fieldEditor="gridpicker" u.fieldOptions.columns=2
    export function ambientLight(u: LIGHT_UNIT): number {
        let val_byte: number;
        let val: number = 0;
        /* Read value from clear channel, low byte register */
        // APDS9960_CDATAL
        val_byte = wireReadDataByte(0x94)
        val = val_byte;
        /* Read value from clear channel, high byte register */
        // APDS9960_CDATAH
        val_byte = wireReadDataByte(0x95)
        val = val + (val_byte << 8);
        if (u == LIGHT_UNIT.FC)
            val = val / 10.764
        return val
    }

    /**
     * Gesture Get Red Light.
     */
    //% group="Light"
    //% blockId=YFGesture_getRedLight weight=90 blockGap=15
    //% block="red light"
    export function redLight(): number {
        let val_byte: number;
        let val: number = 0;
        /* Read value from clear channel, low byte register */
        // APDS9960_RDATAL
        val_byte = wireReadDataByte(0x96)
        val = val_byte;
        /* Read value from clear channel, high byte register */
        val_byte = wireReadDataByte(0x97)
        val = val + (val_byte << 8);
        return val;
    }

    /**
     * Gesture Get Green Light.
     */
    //% group="Light"
    //% blockId=YFGesture_getGreenLight weight=85 blockGap=15
    //% block="green light"
    export function greenLight(): number {
        let val_byte: number;
        let val: number = 0;
        /* Read value from clear channel, low byte register */
        // APDS9960_GDATAL
        val_byte = wireReadDataByte(0x98)
        val = val_byte;
        /* Read value from clear channel, high byte register */
        // APDS9960_GDATAH
        val_byte = wireReadDataByte(0x99)
        val = val + (val_byte << 8);
        return val;
    }

    /**
     * Gesture Get Blue Light.
     */
    //% group="Light"
    //% blockId=YFGesture_getBlueLight weight=80 blockGap=15
    //% block="blue light"
    export function blueLight(): number {
        let val_byte: number;
        let val: number = 0;
        /* Read value from clear channel, low byte register */
        // APDS9960_BDATAL
        val_byte = wireReadDataByte(0x9A)
        val = val_byte;
        /* Read value from clear channel, high byte register */
        // APDS9960_BDATAH
        val_byte = wireReadDataByte(0x9B)
        val = val + (val_byte << 8);
        return val;
    }

    function setProxIntLowThresh(threshold: number) {
        // APDS9960_PILT
        wireWriteDataByte(0x89, threshold)
    }

    function setProxIntHighThresh(threshold: number) {
        // APDS9960_PIHT
        wireWriteDataByte(0x8B, threshold)
    }

    function setLightIntLowThreshold(threshold: number) {
        let val_low: number;
        let val_high: number;
        /* Break 16-bit threshold into 2 8-bit values */
        val_low = threshold & 0x00FF;
        val_high = (threshold & 0xFF00) >> 8;
        /* Write low byte */
        // APDS9960_AILTL
        if (!wireWriteDataByte(0x84, val_low)) {
            return false;
        }
        /* Write high byte */
        // APDS9960_AILTH
        if (!wireWriteDataByte(0x85, val_high)) {
            return false;
        }
        return true;
    }

    function setLightIntHighThreshold(threshold: number) {
        let val_low: number;
        let val_high: number;
        /* Break 16-bit threshold into 2 8-bit values */
        val_low = threshold & 0x00FF;
        val_high = (threshold & 0xFF00) >> 8;
        /* Write low byte */
        // APDS9960_AIHTL
        if (!wireWriteDataByte(0x86, val_low)) {
            return false;
        }
        /* Write high byte */
        // APDS9960_AIHTH
        if (!wireWriteDataByte(0x87, val_high)) {
            return false;
        }
        return true;
    }

    function setGestureEnterThresh(threshold: number): void {
        // APDS9960_GPENTH
        wireWriteDataByte(0xA0, threshold)
    }

    function setGestureExitThresh(threshold: number): void {
        // APDS9960_GEXTH
        wireWriteDataByte(0xA1, threshold)
    }

    function setGestureWaitTime(time: number) {
        let val: number;

        /* Read value from GCONF2 register */
        // APDS9960_GCONF2
        val = wireReadDataByte(0xA3)

        /* Set bits in register to given value */
        time &= 0b00000111;
        val &= 0b11111000;
        val |= time;

        /* Write register value back into GCONF2 register */
        // APDS9960_GCONF2
        wireWriteDataByte(0xA3, val)
    }

    function setLEDBoost(boost: number) {
        let val: number;

        /* Read value from CONFIG2 register */
        // APDS9960_CONFIG2
        val = wireReadDataByte(0x90)

        /* Set bits in register to given value */
        boost &= 0b00000011;
        boost = boost << 4;
        val &= 0b11001111;
        val |= boost;

        /* Write register value back into CONFIG2 register */
        // APDS9960_CONFIG2
        wireWriteDataByte(0x90, val)
    }

    function setGestureMode(mode: number) {
        let val: number;

        /* Read value from GCONF4 register */
        // APDS9960_GCONF4
        val = wireReadDataByte(0xAB)

        /* Set bits in register to given value */
        mode &= 0b00000001;
        val &= 0b11111110;
        val |= mode;

        /* Write register value back into GCONF4 register */
        // APDS9960_GCONF4
        wireWriteDataByte(0xAB, val);
    }

    function resetGestureParameters() {
        gesture_data_index = 0;
        gesture_data_total_gestures = 0;

        gesture_ud_delta_ = 0;
        gesture_lr_delta_ = 0;

        gesture_ud_count_ = 0;
        gesture_lr_count_ = 0;

        gesture_near_count_ = 0;
        gesture_far_count_ = 0;

        gesture_state_ = 0;
        gesture_motion_ = DIR.DIR_NONE;
    }

    function setAmbientLightIntEnable(enable: number): void {
        let val: number;
        /* Read value from ENABLE register */
        // APDS9960_ENABLE
        val = wireReadDataByte(0x80)
        /* Set bits in register to given value */
        enable &= 0b00000001;
        enable = enable << 4;
        val &= 0b11101111;
        val |= enable;
        /* Write register value back into ENABLE register */
        // APDS9960_ENABLE
        wireWriteDataByte(0x80, val)
    }

    function wireWriteByte(val: NumberFormat.UInt8BE): boolean {
        pins.i2cWriteNumber(APDS9960_I2C_ADDR, val, NumberFormat.UInt8BE)
        return true;
    }

    function wireWriteDataByte(reg: number, val: number): boolean {
        let buf = pins.createBuffer(2)
        buf[0] = reg;
        buf[1] = val;
        pins.i2cWriteBuffer(APDS9960_I2C_ADDR, buf)
        return true;
    }

    function wireReadDataByte(reg: number): number {
        pins.i2cWriteNumber(APDS9960_I2C_ADDR, reg, NumberFormat.UInt8BE);
        let val: number = pins.i2cReadNumber(APDS9960_I2C_ADDR, NumberFormat.UInt8BE)
        return val
    }

    function wireReadDataBlock(reg: NumberFormat.UInt8BE, len: number): number[] {
        let buff: number[] = []
        pins.i2cWriteNumber(APDS9960_I2C_ADDR, reg, NumberFormat.UInt8BE);
        for (let i = 0; i < len; i++) {
            buff[i] = pins.i2cReadNumber(APDS9960_I2C_ADDR, NumberFormat.UInt8BE)
        }
        return buff
    }

    begin();
}