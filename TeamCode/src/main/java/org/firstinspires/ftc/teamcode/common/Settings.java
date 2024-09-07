package org.firstinspires.ftc.teamcode.common;


public class Settings extends Object {

    //*********************************************************************************************
    // Rev product constants go HERE.   They always should Start with REV_
    public static final int REV_CORE_HEX_MOTOR_TICKS_PER_REV = 288;
    public static final int REV_HD_40_MOTOR_TICKS_PER_REV = 1120;
    public static final int REV_HD_20_MOTOR_TICKS_PER_REV = 560;
    public static final double REV_MAX_POWER = 1.0;
    public static final double REV_MIN_POWER = -1.0;
    public static final int REV_HD_HEX_MOTOR_TICKS_PER_REV = 28;
    public static final double JOYSTICK_DEADBAND_TRIGGER = 0.1;
    public static final double JOYSTICK_DEADBAND_STICK = 0.1;

    //----------------------------------------------------------------------------------------------
    // Safety Management
    //
    // These constants manage the duration we allow for callbacks to user code to run for before
    // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
    // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
    // necessary as poorly chosen values might inadvertently compromise safety.
    //----------------------------------------------------------------------------------------------
    public static final int msStuckDetectInit = 17000;
    public static final int msStuckDetectInitLoop = 8000;
    public static final int msStuckDetectStart = 8000;
    public static final int msStuckDetectLoop = 5000;
    public static final int msStuckDetectStop = 1000;


    public enum JOYSTICK_SWITCH_NAMES {
        bumper_left,
        bumper_right,
        dpad_up,
        dpad_down,
        dpad_right,
        dpad_left,
        a,
        b,
        x,
        y,
        unknown
    }

    public enum JOYSTICK_THROTTLE_NAMES {
        right_stick_x,
        right_stick_y,
        left_stick_x,
        left_stick_y,
        left_trigger,
        right_trigger,
        unknown
    }
}
