package org.firstinspires.ftc.teamcode.Main.HelperClasses.Stats;

/*
Information for using encoders
 */

class EncoderStats {
    static final double COUNTS_PER_MOTOR_REV  = 2240; //
    static final double DRIVE_GEAR_Reduction  = 1; //
    static final double WHEEL_DIAMETER_INCHES = 10; //10 cm, 100mm wheels
    static final double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV *DRIVE_GEAR_Reduction) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_ANGLE      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_Reduction) / (2 * Math.PI);
}
