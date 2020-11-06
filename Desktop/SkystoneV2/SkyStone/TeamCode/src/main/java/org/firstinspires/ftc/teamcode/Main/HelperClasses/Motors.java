package org.firstinspires.ftc.teamcode.Main.HelperClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors {
    private static DcMotor[] motors;

    public static void setMotors(DcMotor[] m) {
        motors = m;
    }

    public static DcMotor[] get() {
        return motors;
    }
}
