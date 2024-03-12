package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class PerryThePlatypusIntake {
    private DcMotor intake_motor;


    public void initIntake(HardwareMap hwMap) {
        intake_motor = hwMap.get(DcMotor.class, "perp");
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void intake(double speed) {

        intake_motor.setPower(speed);

    }

    //read data from color sensor and return a value
    public String ColorPixel(double red, double green, double blue) {
        if (red > 0.027 && red < 0.048 && green > 0.052 && green < 0.082 &&
                blue > 0.061 && blue < 0.12) {


            return "purple";
        }
        else if (red > 0.036 && red < 0.045 && green > 0.068 && green < 0.082 &&
                blue > 0.046 && blue < 0.053) {
            return "yellow";
        }
        else if (red > 0.021 && red < 0.029 && green > 0.060 && green < 0.078 &&
                    blue > 0.049 && blue < 0.060) {
            return "green";
        }
        else if (red > 0.063 && red < 0.080 && green > 0.129 && green < 0.164 &&
                blue > 0.142 && blue < 0.180) {
            return "white";}

        else{
            return "none";
        }

// if (isInRange(red, 0.024, 0.028) && isInRange(green, 0.052, 0.057) && isInRange(blue, 0.060, 0.074)) {
//            return "purple";

    }


}


