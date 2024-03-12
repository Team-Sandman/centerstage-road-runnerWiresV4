package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hang {
    private DcMotor par1;
    private DcMotor launch_motor;

    public void inithang( HardwareMap hwMap) {
        par1 = hwMap.get(DcMotor.class, "par1");
        par1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        par1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        par1.setDirection(DcMotorSimple.Direction.FORWARD);

        launch_motor = hwMap.get(DcMotor.class, "par0");
        launch_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch_motor.setDirection(DcMotorSimple.Direction.FORWARD);


    }
    public void hang(double speed) {

        par1.setPower(speed);
        launch_motor.setPower(speed);
    }    }


