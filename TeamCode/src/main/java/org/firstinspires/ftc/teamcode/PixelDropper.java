package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelDropper {
    private Servo Pixel_Dropper;
    private Servo Lincoln_Linkage;

    public void initPixel_Servo(HardwareMap hwMap) {


        Pixel_Dropper = hwMap.get(Servo.class, "Pixel_Dropper");
        Lincoln_Linkage = hwMap.get(Servo.class, "Linkage");
    }
    public void pixelDrop(){
        Pixel_Dropper.setPosition(.65); //was.5

    }

    public void pixelDropClose(){
        Pixel_Dropper.setPosition(1);
    }

    public void linkageForward(){
        Lincoln_Linkage.setPosition(0.6);

    }

    public void linkageBackward(){
        Lincoln_Linkage.setPosition(1);
    }

}
