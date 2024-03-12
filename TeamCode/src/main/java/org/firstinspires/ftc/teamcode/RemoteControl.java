package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.teamcode.MechanumDriveTrain;
import org.firstinspires.ftc.teamcode.PerryThePlatypusIntake;
//import org.firstinspires.ftc.teamcode.AirplaneLauncher;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class RemoteControl extends OpMode {

    MechanumDriveTrain drive = new MechanumDriveTrain();
    PerryThePlatypusIntake  intake = new PerryThePlatypusIntake();
    //AirplaneLauncher launch = new AirplaneLauncher();

    PixelDropper pixelDrop = new PixelDropper();

    Lights light = new Lights();

    Lift lift = new Lift();

    hang hang= new hang();

    private NormalizedColorSensor scanny;

    boolean PixelDropperOpen = false;

    float Red;
    float Green;
    float Blue;


    @Override
    public void init () {
        drive.initMechanumDrivetrainTeleOp(hardwareMap);
        intake.initIntake(hardwareMap);
        //launch.initLaunchMotor(hardwareMap);
        lift.initLift(hardwareMap);
        hang.inithang(hardwareMap);
        pixelDrop.initPixel_Servo(hardwareMap);
        scanny = hardwareMap.get(NormalizedColorSensor.class,"color_sensor");
        light.initLights(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            drive.mechanumDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x);
        } else {
            drive.mechanumDrive(drive.strafeMultiplier(gamepad1.right_stick_x), drive.forwardMultiplier(gamepad1.right_stick_y) + drive.forwardMultiplier(gamepad1.left_trigger) - drive.forwardMultiplier(gamepad1.right_trigger), drive.turnMultiplier(gamepad1.left_stick_x));
        }
        telemetry.addData("Strafe", gamepad1.right_stick_x);
        telemetry.addData("Forward", gamepad1.right_stick_y);
        telemetry.addData("Turn", gamepad1.left_stick_x);

        intake.intake(gamepad2.right_trigger - gamepad2.left_trigger);


        NormalizedRGBA colors = scanny.getNormalizedColors();



        Red = colors.red;
        Green = colors.green;
        Blue = colors.blue;

        //intake.ColorPixel(colors.red,colors.green, colors.blue);



        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f",colors.green);
        telemetry.addData("Blue","%.3f", colors.blue);
        telemetry.addData("pixel",intake.ColorPixel(colors.red,colors.green, colors.blue) );
        telemetry.addData("Lift", lift.liftPosition());
        if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="purple" ){
           light.Purple();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="yellow" ){
            light.Yellow();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="green" ){
            light.Green();
        }
        else if (intake.ColorPixel(colors.red,colors.green, colors.blue)=="white" ){
            light.White();
        }

        else {light.Black();}


        //launch.launch(gamepad2.left_stick_y);

        //lift.lift(.5 * gamepad2.right_stick_y);
        if (lift.liftPosition() <-2000 ){
            if (gamepad2.right_stick_y < 0){
                lift.lift(0);
            }
            else{
                lift.lift(.5 * gamepad2.right_stick_y);
            }

        }
        else if (lift.liftPosition() >0 ){
            if (gamepad2.right_stick_y > 0){
                lift.lift(0);
            }
            else{
                lift.lift(.5 * gamepad2.right_stick_y);
            }

        }
        else {
            lift.lift(.5 *gamepad2.right_stick_y);
        }


        hang.hang(.75 * gamepad2.left_stick_x);

        if (gamepad2.dpad_up){
            hang.hang(.5); //was .3
        }
        else if (gamepad2.dpad_down){
            hang.hang(0);
        }

        if (gamepad2.right_bumper) {
            pixelDrop.pixelDrop();

        }
        else {
            pixelDrop.pixelDropClose();
        }

        if (gamepad2.left_bumper) {
            pixelDrop.linkageForward();

        }
        else {
            pixelDrop.linkageBackward();
        }

      /* if (gamepad2.a){
           if (PixelDropperOpen==false) {

               pixelDrop.pixelDrop();

               PixelDropperOpen=true;

           }
            else {
               PixelDropperOpen = false;
               pixelDrop.pixelDropClose();

           }
       } */



    }

}
