/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires Autonomous Mode (New)", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class FTCWiresAutonomousNew extends LinearOpMode {

    public static String TEAM_NAME = "Team Sandman"; //: Enter team Name
    public static int TEAM_NUMBER = 16312; //: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //Begin copy from ConceptTensorflowObjectDetection
    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    // private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_RedAndBlueElement = "model_20231205_170537.tflite";
    //private static final String TFOD_RED_ELEMENT = "model_20231203_222209.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "BlueElement",
            //  "Pixel",
            "RedElement",

    };

    //end copy from ConceptTensorflowObjectDetection

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public enum PARK_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;
    public static PARK_POSITION parkPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
    public Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
    public Pose2d parkPose = new Pose2d(0, 0, 0); // Parking Pose

    @Override
    public void runOpMode() throws InterruptedException {

        //Key Pad inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses TensorFlow for pixel detection
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Parking Position", parkPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    //Static Locations
    Pose2d CENTER_FIELD = new Pose2d(0,0,Math.toRadians(180));
    Pose2d RED_RIGHT_START = new Pose2d(15.98,-61.35,Math.toRadians(270));
    Pose2d RED_LEFT_START = new Pose2d(0.0,0.0,Math.toRadians(270));    //Need Actual Coordinates
    Pose2d BLUE_RIGHT_START = new Pose2d(0.0,0.0,Math.toRadians(90));   //Need Actual Coordinates
    Pose2d BLUE_LEFT_START = new Pose2d(0.0,0.0,Math.toRadians(90));    //Need Actual Coordinates
    Pose2d PARK_RED_RIGHT = new Pose2d(65.79,-58.53,Math.toRadians(180));
    Pose2d PARK_RED_LEFT = new Pose2d(62.69,-9.35,Math.toRadians(180));
    Pose2d PARK_BLUE_RIGHT = new Pose2d(63.18,13.20,Math.toRadians(180));
    Pose2d PARK_BLUE_LEFT = new Pose2d(63.53,60.78,Math.toRadians(180));
    Pose2d RED_BACKDROP_CENTER = new Pose2d(53.14,-34.66,Math.toRadians(180));  //Guesstimate based on inverse of BLUE
    Pose2d BLUE_BACKDROP_CENTER = new Pose2d(53.14,34.66,Math.toRadians(180));

    public void runAutonoumousMode() {


        double waitSecondsBeforeDrop = 0;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        PerryThePlatypusIntake intake = new PerryThePlatypusIntake();

        intake.initIntake(hardwareMap); //added to see if can get intake to work

        Lift lift = new Lift();

        PixelDropper pixelDrop = new PixelDropper();

        boolean PixelDropperOpen = false;

        lift.initLiftAuto(hardwareMap);

        pixelDrop.initPixel_Servo(hardwareMap);

        drive = new MecanumDrive(hardwareMap, initPose);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(CENTER_FIELD.position, CENTER_FIELD.heading)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

    }


    //Method to select starting & parking positions using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepad1.x){
                //Start position BLUE_LEFT
                startPosition = START_POSITION.BLUE_LEFT;
                initPose = BLUE_LEFT_START;
                boolean parkingSelected = false;
                telemetry.clearAll();
                do {
                    //Set BLUE parking
                    sleep(500);
                    telemetry.addData("Select Parking Position", "");
                    telemetry.addData("    Park BLUE Left   ", "(X / ▢)");
                    telemetry.addData("    Park BLUE Right   ", "(B / O)");
                    if (gamepad1.x) {
                        parkPosition = PARK_POSITION.BLUE_LEFT;
                        parkPose = PARK_BLUE_LEFT;
                        parkingSelected=true;
                        //break;
                    }
                    if (gamepad1.b) {
                        parkPosition = PARK_POSITION.BLUE_RIGHT;
                        parkPose = PARK_BLUE_RIGHT;

                        parkingSelected=true;
                        //break;
                    }
                    telemetry.update();
                }while(parkingSelected==false);
                break;
            }
            if(gamepad1.y){
                //Start position BLUE_RIGHT
                startPosition = START_POSITION.BLUE_RIGHT;
                initPose = BLUE_RIGHT_START;
                boolean parkingSelected = false;
                telemetry.clearAll();
                do {
                    //Set BLUE parking
                    sleep(500);
                    telemetry.addData("Select Parking Position", "");
                    telemetry.addData("    Park BLUE Left   ", "(X / ▢)");
                    telemetry.addData("    Park BLUE Right   ", "(B / O)");
                    if (gamepad1.x) {
                        parkPosition = PARK_POSITION.BLUE_LEFT;
                        parkPose = PARK_BLUE_LEFT;
                        parkingSelected=true;
                        //break;
                    }
                    if (gamepad1.b) {
                        parkPosition = PARK_POSITION.BLUE_RIGHT;
                        parkPose = PARK_BLUE_RIGHT;
                        parkingSelected=true;
                        //break;
                    }
                    telemetry.update();
                }while(parkingSelected==false);
                break;
            }
            if(gamepad1.b){
                //Start position RED_LEFT
                startPosition = START_POSITION.RED_LEFT;
                initPose = RED_LEFT_START;
                boolean parkingSelected = false;
                telemetry.clearAll();
                do {
                    //Set RED parking
                    sleep(500);
                    telemetry.addData("Select Parking Position", "");
                    telemetry.addData("    Park RED Left   ", "(X / ▢)");
                    telemetry.addData("    Park RED Right   ", "(B / O)");
                    if (gamepad1.x) {
                        parkPosition = PARK_POSITION.RED_LEFT;
                        parkPose = PARK_RED_LEFT;
                        parkingSelected=true;
                        //break;
                    }
                    if (gamepad1.b) {
                        parkPosition = PARK_POSITION.RED_RIGHT;
                        parkPose = PARK_RED_RIGHT;
                        parkingSelected=true;
                        //break;
                    }
                    telemetry.update();
                }while(parkingSelected==false);
                break;
            }
            if(gamepad1.a){
                //Start position RED_RIGHT
                startPosition = START_POSITION.RED_RIGHT;
                initPose = RED_RIGHT_START;
                boolean parkingSelected = false;
                telemetry.clearAll();
                do {
                    //Set RED parking
                    sleep(500);
                    telemetry.addData("Select Parking Position","");
                    telemetry.addData("    Park RED Left   ", "(X / ▢)");
                    telemetry.addData("    Park RED Right   ", "(B / O)");
                    if(gamepad1.x){
                        parkPosition = PARK_POSITION.RED_LEFT;
                        parkPose = PARK_RED_LEFT;
                        parkingSelected=true;
                        //break;
                    }
                    if(gamepad1.b){
                        parkPosition = PARK_POSITION.RED_RIGHT;
                        parkPose = PARK_RED_RIGHT;
                        parkingSelected=true;
                        //break;
                    }
                    telemetry.update();
                } while (parkingSelected==false);
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }


    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        // tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        // if (USE_WEBCAM) {
        //     visionPortal = VisionPortal.easyCreateWithDefaults(
        //         hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        //  } else {
        //     visionPortal = VisionPortal.easyCreateWithDefaults(
        //        BuiltinCameraDirection.BACK, tfod);
        // }

        // Set confidence threshold for TFOD recognitions, at any time.
        // tfod.setMinResultConfidence(0.2f); //was 0.095f

        //code from conceptTensorFlowObjectDetection to use our model
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_RedAndBlueElement)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.enableLiveView(true);

        builder.addProcessor(tfod);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);


        //end of code from ConceptTensorFlowObjectDetection





    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_LEFT and BLUE_LEFT If pixel not visible, assume Right spike Mark
        if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.BLUE_LEFT) {
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
        } else { //RED_RIGHT or BLUE_RIGHT
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.BLUE_LEFT) {
                if (recognition.getLabel() == "Pixel"|| recognition.getLabel() =="BlueElement" ||recognition.getLabel() =="RedElement") {
                    if (x < 200) {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    } else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    }
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                if (recognition.getLabel() == "Pixel" || recognition.getLabel() =="BlueElement" ||recognition.getLabel() =="RedElement") {
                    if (x < 300) // was <200 250 worked
                    {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    } else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    }
                }
            }

        }   // end for() loop

    }   // end method runTfodTensorFlow()

}   // end class
