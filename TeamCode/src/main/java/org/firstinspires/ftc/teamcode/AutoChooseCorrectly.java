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

@Autonomous(name = "FTC Wires Autonomous Mode (Choose Correctly)", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class AutoChooseCorrectly extends LinearOpMode {

    /* Copyright (c) 2019 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *  b
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

    /**
     * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
     */


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
        public enum START_POSITION {
            BLUE_LEFT,
            BLUE_RIGHT,
            RED_LEFT,
            RED_RIGHT
        }

        public enum PARK_POSITION {
            INSIDE,
            OUTSIDE
        }

        public static PARK_POSITION parkPosition;
        public static START_POSITION startPosition;

        public enum IDENTIFIED_SPIKE_MARK_LOCATION {
            LEFT,
            MIDDLE,
            RIGHT
        }

        public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

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


        public void runAutonoumousMode() {
            //Initialize Pose2d as desired
            Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
            Pose2d moveBeyondTrussPose = new Pose2d(0, 0, 0);
            Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
            Pose2d midwayPose1 = new Pose2d(0, 0, 0);
            Pose2d midwayPose1a = new Pose2d(0, 0, 0);
            Pose2d intakeStack = new Pose2d(0, 0, 0);
            Pose2d midwayPose2 = new Pose2d(0, 0, 0);
            Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
            Pose2d parkPose = new Pose2d(0, 0, 0);
            Pose2d parkPoseFinal = new Pose2d(0, 0, 0);
            double waitSecondsBeforeDrop = 0;

            MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);


            PerryThePlatypusIntake intake = new PerryThePlatypusIntake();
            intake.initIntake(hardwareMap); //added to see if can get intake to work
            Lift lift = new Lift();
            PixelDropper pixelDrop = new PixelDropper();

            boolean PixelDropperOpen = false;

            lift.initLiftAuto(hardwareMap);

            pixelDrop.initPixel_Servo(hardwareMap);

            initPose = new Pose2d(0, 0, Math.toRadians(180)); //Starting pose
            moveBeyondTrussPose = new Pose2d(15, 0, Math.toRadians(180));

            switch (startPosition) {
                case BLUE_LEFT:
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch (identifiedSpikeMarkLocation) {
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(25.5, 8, Math.toRadians(180));//x was 25.5 y was 8
                            dropYellowPixelPose = new Pose2d(21.6, 38.9, Math.toRadians(90)); //y was 37.8 meet 3 y was 38.4
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(29, 3, Math.toRadians(180));
                            dropYellowPixelPose = new Pose2d(28.5, 38.9, Math.toRadians(90));//x originally 30, y was 37.8, meet 3 y was 38.4 match 3 x was 29.5 x was 30.5
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(26.8, -7, Math.toRadians(124));//x was 30  y was -9 mtr was -45
                            dropYellowPixelPose = new Pose2d(36, 38.9, Math.toRadians(90));//x was 31.5 y was 37.8  //for meet 3 x was 32 y was 38.4

                            break;
                    }
                    switch (parkPosition) {
                        case INSIDE:
                            parkPose = new Pose2d(50.7, 30.7, Math.toRadians(90));
                            parkPoseFinal = new Pose2d(51.9, 43.5, Math.toRadians(90));
                            break;
                        case OUTSIDE:
                            parkPose = new Pose2d(3.5, 31.5, Math.toRadians(90));
                            parkPoseFinal = new Pose2d(2.8, 43.5, Math.toRadians(90));
                            break;
                    }
                    midwayPose1 = new Pose2d(14, 13, Math.toRadians(90)); //was -45 in mathtoradians was 45
                    waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board was 2

                    break;

                case RED_RIGHT:
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch (identifiedSpikeMarkLocation) {
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(30, 9, Math.toRadians(225));
                            dropYellowPixelPose = new Pose2d(34, -40.6, Math.toRadians(267)); //was 267 y was -39
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(28, -1, Math.toRadians(180));//Origin x was 30
                            dropYellowPixelPose = new Pose2d(28.4, -40.6, Math.toRadians(267));//x was 29 heading was 90 y was -39 x was 27
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(26, -9, Math.toRadians(180));
                            dropYellowPixelPose = new Pose2d(20, -40.6, Math.toRadians(267)); //y was -39
                            break;
                    }
                    switch (parkPosition) {
                        case INSIDE:
                            parkPose = new Pose2d(50.3, -28.5, Math.toRadians(270));
                            parkPoseFinal = new Pose2d(52.9, -45, Math.toRadians(270));
                            break;
                        case OUTSIDE:
                            parkPose = new Pose2d(2.4, -33.4, Math.toRadians(270));
                            parkPoseFinal = new Pose2d(2.3, -45, Math.toRadians(270));
                            break;
                    }

                    midwayPose1 = new Pose2d(14, -13, Math.toRadians(267));
                    waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                    break;

                case BLUE_RIGHT:
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch (identifiedSpikeMarkLocation) {
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(26.5, 9, Math.toRadians(225)); //y was 9.5
                            dropYellowPixelPose = new Pose2d(24, 94, Math.toRadians(90));//was 27,86,-90 y was 91.4 x was 21.6 y was 92.8
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(29.5, 0, Math.toRadians(180)); //y was -3
                            dropYellowPixelPose = new Pose2d(31.5, 94.3, Math.toRadians(90));//y was 91.4 x was 29.5 y was 92.8
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(24, -8.5, Math.toRadians(180));
                            dropYellowPixelPose = new Pose2d(37, 94.3, Math.toRadians(90));//y was 91.4 //x was 32.2 x was 36
                            break;
                    }
                    switch (parkPosition) {
                        case INSIDE:
                            parkPose = new Pose2d(52.8, 87.1, Math.toRadians(90));
                            parkPoseFinal = new Pose2d(52.8, 96.3, Math.toRadians(90));

                            midwayPose1 = new Pose2d(14, -18, Math.toRadians(180));
                            midwayPose1a = new Pose2d(52, -20, Math.toRadians(180));
                            intakeStack = new Pose2d(52, 72, Math.toRadians(180));
                            midwayPose2 = new Pose2d(28, 75, Math.toRadians(90));//had -62 for y x was 28

                            break;
                        case OUTSIDE:
                            parkPose = new Pose2d(3.4, 87.1, Math.toRadians(90));
                            parkPoseFinal = new Pose2d(2, 96.3, Math.toRadians(90)); //x was 3.4

                            midwayPose1 = new Pose2d(4, 0, Math.toRadians(180));
                            midwayPose1a = new Pose2d(4, 50, Math.toRadians(180));
                            intakeStack = new Pose2d(15, 75, Math.toRadians(90));
                            midwayPose2 = new Pose2d(25, 78, Math.toRadians(90));//had -62 for y
                            break;
                    }
                  //  midwayPose1 = new Pose2d(3, -3, Math.toRadians(180));//was 3,-3,180 for purple pixel only
                 //   midwayPose1a = new Pose2d(18, -18, Math.toRadians(90));//was 18,-18, 90
                   // intakeStack = new Pose2d(3, 1, Math.toRadians(90));
                  //  midwayPose2 = new Pose2d(3, 62, Math.toRadians(90));//was 50,62,90
                    waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                    //parkPose = new Pose2d(30, 91.4, Math.toRadians(90));//was 50,84,90
                    break;

                case RED_LEFT:
                    drive = new MecanumDrive(hardwareMap, initPose);
                    switch (identifiedSpikeMarkLocation) {
                        case LEFT:
                            dropPurplePixelPose = new Pose2d(27.5, 9.8, Math.toRadians(180));
                            dropYellowPixelPose = new Pose2d(34, -92.6, Math.toRadians(273));//was 37,-86,90 y was -91.2 also was 34, -92.1 for meet 3 field
                            break;
                        case MIDDLE:
                            dropPurplePixelPose = new Pose2d(30, 0, Math.toRadians(180));
                            dropYellowPixelPose = new Pose2d(2, -92.6, Math.toRadians(267));//y was -91.2 also was -92.1 for meet 3 field x was 28.4
                            break;
                        case RIGHT:
                            dropPurplePixelPose = new Pose2d(25.5, -6, Math.toRadians(112)); //y was -8
                            dropYellowPixelPose = new Pose2d(20, -92.6, Math.toRadians(267));//y was -91.2 was -92.1 for meet 3 field

                            break;
                    }
                    switch (parkPosition) {
                        case INSIDE:
                            parkPose = new Pose2d(48.6, -84.9, Math.toRadians(270));
                            parkPoseFinal = new Pose2d(49, -98, Math.toRadians(270));

                            midwayPose1 = new Pose2d(14, 19.2, Math.toRadians(180)); //y was 18
                            midwayPose1a = new Pose2d(52, 20, Math.toRadians(180));
                            intakeStack = new Pose2d(54, -72, Math.toRadians(180));
                            midwayPose2 = new Pose2d(28, -75, Math.toRadians(270));//had -62 for y //was 267 x was 28

                            break;
                        case OUTSIDE:
                            parkPose = new Pose2d(3, -86.7, Math.toRadians(270));
                            parkPoseFinal = new Pose2d(3, -98, Math.toRadians(270));

                            midwayPose1 = new Pose2d(4, 0, Math.toRadians(180));//was 3, -3
                            midwayPose1a = new Pose2d(5, -50, Math.toRadians(180));//was 18, 18
                            intakeStack = new Pose2d(14, -75, Math.toRadians(270)); //was 267
                            midwayPose2 = new Pose2d(27, -75, Math.toRadians(270));//had -62 for y //was 267


                            break;
                    }

                   // midwayPose1 = new Pose2d(3, -3, Math.toRadians(180));
                   // midwayPose1a = new Pose2d(18, 18, Math.toRadians(267));
                  //  intakeStack = new Pose2d(3, 1, Math.toRadians(267));
                  //  midwayPose2 = new Pose2d(4, -70, Math.toRadians(267));//had -62 for y


                    waitSecondsBeforeDrop = 1; //TODO: Adjust time to wait for alliance partner to move from board
                   // parkPose = new Pose2d(3, -91.4, Math.toRadians(267));
                    break;
            }

            //Move robot to dropPurplePixel based on identified Spike Mark Location
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                            .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                            .build());

            //TODO : Code to drop Purple Pixel on Spike Mark

            safeWaitSeconds(.1); //was 1
            intake.intake(.2);

            //pixelDrop.linkageForward();
            //lift.autoLift(-
            //                    740);  //Try this here to save time, comment out from below when you do
            safeWaitSeconds(.75); // was 2 was 1
            intake.intake(0);


            //Move robot to midwayPose1
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());

            //For Blue Right and Red Left, intake pixel from stack
            if (startPosition == START_POSITION.BLUE_RIGHT ||
                    startPosition == START_POSITION.RED_LEFT) {
                safeWaitSeconds(.1); //was 1
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                                .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                                .build());

                //TODO : Code to intake pixel from stack
                safeWaitSeconds(.1);

                //Move robot to midwayPose2 and to dropYellowPixelPose
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                                .build());
            }

            //added to see if linkage will extend further
            safeWaitSeconds(.2); //was .2
            pixelDrop.linkageForward();
            safeWaitSeconds(waitSecondsBeforeDrop);

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            //.splineToLinearHeading(dropYellowPixelPose,0)
                            .strafeToConstantHeading(new Vector2d(dropYellowPixelPose.position.x, dropYellowPixelPose.position.y))//Reduce the rotation until paralell to backdrop.
                            .build());


            //TODO : Code to drop Pixel on Backdrop


            safeWaitSeconds(.1); //was 1
            lift.autoLift(-510);//was -550 was -530

            safeWaitSeconds(3); //was 3
            pixelDrop.pixelDrop();

            safeWaitSeconds(1);


            pixelDrop.pixelDropClose();
            //lift.autoLift(0);
            safeWaitSeconds(1); //was 1
            pixelDrop.linkageBackward();
            safeWaitSeconds(1);


            //lift.lift(.5 * gamepad2.right_stick_y); //We need to run using encoder
            //Move robot to park in Backstage
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .strafeToLinearHeading(parkPoseFinal.position, parkPoseFinal.heading)
                            //.splineToLinearHeading(parkPose,0)
                            .build());
        }


        //Method to select starting position using X, Y, A, B buttons on gamepad
        public void selectStartingPosition() {
            telemetry.setAutoClear(true);
            telemetry.clearAll();
            //******select start pose*****
            while (!isStopRequested()) {
                telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                        TEAM_NAME, " ", TEAM_NUMBER);
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:", "");
                telemetry.addData("    Blue Left   ", "(X / ▢)");
                telemetry.addData("    Blue Right ", "(Y / Δ)");
                telemetry.addData("    Red Left    ", "(B / O)");
                telemetry.addData("    Red Right  ", "(A / X)");
                if (gamepad1.x) {
                    startPosition = START_POSITION.BLUE_LEFT;
                    break;
                }
                if (gamepad1.y) {
                    startPosition = START_POSITION.BLUE_RIGHT;
                    break;
                }
                if (gamepad1.b) {
                    startPosition = START_POSITION.RED_LEFT;
                    break;
                }
                if (gamepad1.a) {
                    startPosition = START_POSITION.RED_RIGHT;
                    break;
                }
                telemetry.update();
            }
            while (!isStopRequested()) {
                telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                        TEAM_NAME, " ", TEAM_NUMBER);
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Select Parking Position using rb/lb on gamepad 1:", "");
                telemetry.addData("    INSIDE   ", "(rb)");
                telemetry.addData("    OUTSIDE ", "(lb)");

                if (gamepad1.right_bumper) {
                    parkPosition = PARK_POSITION.INSIDE;
                    break;
                }
                if (gamepad1.left_bumper) {
                    parkPosition = PARK_POSITION.OUTSIDE;
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
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.BLUE_LEFT) {
                    if (recognition.getLabel() == "Pixel" || recognition.getLabel() == "BlueElement" || recognition.getLabel() == "RedElement") {
                        if (x < 200) {
                            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        } else {
                            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        }
                    }
                } else { //RED_RIGHT or BLUE_RIGHT
                    if (recognition.getLabel() == "Pixel" || recognition.getLabel() == "BlueElement" || recognition.getLabel() == "RedElement") {
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

