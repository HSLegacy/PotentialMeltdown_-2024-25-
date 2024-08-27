// BadBlueBackstage

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

package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "BlueAudienceYellowPixel", group = "Concept")
public class BlueAudienceYellowPixel extends LinearOpMode {

    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private Servo launch = null;
    private DcMotor intake = null;
    private DcMotor Rarm = null;
    private DcMotor Larm = null;
    private Servo bar = null;
    private Servo LClaw = null;
    private Servo RClaw = null;
    private DcMotor sArm = null;
    private Servo Funnel = null;
    private Servo Door = null;
    private CRServo Outake = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.45;
    static final double     TURN_SPEED              = 0.3;
    static final double     SLOW_SPEED = 0.3;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "Blue_Box.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red_hat.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue box",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initTfod();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        launch = hardwareMap.get(Servo.class, "launch");
        intake = hardwareMap.get(DcMotor.class, "intake");
        Rarm = hardwareMap.get(DcMotor.class, "Rarm");
        Larm = hardwareMap.get(DcMotor.class, "Larm");
        bar = hardwareMap.get(Servo.class, "bar");
        RClaw = hardwareMap.get(Servo.class, "RClaw");
        LClaw = hardwareMap.get(Servo.class, "LClaw");
        sArm = hardwareMap.get(DcMotor.class, "sArm");
        Funnel = hardwareMap.get(Servo.class, "Funnel");
        Door = hardwareMap.get(Servo.class, "Door");
        Outake = hardwareMap.get(CRServo.class, "Outake");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        Rarm.setDirection(DcMotor.Direction.FORWARD);
        Larm.setDirection(DcMotor.Direction.REVERSE);
        sArm.setDirection(DcMotor.Direction.FORWARD);

        Larm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        //---------------------Left Trajectories-----------------

        Pose2d startPose = new Pose2d(-60, -60, Math.toRadians(90));

        Trajectory left_traj1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-40, 15), Math.toRadians(0))
                .build();
        TrajectorySequence left_trajTurn1 = drive.trajectorySequenceBuilder(left_traj1.end())
                .turn(Math.toRadians(93))
                .waitSeconds(6)
                .build();
        Trajectory left_traj2 = drive.trajectoryBuilder(left_trajTurn1.end())
                .forward(3)
                .build();
        Trajectory left_traj3 = drive.trajectoryBuilder(left_traj2.end())
                .back(6)
                .build();
        /*TrajectorySequence left_trajTurn2 = drive.trajectorySequenceBuilder(left_traj3.end())
                .turn(Math.toRadians(90))
                .build();*/
        Trajectory left_traj4 = drive.trajectoryBuilder(left_traj3.end())
                .strafeRight(23)
                .build();
        Trajectory left_traj5 = drive.trajectoryBuilder(left_traj4.end())
                .forward(72)
                .build();
        Trajectory left_traj6 = drive.trajectoryBuilder(left_traj5.end())
                .strafeLeft(28)
                .build();
        Trajectory left_traj7 = drive.trajectoryBuilder(left_traj6.end())
                .forward(15)
                .build();
        TrajectorySequence arm_left = drive.trajectorySequenceBuilder(left_traj7.end())
                .addTemporalMarker(1, () -> {
                    Door.setPosition(1);
                })
                .addTemporalMarker(0.3, () -> {
                    Outake.setPower(-.23);
                })
                .addTemporalMarker(1, () -> {
                    Outake.setPower(0);
                })
                .waitSeconds(2.5)
                .build();
        Trajectory left_traj8 = drive.trajectoryBuilder(arm_left.end())
                .strafeRight(24)
                .addTemporalMarker(0.5, () -> {
                    Door.setPosition(0);
                })
                .build();

        // -------------------Middle Trajectories-------------
        Trajectory middle_traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        Trajectory middle_traj2 = drive.trajectoryBuilder(middle_traj1.end())
                .back(8)
                .build();
        /*TrajectorySequence middle_trajTurn1 = drive.trajectorySequenceBuilder(middle_traj1.end())
                .turn(Math.toRadians(-90))
                .build();*/
        Trajectory middle_traj3 = drive.trajectoryBuilder(middle_traj2.end())
                .strafeRight(15)
                .build();
        /*TrajectorySequence middle_trajTurn2 = drive.trajectorySequenceBuilder(middle_traj3.end())
                .turn(Math.toRadians(-90))
                .build();*/
        Trajectory middle_traj4 = drive.trajectoryBuilder(middle_traj3.end())
                .forward(28)
                .build();
        TrajectorySequence middle_trajTurn1 = drive.trajectorySequenceBuilder(middle_traj4.end())
                .turn(Math.toRadians(89))
                .waitSeconds(5)
                .build();
        Trajectory middle_traj5 = drive.trajectoryBuilder(middle_trajTurn1.end())
                .forward(82)
                .build();
        Trajectory middle_traj6 = drive.trajectoryBuilder(middle_traj5.end())
                .strafeLeft(28)
                .build();
        Trajectory middle_traj7 = drive.trajectoryBuilder(middle_traj6.end())
                .forward(17)
                .build();
        TrajectorySequence arm_middle = drive.trajectorySequenceBuilder(middle_traj7.end())
                .addTemporalMarker(1, () -> {
                    Door.setPosition(1);
                })
                .addTemporalMarker(0.3, () -> {
                    Outake.setPower(-.23);
                })
                .addTemporalMarker(1, () -> {
                    Outake.setPower(0);
                })
                .waitSeconds(2.5)
                .build();
        Trajectory middle_traj8 = drive.trajectoryBuilder(arm_middle.end())
                .strafeRight(23)
                .addTemporalMarker(0.5, () -> {
                    Door.setPosition(0);
                })
                .build();

        // -------------------- Right Trajectories -----------
        Trajectory right_traj1 = drive.trajectoryBuilder(new Pose2d(-60, -60, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d( -40,15), Math.toRadians(0))
                .build();
        /*TrajectorySequence right_trajTurn1 = drive.trajectorySequenceBuilder(right_traj1.end())
                .turn(Math.toRadians(-90))
                .waitSeconds(17)
                .build();*/
        Trajectory right_traj2 = drive.trajectoryBuilder(right_traj1.end())
                .forward(26)
                .build();
        Trajectory right_traj3 = drive.trajectoryBuilder(right_traj2.end())
                .back(6)
                .build();
        /*TrajectorySequence right_trajTurn2 = drive.trajectorySequenceBuilder(right_traj3.end())
                .turn(Math.toRadians(-90))
                .build();*/
        Trajectory right_traj4 = drive.trajectoryBuilder(right_traj3.end())
                .strafeLeft(12)
                .build();
        /*TrajectorySequence right_trajTurn3 = drive.trajectorySequenceBuilder(right_traj4.end())
                .turn(Math.toRadians(-90))
                .build();*/
        Trajectory right_traj5 = drive.trajectoryBuilder(right_traj4.end())
                .forward(31)
                .build();
        TrajectorySequence right_trajTurn1 = drive.trajectorySequenceBuilder(right_traj5.end())
                .turn(Math.toRadians(87.5))
                .waitSeconds(5)
                .build();
        Trajectory right_traj6 = drive.trajectoryBuilder(right_trajTurn1.end())
                .forward(72)
                .build();
        Trajectory right_traj7 = drive.trajectoryBuilder(right_traj6.end())
                .strafeLeft(17)
                .build();
        Trajectory right_traj8 = drive.trajectoryBuilder(right_traj7.end())
                .forward(12)
                .build();
        TrajectorySequence arm_right = drive.trajectorySequenceBuilder(right_traj8.end())
                .addTemporalMarker(1, () -> {
                    Door.setPosition(1);
                })
                .addTemporalMarker(0.3, () -> {
                    Outake.setPower(-.23);
                })
                .addTemporalMarker(1, () -> {
                    Outake.setPower(0);
                })
                .waitSeconds(2.5)
                .build();
        Trajectory right_traj9 = drive.trajectoryBuilder(arm_right.end())
                .strafeRight(18)
                .addTemporalMarker(0.5, () -> {
                    Door.setPosition(0);
                })
                .build();

        // Camera Stuff Don't edit

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();





                if (spikeLocation() == 1) {

                    drive.followTrajectory(left_traj1);
                    drive.followTrajectorySequence(left_trajTurn1);
                    drive.followTrajectory(left_traj2);
                    drive.followTrajectory(left_traj3);
                    //drive.followTrajectorySequence(left_trajTurn2);
                    drive.followTrajectory(left_traj4);
                    //drive.followTrajectorySequence(left_trajTurn3);
                    drive.followTrajectory(left_traj5);
                    drive.followTrajectory(left_traj6);
                    armUp(1350);
                    smallUp(1040);
                    drive.followTrajectory(left_traj7);
                    drive.followTrajectorySequence(arm_left);
                    armDown(1350);
                    smallDown(1040);
                    drive.followTrajectory(left_traj8);

                    sleep(100000);


                } else if (spikeLocation() == 2) {

                    drive.followTrajectory(middle_traj1);
                    drive.followTrajectory(middle_traj2);
                    //drive.followTrajectorySequence(middle_trajTurn1);
                    drive.followTrajectory(middle_traj3);
                    //drive.followTrajectorySequence(middle_trajTurn2);
                    drive.followTrajectory(middle_traj4);
                    drive.followTrajectorySequence(middle_trajTurn1);
                    drive.followTrajectory(middle_traj5);
                    drive.followTrajectory(middle_traj6);
                    armUp(1350);
                    smallUp(1040);
                    drive.followTrajectory(middle_traj7);
                    drive.followTrajectorySequence(arm_middle);
                    armDown(1350);
                    smallDown(1040);
                    drive.followTrajectory(middle_traj8);



                    sleep(100000);



                } else {

                    drive.followTrajectory(right_traj1);
                    //drive.followTrajectorySequence(right_trajTurn);
                    //drive.followTrajectorySequence(right_trajTurn1);
                    /*drive.followTrajectory(right_traj2);
                    //drive.followTrajectorySequence(right_trajTurn2);
                    drive.followTrajectory(right_traj3);
                    //drive.followTrajectorySequence(right_trajTurn3);
                    drive.followTrajectory(right_traj4);
                    drive.followTrajectory(right_traj5);
                    drive.followTrajectorySequence(right_trajTurn1);
                    drive.followTrajectory(right_traj6);
                    drive.followTrajectory(right_traj7);
                    armUp(1350);
                    smallUp(1040);
                    drive.followTrajectory(right_traj8);
                    drive.followTrajectorySequence(arm_right);
                    armDown(1350);
                    smallDown(1040);
                    drive.followTrajectory(right_traj9);
                    //armpose(-4);*/





                    sleep(100000);

                }



                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;



            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


    }   // end method telemetryTfod()

    /*public void intake(String mode, double power){
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (mode == "intake"){
            Intake.setPower(power);
        }

        if (mode == "outtake"){
            Intake.setPower(-power);
        }
        if (mode == "stop"){
            Intake.setPower(0);
        }

    }*/
    private double spikeLocation() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        double location = 3;

        for (Recognition recognition : currentRecognitions) {

            if (recognition.getLeft() <= 263) {
                location = 1;
                telemetry.addData("Spike mark location: ", "left");
            } else if (recognition.getLeft() > 263) {
                location = 2;
                telemetry.addData("Spike mark location: ", "middle");
            } else {
                location = 3;
                telemetry.addData("Spike mark location: ", "right");
            }

        }   // end for() loop

        return location;
    }

    /*
        public void armDown(double distance, double power) {

            //Reset Encoders
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            slide.setPower(-power);


            while (-slide.getCurrentPosition() < distance) {
                telemetry.addData("Arm Encoder", slide.getCurrentPosition());
                telemetry.update();
            }

            slide.setPower(0);

            sleep(500);

        }


        public void armUp(double power, String mode) {

            //Reset Encoders
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slide.setPower(power);
            if (mode == "Up"){
                slide.setPower(power);
            }

            if (mode == "Down"){
                slide.setPower(-power);
            }
            if (mode == "stop"){
                slide.setPower(0);
            }



            slide.setPower(0);

            sleep(1000);

        }*/
    public void encoderDriveArm(double speed,
                                double Larminches, double Rarminches) {
        int newLarmtarget;
        int newRarmtarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLarmtarget = Larm.getCurrentPosition() + (int)(Larminches * COUNTS_PER_INCH);
        newRarmtarget = Rarm.getCurrentPosition() + (int)(Rarminches * COUNTS_PER_INCH);


        Larm.setTargetPosition(newLarmtarget);
        Rarm.setTargetPosition(newRarmtarget);


        // Turn On RUN_TO_POSITION
        Larm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // reset the timeout time and start motion
        Larm.setPower(Math.abs(speed));
        Rarm.setPower(Math.abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (Larm.isBusy() && Rarm.isBusy())) {

            // Display it for the driver.
            telemetry.update();
        }

        // Stop all motion;
        Larm.setPower(0);
        Rarm.setPower(0);


        // Turn off RUN_TO_POSITION
        Larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    private void runEncoder(DcMotor motor, int newMotorTarget, double speed){

        // Ensure that the opmode is still active

        motor.setTargetPosition(newMotorTarget);

        // Turn On RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        motor.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && motor.isBusy()) {

            // Display it for the driver.
            telemetry.update();
        }

        // Stop all motion;
        motor.setPower(0);

        // Turn off RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*public void armUp(double inches){
        encoderDriveArm(DRIVE_SPEED, -inches, -inches);
    }
    public void armDown(double inches){
        encoderDriveArm(SLOW_SPEED, inches, inches);
    }*/
    public void armpose(int pose){
        double ticks = 22.76;
        double armAngle = Larm.getCurrentPosition() / ticks - 25;
        while (armAngle != pose) {
            armAngle = Larm.getCurrentPosition() / ticks - 25;
            if (pose < armAngle + 1 && pose > armAngle - 1) {// Stop arm movement within a 4 degree range
                Larm.setPower(0);
                Rarm.setPower(0);
                break;

            } else if (pose > armAngle + 8 || pose < armAngle - 8) {//  Far and fast arm move into position within an infinite range
                if (pose < armAngle) {
                    Larm.setPower(1);
                    Rarm.setPower(1);
                }
                if (pose > armAngle) {
                    Larm.setPower(-1);
                    Rarm.setPower(-1);
                }

            } else { //Close and slow arm move into position if arm is in a 16 degree range
                if (pose < armAngle) {
                    Larm.setPower(.2);
                    Rarm.setPower(.2);
                }
                if (pose > armAngle) {
                    Larm.setPower(-.2);
                    Rarm.setPower(-.2);
                }

            }
        }
    }
    public void armUp(double distance) {

        //Reset Encoders
        Rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Rarm.setPower(-1);
        Larm.setPower(-1);

        while (-Rarm.getCurrentPosition() < distance) {
            telemetry.addData("Arm Encoder", Rarm.getCurrentPosition());
            telemetry.update();
        }

        Rarm.setPower(0);
        Larm.setPower(0);

        // sleep(500);

    }
    public void armDown(double distance) {

        //Reset Encoders
        Rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Rarm.setPower(1);
        Larm.setPower(1);

        while (Rarm.getCurrentPosition() < distance) {
            telemetry.addData("Arm Encoder", Rarm.getCurrentPosition());
            telemetry.update();
        }

        Rarm.setPower(0);
        Larm.setPower(0);

        // sleep(500);

    }
    public void smallUp(double distance) {

        //Reset Encoders
        sArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sArm.setPower(-1);

        while (-sArm.getCurrentPosition() < distance) {
            telemetry.addData("smallArm Encoder", sArm.getCurrentPosition());
            telemetry.update();
        }

        sArm.setPower(0);

        //sleep(500);

    }
    public void smallDown(double distance) {

        //Reset Encoders
        sArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sArm.setPower(1);

        while (sArm.getCurrentPosition() < distance) {
            telemetry.addData("smallArm Encoder", sArm.getCurrentPosition());
            telemetry.update();
        }

        sArm.setPower(0);

        // sleep(500);

    }
} // end class
