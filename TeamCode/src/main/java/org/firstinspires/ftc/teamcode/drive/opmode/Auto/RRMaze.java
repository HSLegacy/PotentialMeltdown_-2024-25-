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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
@Autonomous(name = "RRmaze", group = "Concept")
public class RRMaze extends LinearOpMode {

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
    private static final String TFOD_MODEL_ASSET = "Red_Box.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red_hat.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "red box",
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


        Pose2d startPose = new Pose2d(-60, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory left_traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40, 15), Math.toRadians(0))
                .build();
        Trajectory left_trajTurn1 = drive.trajectoryBuilder(left_traj1.end())
                .splineToConstantHeading(new Vector2d(-10, -60), Math.toRadians(-90))
                .build();
        Trajectory left_traj2 = drive.trajectoryBuilder(left_trajTurn1.end())
                .lineToConstantHeading(new Vector2d(50, -15))
                .build();
        Trajectory left_traj3 = drive.trajectoryBuilder(left_traj2.end())
                .lineToConstantHeading(new Vector2d(60, 56))
                .build();
        Trajectory left_traj4 = drive.trajectoryBuilder(left_traj3.end())
                .splineToConstantHeading(new Vector2d(30, 35), Math.toRadians(-120))
                .build();
        Trajectory left_traj5 = drive.trajectoryBuilder(left_traj4.end())
                .splineToConstantHeading(new Vector2d(13, 57), Math.toRadians(0))
                .build();
        Trajectory left_traj6 = drive.trajectoryBuilder(left_traj5.end())
                .lineToConstantHeading(new Vector2d(-53, 57))
                .build();
        Trajectory left_traj7 = drive.trajectoryBuilder(left_traj6.end())
                .splineToConstantHeading(new Vector2d(-63, 39), Math.toRadians(-120))
                .build();


        /*Trajectory mazeTraj = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-40, 15), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(180)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(50, -15))
                /*
                .lineToConstantHeading(new Vector2d(60, 56))
                .splineToConstantHeading(new Vector2d(30, 35), Math.toRadians(-120))
                .splineToConstantHeading(new Vector2d(13, 57), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-53, 57))
                .splineToConstantHeading(new Vector2d(-63, 39), Math.toRadians(-120))


                .build();
                */
        if(opModeIsActive()) {

            drive.followTrajectory(left_traj1);
            drive.followTrajectory(left_trajTurn1);
            /*
            drive.followTrajectory(left_traj2);
            drive.followTrajectory(left_traj3); 
            drive.followTrajectory(left_traj4);
            drive.followTrajectory(left_traj5);
            drive.followTrajectory(left_traj6);
            drive.followTrajectory(left_traj7);
            */
        }







            /*Trajectory mazeTraj =   drive.trajectorySequenceBuilder(new Pose2d(-60, -60, Math.toRadians(90)))



                                .splineToConstantHeading(new Vector2d( -40,15), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(180)), Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(50, -15))
                                .lineToConstantHeading(new Vector2d(60, 56))
                                .splineToConstantHeading(new Vector2d(30, 35),Math.toRadians(-120))
                                .splineToConstantHeading(new Vector2d(13,57), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(-53, 57))
                                .splineToConstantHeading(new Vector2d(-63, 39),Math.toRadians(-120))






                                .build();*/
    }
}// end class
