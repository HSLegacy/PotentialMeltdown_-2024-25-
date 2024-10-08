package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(625);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54.044, 45, Math.toRadians(215.55007908148409), Math.toRadians(275), 12)
                .setDimensions(17, 17)
                .setStartPose(new Pose2d(-60, -60, Math.toRadians(90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -60, Math.toRadians(90)))


                                .splineToConstantHeading(new Vector2d( -40,15), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-10, -60), Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(50, -15))
                                .lineToConstantHeading(new Vector2d(60, 56))
                                .splineToConstantHeading(new Vector2d(30, 35),Math.toRadians(-120))
                                .splineToConstantHeading(new Vector2d(13,57), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(-53, 57))
                                .splineToConstantHeading(new Vector2d(-63, 39),Math.toRadians(-120))










                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.999f)
                .addEntity(myBot)
                .start();
    }
}