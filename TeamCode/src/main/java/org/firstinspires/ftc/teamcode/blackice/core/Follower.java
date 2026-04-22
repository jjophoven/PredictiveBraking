package org.firstinspires.ftc.teamcode.blackice.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blackice.utils.RobotLogTelemetry;
import org.firstinspires.ftc.teamcode.blackice.core.controllers.PDController;
import org.firstinspires.ftc.teamcode.blackice.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackice.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackice.localizers.Localizer;
import org.firstinspires.ftc.teamcode.blackice.localizers.LocalizerConfig;

import java.util.function.DoubleSupplier;

public class Follower {
    /**
     * Responsible for turning the robot and making sure it is facing the correct
     * direction.
     */
    public final PDController headingController;

    public final Drivetrain drivetrain;
    public final Localizer localizer;
    public Telemetry telemetry;
    double lastTime = System.nanoTime();
    DoubleSupplier voltageSupplier;
    private double deltaTime;
    
    public Follower(PDController headingController,
                    DrivetrainConfig drivetrain, LocalizerConfig localizer,
                    HardwareMap hardwareMap) {
        this.headingController = headingController;
        this.drivetrain = drivetrain.build(hardwareMap);
        this.localizer = localizer.build(hardwareMap);
        this.drivetrain.zeroPowerFloatMode();
        this.voltageSupplier = () -> {
            double minV = Double.POSITIVE_INFINITY;
            for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                double v = vs.getVoltage();
                if (v > 0) minV = Math.min(minV, v);
            }
            return Math.max(9.0, Math.min(14.5, minV));
        };
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }
    
    public double getDeltaTime() {
        return deltaTime;
    }
    
    public void stop() {
        drivetrain.zeroPowerBrakeMode();
        drivetrain.zeroPower();
    }
    
    public void setTelemetry(Telemetry telemetry, boolean includeRobotLog) {
        this.telemetry =
            new MultipleTelemetry(this.telemetry, telemetry);
        
        if (includeRobotLog) {
            this.telemetry =
                new MultipleTelemetry(this.telemetry,
                                      new RobotLogTelemetry(
                                          "BLACK-ICE"));
        }
    }
    
    public void setTelemetry(Telemetry telemetry) {
        setTelemetry(telemetry, true);
    }

    public Vector getVelocity() {
        return localizer.getVelocity();
    }

    public double getAverageWheelVelocity(double theta) {
        return drivetrain.getAverageWheelVelocity(theta);
    }
    
    public Vector getPosition() {
        return localizer.getPose().getPosition();
    }
    
    public double getHeading() {
        return localizer.getPose().getHeading();
    }
    
    public double getVoltage() {
        return voltageSupplier.getAsDouble();
    }
    
    public Pose getCurrentPose() {
        return localizer.getPose();
    }
    
    public void setCurrentPose(Pose pose) {
        localizer.setCurrentPose(pose.getPosition().getX(), pose.getPosition().getY(),
                                 pose.getHeading());
        localizer.update(deltaTime);
    }

    public double computeHeadingCorrectionPower(double targetHeading) {
        double headingError =
            AngleUnit.RADIANS.normalize(targetHeading - localizer.getPose().getHeading());
        
        return headingController.computeOutput(headingError, deltaTime);
    }
    
    private double calculateDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;
        return deltaTime;
    }
    
    public void update() {
        deltaTime = calculateDeltaTime();
        localizer.update(deltaTime);
    }

    public void setCurrentHeading(double headingDegrees) {
        setCurrentPose(new Pose(localizer.getPose().getPosition().getX(),
                localizer.getPose().getPosition().getY(),
                headingDegrees));
    }
}
