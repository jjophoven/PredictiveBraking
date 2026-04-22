package org.firstinspires.ftc.teamcode.blackice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;

@Autonomous
public class LocalizationTest extends OpMode {
    Follower follower;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        
        follower.setTelemetry(telemetry);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.telemetry.addData("Pose", follower.localizer.getPose());
        follower.telemetry.update();
    }
}
