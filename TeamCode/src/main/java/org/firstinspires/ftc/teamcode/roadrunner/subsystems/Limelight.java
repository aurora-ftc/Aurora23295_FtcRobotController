package org.firstinspires.ftc.teamcode.roadrunner.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.LimelightControl;
import org.firstinspires.ftc.teamcode.teleOp.util.Mosaic;

public class Limelight {
    private LimelightControl limelightControl;

    private Mosaic mosaic = Mosaic.UNKNOWN;

    public Limelight(HardwareMap hwMap, Telemetry tele) {
        limelightControl = new LimelightControl(hwMap, llPipelines.APRIL_TAG_DETECTION);
    }

    public Mosaic scanObelisk() {
        mosaic = limelightControl.scanObelisk();
        if (mosaic != null)
            return mosaic;
        else
            return Mosaic.UNKNOWN;
    }

    public Mosaic getLastDetectedMosaic() {
        if (mosaic != null)
            return mosaic;
        else
            return Mosaic.UNKNOWN;
    }

    // ---------- Actions ----------
    public Action scanForTime(double duration, Telemetry tele) {
        return new Action() {
            private boolean init = false;
            private final ElapsedTime timer = new ElapsedTime();
            private ElapsedTime photoTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    timer.reset();
                    photoTimer.reset();
                    init = true;
                    packet.put("Timer", "Reset");
                }

                if (timer.seconds() < duration) {
                    if (photoTimer.milliseconds() >= 50) {
                        Mosaic mosaic = limelightControl.scanObelisk();
                        if (mosaic != Mosaic.UNKNOWN && mosaic != null)
                            Limelight.this.mosaic = mosaic;
                        packet.put("Mosaic", mosaic.name());
                        photoTimer.reset();
                    }
                    return true;
                } else {
                    return false;
                }
            }
        };
    }

}
