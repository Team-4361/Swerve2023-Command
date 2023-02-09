package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.camera.CameraModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

public class CameraSubsystem extends SubsystemBase {

    private final ArrayList<CameraModule> cameras;

    public CameraSubsystem addCamera(CameraModule camera) {
        cameras.add(camera);
        return this;
    }

    public CameraModule getCamera(String name) {
        AtomicReference<CameraModule> module = new AtomicReference<>(null);
        cameras.forEach(c -> {
            if (c.getName().equals(name)) {
                module.set(c);
            }
        });
        return module.get();
    }

    public CameraSubsystem() {
        this.cameras = new ArrayList<>();
    }

    public CameraSubsystem(CameraModule... cameraList) {
        this.cameras = new ArrayList<>();
        cameras.addAll(Arrays.asList(cameraList));
    }

    @Override
    public void periodic() {
        cameras.forEach((CameraModule::update));
    }
}
