package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.camera.PhotonCameraModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

public class CameraSubsystem extends SubsystemBase {

    private final ArrayList<PhotonCameraModule> cameras;

    public CameraSubsystem addCamera(PhotonCameraModule camera) {
        cameras.add(camera);
        return this;
    }

    public PhotonCameraModule getCamera(String name) {
        AtomicReference<PhotonCameraModule> module = new AtomicReference<>(null);
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

    public CameraSubsystem(PhotonCameraModule... cameraList) {
        this.cameras = new ArrayList<>();
        cameras.addAll(Arrays.asList(cameraList));
    }

    @Override
    public void periodic() {
        cameras.forEach((PhotonCameraModule::update));
    }
}
