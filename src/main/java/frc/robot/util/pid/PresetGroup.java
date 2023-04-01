package frc.robot.util.pid;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import static frc.robot.Constants.TEST_MODE;

public class PresetGroup extends HashMap<String, PresetList> {
    private int index = 0;

    public PresetGroup addPreset(String name, PresetList extensionPresets) {
        this.put(name, extensionPresets);
        return this;
    }

    public Command setPresetCommand(int index) {
        return Commands.runOnce(() -> setPreset(index));
    }

    public Double getCurrentPreset(String name) {
        return get(name).getCurrentPreset();
    }

    public PresetGroup setPreset(int index) {
        this.index = index;
        this.forEach((name, preset) -> preset.setPreset(index));
        new PrintCommand("SETTING PRESET TO " + index).schedule();
        return this;
    }

    public void updateDashboard() {
        if (TEST_MODE) {
            forEach((name, presetList) -> presetList.updateDashboard(name));
        }
    }

    public PresetGroup nextPreset() {
        if (index+1 <= size()) {
            setPreset(index+1);
        }
        return this;
    }

    public PresetGroup prevPreset() {
        if (index-1 >= 0) {
            setPreset(index-1);
        }
        return this;
    }
}
