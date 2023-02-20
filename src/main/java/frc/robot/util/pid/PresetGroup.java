package frc.robot.util.pid;

import java.util.HashMap;

public class PresetGroup extends HashMap<String, PresetList> {
    private int index = 0;

    public PresetGroup addPreset(String name, PresetList extensionPresets) {
        this.put(name, extensionPresets);
        return this;
    }

    public Double getCurrentPreset(String name) {
        return get(name).getCurrentPreset();
    }

    public PresetGroup setCurrentPreset(int index) {
        this.index = index;
        this.forEach((name, preset) -> preset.setCurrentPreset(index));
        return this;
    }

    public void updateDashboard() {
        forEach((name, presetList) -> presetList.updateDashboard(name));
    }

    public PresetGroup nextPreset() {
        if (index+1 <= size()-1) {
            setCurrentPreset(index+1);
        }
        return this;
    }

    public PresetGroup prevPreset() {
        if (index-1 >= 0) {
            setCurrentPreset(index-1);
        }
        return this;
    }
}
