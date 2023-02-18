package frc.robot.util.pid;

import java.util.HashMap;

public class PresetGroup extends HashMap<String, PresetList<Double>> {
    private int index = 0;

    public PresetGroup addPreset(String name, PresetList<Double> extensionPresets) {
        this.put(name, extensionPresets);
        return this;
    }

    public double getCurrentPreset(String name) {
        PresetList<Double> preset = get(name);
        return preset.get(index);
    }

    public PresetGroup setCurrentPreset(int index) {
        this.index = index;
        return this;
    }

    public PresetGroup nextPreset() {
        if (index+1 <= size()-1) {
            index++;
        }
        return this;
    }

    public PresetGroup prevPreset() {
        if (index-1 >= 0) {
            index--;
        }
        return this;
    }
}
