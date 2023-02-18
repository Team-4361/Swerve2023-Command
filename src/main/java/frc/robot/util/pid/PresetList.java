package frc.robot.util.pid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

public class PresetList<T> extends ArrayList<T> {
    private double index = 0;
    private String name;

    @SafeVarargs
    public PresetList(T... elements) {
        this.addAll(Arrays.asList(elements));
    }

    public T getCurrentPreset() {
        return get((int)MathUtil.clamp(index, 0, size()-1));
    }

    public PresetList<T> setCurrentPreset(int index) {
        this.index = index;
        return this;
    }

    public PresetList<T> nextPreset() {
        if (index+1 <= size()-1) {
            index++;
        }
        return this;
    }

    public PresetList<T> prevPreset() {
        if (index-1 >= 0) {
            index--;
        }
        return this;
    }
}
