# kdemo_record
GUI for robot kinesthetic demo recording

## =====  Installation  =====
Change directory to where install, src, kdemo_matlab etc are located and run:
```
./install.sh
```

## =====  Build  =====
Every time you want to rebuild the project run:
```
./build.sh
```

## =====  Execution  =====
```
source devel/setup.bash
roslaunch kdemo_record kdemo_record_lwr4p.launch
```


## ====  How to use the GUI  ====
There are 2 modes:

IDLE: the robot is still and stiff.

FREEDRIVE: the robot is free to be moved (gravity compensation mode)

### Register the data you want to record
You can do either of the following:
- press the "set recorded data" and check the data you want to be recorded. Then press ok.
- go to the menu bar and press "edit -> set recorded data"

### Record demo
Press the "FREEDRIVE" button to set the robot in gravity compensation mode.
You may have to wait a bit for the robot to change mode.
A popup will appear saying that the mode has changed.
Buttons "start", "stop" are now enabled.
Press button "clear" to clear all previously recorded data.
Press "start" to start recording.
Press "stop" to stop recording.

### Save the recorded data
"save data" button: saves the recorded data to "kdemo_matlab/data/recorded_data.bin"
"save data as" button: opens a file-save-dialog and saves the recorded data to the location specified by the user.
The above functionalities are also provided through the "file" menu bar.

### Register start pose and move to start pose (joint configuration)
In case you want to repeat a kinesthetic demo from the same initial joint configuration you can:
Set the mode to "FREEDRIVE" and move the robot to the desired initial configuration.
Press the button "set start pose" (registers the robot's current pose as starting pose)
After you have moved the robot and you want to move it back to its initial pose press "goto start pose".
The robot moves to the starting pose (using a 5th order poly and joint position control).
After the start pose is reached, a message is displayed and the mode is restored to the last one.

### Additional utilities
"view joint pose" button: opens a GUI that shows the robot's joints positions with sliders updated in real-time.
"view cart pose" button: opens a GUI that shows the robot's end effector pose updated in real-time.
"plot" button: plots the last recorded data. Data that were not registered for recording wont be plotted.
The above functionalities are also provided through the menu bar "view".

## ====  Plot/Export results in matlab  =====
Open matlab and browse to the directory "kdemo_matlab".
Change variable "filename" in the script "plotData.m" and run it to plot the recorded data.
Run the function "convertToMat.m" passing as argument the data filename to convert the data to *.mat format.
