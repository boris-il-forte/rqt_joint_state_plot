## rqt_joint_state_plot

### To start up

Run rqt as:
```
 $ rqt
```
Choose the 'Plugins'->'Visualization'->'JointTrajectoryPlot'

### To select topic

Choose the topic name from the pulldown menu. The menu has the topic
of sensor_msgs/JointState type. If you don't see target
topic, push the reload button to refresh the topic list.

When you choose the topic, the plugin subscribe the target topic and ready to draw the curve in the plot area.

### To select curve to plot

Click the checkbox of the target joint name. You can choose the curve
of poistion, velocity, and effort.

Beware the plugin redraws the graph each time when the new message arrived. If you want to stop the data update, push pause button. 
