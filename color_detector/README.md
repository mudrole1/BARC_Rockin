# Color Detection package

This program is to identify a person from specific ranges of color. Currently two identities are encoded. It subscribes to an `/camera/image_raw` topic comming from the `gscam` package and to a `/color_detector/start` topic to run the detection code until it correctly identifies the person or it reaches it's maximum number of samples.

To run this program, you do not need any additional arguments.

Finally, before running this program, you first need to configure & run the `gscam` node.

The complete configuration & running would be something like this:

      $ export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
  
      $ rosrun gscam gscam
  
      $ rosrun color_detector color_detector

The parameter `/dev/video0` must be adjusted to the apropiate camera.

To test it, you can publish the start message manually, like this:

      $ rostopic pub -1 /color_detection/start std_msgs::Empty

And do echo to the `/recognition/response`:

      $ rostopic echo /recognition/response

You should see a message with the class ID whenever a person is detected in front of the camera.

 
 
