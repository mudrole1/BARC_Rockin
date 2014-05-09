# Face recognition package

This program is to identify a person from a dataset whenever is possible (when a threshold is reached). It subscribes to an `/camera/image_raw` topic comming from the `gscam` package and to a `/face_rec_wakeup` topic to perform the face recognition after receiving the string  "DoFaceRecognition".

To run this program, you have to pass two arguments. The first one is the path to the Haar Cascade Filter file selected for face detection; and the second argument is the path to the CSV file (dataset). It is important that the CSV file contains absolute paths to prevent initializing this node from an invalid source folder.

Finally, before running this program, you first need to configure & run the `gscam` node.

The complete configuration & running would be something like this:

      $ export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
  
      $ rosrun gscam gscam
  
      $ rosrun face_recognizer <ABSOLUTE_PATH_TO_FILTER_FILE> <ABSOLUTE_PATH_TO_CSV_FILE>

The parameter `/dev/video0` must be adjusted to the apropiate camera.

To test it, you can publish the wake up messages manually, like this:

      $ rostopic pub -1 /face_rec_wakeup std_msgs::String -- 'DoFaceRecognition'

And do echo to the `/face_recognizer_ID`:

      $ rostopic echo /face_recognizer_ID

You should see a message with the class ID whenever a person is detected in front of the camera.

 
 
