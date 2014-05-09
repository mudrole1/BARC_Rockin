# Face dataset builder package

This program is build a dataset from scratch or to add new members to it. It subscribes to an `/camera/image_raw` topic comming from the `gscam` package. To run this program, you have to pass five arguments. 

1. Absolute path to the filter file
2. Absolute path to the nested filter file
3. Absolute path to the dataset folder
4. Absolute path to the CSV file
5. ID number for the new person [1-99]

Finally, before running this program, you first need to configure & run the `gscam` node.

The complete configuration & running would be something like this:

      $ export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
      $ rosrun gscam gscam
      $ rosrun face_add face_add_node <PATH_FILTER_FILE> <PATH_NESTEDF__FILE> <PATH_DATASET_FOLDER><PATH_CSV_FILE> <ID_NUMBER>

The parameter `/dev/video0` must be adjusted to the apropiate camera.

At the end of the program, a number (configured inside the code) of images wid faces samples will be stored inside the dataset folder, and the new CVS entries would be updated.
 
 
