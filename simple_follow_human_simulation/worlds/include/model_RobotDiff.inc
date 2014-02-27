define robot_diff position
(
  size [0.4 0.4 1]	
   block #BODY
   (
      points 12
      point[0]  [ 1 -4]
      point[1]  [ 3 -3]
      point[2]  [ 4 -1]
      point[3]  [ 4  1]
      point[4]  [ 3  3]
      point[5]  [ 1  4]
      point[6]  [-1  4]
      point[7]  [-3  3]
      point[8]  [-4  1]
      point[9]  [-4 -1]
      point[10] [-3 -3]
      point[11] [-1 -4]
      z [0 0.3]
   )
   block #NECK
   (
      points 4
      point[0]  [ -3 -0.5 ]
      point[1]  [ -3  0.5 ]
      point[2]  [ -4  0.5 ]
      point[3]  [ -4 -0.5 ]
      z [0.3 1]
   )
   
   

   color "cyan"	
   drive "diff"
)
