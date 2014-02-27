define laser_fake ranger
(
	sensor( 			
  range [ 0.0  0.0 ]
  fov 10.0
  samples 3
  )
  color "blue"
  size [ 0.001 0.001 0.001 ] 
)



## DEPRECATED
#define laser_fake laser
#(
#  # laser-specific properties
#  range_max 0.0
#  fov 10.0
#  samples 3
#
#  # generic model properties
#  size [ 0.001 0.001 0.001 ]
#)
