define laser_urg04lx ranger
(
	sensor( 			
  # laser-specific properties
  # factory settings for URG04LX
  range [ 0.0  4.0 ]
  fov 240.0
  samples 683
  )
  # generic model properties
  color "blue"
  size [ 0.05 0.05 0.05 ] # dimensions from LMS200 data sheet	
)


## DEPRECATED (it has chanched for new versions --> Marco Becerra feb2014)
#define laser_urg04lx laser
#(
#  # laser-specific properties
#  range_max 4.0
#  fov 240.0
#  samples 683
#
#  # generic model properties
#  color "blue"
#  size [ 0.05 0.05 0.05 ]
#)
