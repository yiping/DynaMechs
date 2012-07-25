# DynaMechs V 4.0 ascii
# kurmet parameter file

Articulation {
	Name		"Articulation"
	Graphics_Model	""

	Position		0	0	0
	# Orientation_Quat	0.5	0.5	0.5	0.5
    Orientation_Quat	0 0 0 1
	

	MobileBaseLink {
		Name		"MobileBox"
		Graphics_Model	"./model/box.xan"

		Mass			10.0
		Inertia			0.1667	0.0  	0.0
						0.0 	0.1667	0.0
						0.0 	0.0 	0.2667
		Center_of_Gravity	0.0	0.0	0.0	
		Number_of_Contact_Points	0 
		#Contact_Locations
					
		Number_of_Dynamic_Contact_Points 1
		Dynamic_Contact_Locations
					   0.00   0.00    0.00

		Position 2.2 2.2 0.9
		Orientation_Quat  0 0 0 1
		Velocity 0 0 0 0 0 0
	}
	# End Mobile Base

	
}
