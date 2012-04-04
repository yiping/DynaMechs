# DynaMechs V 4.0 ascii
# kurmet parameter file

Articulation {
	Name		"Articulation"
	Graphics_Model	""

	Position		3 3 2
	Orientation_Quat	0	0	0	1

	

	MobileBaseLink {
		Name		"Torso"
		Graphics_Model	"./catbot_model/obj_cylinder_x.xan"

		Mass			1
		Inertia			.1	0 0
					    0  .55 0
					    0  0 .55
		Center_of_Gravity	.5 0 0	
		Number_of_Contact_Points	0

		Position 0 0 0
		Orientation_Quat  0 0 0 1
		Velocity 0 2 0 0 0 1
	}
	# End Mobile Base
	
	RevoluteLink {
		Name		"BackA"
		Graphics_Model	"./catbot_model/nullObj.xan"
	
		Mass			0
		Inertia			0 0 0
						0 0 0
						0 0 0
		Center_of_Gravity	0 	0	0	
		Number_of_Contact_Points	0
	
	
		MDH_Parameters		0	0	0	0	
		Initial_Joint_Velocity	0
		Joint_Limits			0	0
		Joint_Limit_Spring_Constant	0
		Joint_Limit_Damper_Constant	0
		Actuator_Type		0
		Joint_Friction		0
	}
	
	ZScrewTxLink {
			Name			"ZScrewTorso"
			ZScrew_Parameters	0	3.14159265
	}
	
	RevoluteLink {
		Name		"BackB"
		Graphics_Model	"./catbot_model/obj_cylinder_x_blue.xan"
	
				Mass			1
		Inertia			.1	0 0
					    0  .55 0
					    0  0 .55
		Center_of_Gravity	.5 0 0
		
		Number_of_Contact_Points	0	
	
		MDH_Parameters		0	-1.5708	0	0	
		Initial_Joint_Velocity	4
		Joint_Limits			0	0
		Joint_Limit_Spring_Constant	0
		Joint_Limit_Damper_Constant	0
		Actuator_Type		0
		Joint_Friction		0
	}
			
			
}
