# DynaMechs V 4.0 ascii
# kurmet parameter file

Articulation {
	Name		"Articulation"
	Graphics_Model	""

	Position		0	0	0
	# Orientation_Quat	0.5	0.5	0.5	0.5
    Orientation_Quat	0 0 0 1
	

	MobileBaseLink {
		Name		"Torso"
		Graphics_Model	"./humanoid_model/torso.xan"

		Mass			12.1
		Inertia			0.218	0	0
						0	0.257	0
						0	0	0.121
		#Center_of_Gravity	0.038	0	0.0849
		Center_of_Gravity	0	0	0.15	
		Number_of_Contact_Points	0

		Position 2 2 .62
		Orientation_Quat  0 0 0 1
		Velocity 0 0 0 0 0 0
	}
	# End Mobile Base

	
	# Lower Body
	Branch {
		ZScrewTxLink {
			Name			"ZScrewTorso"
			ZScrew_Parameters	0	0
		}
	
		# Right Leg
		Branch {
			
			QuaternionLink {
				Name		"RightThigh"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	0 0	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 -.09 0
				#Initial_Joint_Angles	0 0 -.2
				#Initial_Joint_Angles	0 1.57079633 1.57079633
				
				Orientation_Quat   -0.547418790962427   0.447585374315599   0.447585374315599 0.547418790962427
				#Orientation_Quat 0 0 0 1
				
				Initial_Angular_Velocity	0 0 0
				#Axes_Limits			0	0  0
				#Joint_Limit_Spring_Constant	0
				#Joint_Limit_Damper_Constant	0
				Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"RightShank"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0 0	
				Number_of_Contact_Points	1
					Contact_Locations	 0.2500         0         0
	
	
				MDH_Parameters		0.25	0	0	.5	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"RightAnkle1"
				Graphics_Model	"./humanoid_model/nullObj.xan"
	
				Mass			0
				Inertia			0 0 0
								0 0 0
								0 0 0
				Center_of_Gravity	0 	0	0	
				Number_of_Contact_Points	0	
	
				MDH_Parameters		0.25	0	0	-.3	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"RightAnkle2"
				Graphics_Model	"./humanoid_model/ankle.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0 0
				Number_of_Contact_Points	4
					Contact_Locations	 0.01         .05         .1
										 0.01         .05         -.05
										 0.01         -.05         .1
										 0.01         -.05         -.05
	
	
				MDH_Parameters		0	1.5708	0	0
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			# End Ankle
		}
		# End Right Leg
		
		# Left Leg
		Branch {
		
			QuaternionLink {
				Name		"LeftThigh"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	0 0
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 .09 0
				#Initial_Joint_Angles	0 0 -.2
				Orientation_Quat  -0.547418790962427   0.447585374315599   0.447585374315599 0.547418790962427
				#Orientation_Quat 0 0 0 1
				
				Initial_Angular_Velocity	0 0 0
				#Axes_Limits			0	0  0
				#Joint_Limit_Spring_Constant	0
				#Joint_Limit_Damper_Constant	0
				Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"LeftShank"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0 0	
				Number_of_Contact_Points	0
	
				MDH_Parameters		0.25	0	0	.5
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"LeftAnkle1"
				Graphics_Model	"./humanoid_model/nullObj.xan"
	
				Mass			0
				Inertia			0 0 0
								0 0 0
								0 0 0
				Center_of_Gravity	0 	0	0	
				Number_of_Contact_Points	0	
	
				MDH_Parameters		0.25	0	0	-.3
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			RevoluteLink {
				Name		"LeftAnkle2"
				Graphics_Model	"./humanoid_model/ankle.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0 0
				Number_of_Contact_Points	4
					Contact_Locations	 0.01         .05         .1
										 0.01         .05         -.05
										 0.01         -.05         .1
										 0.01         -.05         -.05
	
				MDH_Parameters		0	1.5708	0	0
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			
			# End Left Shank
		}
		# End Left leg
	}
	#End Lower Body
	
	# Upper Body
	Branch {
	
		# Right Arm
		Branch {
			
			QuaternionLink {
				Name		"RightArm"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	0 0	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 -.09 .4 
				
				#Orientation_Quat  0.7071         0   -0.1405    0.6930
				Orientation_Quat 0.411088474250840   0.481322761710462  -0.588681264845017 0.502781298198333
				#Orientation_Quat 0 0 0 1
				
				Initial_Angular_Velocity	0 0 0
				#Axes_Limits			0	0  0
				#Joint_Limit_Spring_Constant	0
				#Joint_Limit_Damper_Constant	0
				Joint_Friction		0
				
			}
	
			RevoluteLink {
				Name		"RightForearm"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0 0	
				Number_of_Contact_Points	1
					Contact_Locations	 0.2500         0         0
	
	
				MDH_Parameters		0.25	0	0	0.50000	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			# End Forearm
		}
		# End Right Arm
		
		# Left Arm
		Branch {
		
			QuaternionLink {
				Name		"LeftArm"
				Graphics_Model	"./humanoid_model/thigh.xan"
	
				Mass			0.81
				Inertia			0.00039	-1.6e-005	-0.00021
							-1.6e-005	0.0127	0
							-0.00021	0	0.0127
				Center_of_Gravity	0.0784	0 0	
				Number_of_Contact_Points	1
				Contact_Locations	0.25	0	0	
	
				Position_From_Inboard_Link 0 .09 .4
				#Initial_Joint_Angles	0 -2.4 -1.57
				#Initial_Joint_Angles	0 0 -1.57
				
				#Orientation_Quat  0.6930    0.1405         0    0.7071
				Orientation_Quat 0.502781298198333   0.588681264845017  -0.481322761710462 0.411088474250840
				#Orientation_Quat 0 0 0 1
				
				Initial_Angular_Velocity	0 0 0
				#Axes_Limits			0	0  0
				#Joint_Limit_Spring_Constant	0
				#Joint_Limit_Damper_Constant	0
				Joint_Friction		0
			}
	
			RevoluteLink {
				Name		"LeftForearm"
				Graphics_Model	"./humanoid_model/shank.xan"
	
				Mass			0.63
				Inertia			0.000238	-2e-006	-1e-006
							-2e-006	0.0116	0
							-1e-006	0	0.0116
				Center_of_Gravity	0.0964	0	0	
				Number_of_Contact_Points	1
				Contact_Locations	 0.2500         0         0
	
				MDH_Parameters		0.25	0	0	.5000	
				Initial_Joint_Velocity	0
				Joint_Limits			0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type		0
				Joint_Friction		0
			}
			# End Left Forearm
		}
		# End Left Arm
	}
	#End Upper Body
}
