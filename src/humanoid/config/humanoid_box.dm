# DynaMechs V 4.0 ascii
# humanoid parameter file

Articulation {
	Name				"Articulation"
	Graphics_Model		""
	Position			0	0	0
	Orientation_Quat	0	0	0	1

	MobileBaseLink {
		Name	"Torso"
		Graphics_Model	"./humanoid_box_model/torso.xan"
		Mass				12.100000
		Inertia				0.315003	0.000000	0.000000
							0.000000	0.466253	0.000000
							0.000000	0.000000	0.171417
		Center_of_Gravity	0.000000	0.000000	0.150000
		Number_of_Contact_Points	0
		Position	2.000000	2.000000	0.620000
		Orientation_Quat	0.000000	0.000000	0.000000	1.000000
		Velocity 0 0 0 0 0 0
	}
	# End "Torso"

	Branch {

		ZScrewTxLink {
			Name	"ZScrewTorso"
			ZScrew_Parameters	0.000000	0.000000
		}
		# End "ZScrewTorso"

		Branch {

			QuaternionLink {
				Name	"RightThigh"
				Graphics_Model	"./humanoid_box_model/thigh.xan"
				Mass				0.810000
				Inertia				0.000486	0.000000	0.000000
									0.000000	0.009440	0.000000
									0.000000	0.000000	0.009440
				Center_of_Gravity	0.078400	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250000	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	-0.090000	0.000000
				Orientation_Quat	-0.514773	0.484777	0.484777	0.514773
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "RightThigh"

			RevoluteLink {
				Name	"RightShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				0.630000
				Inertia				0.000378	0.000000	0.000000
									0.000000	0.009325	0.000000
									0.000000	0.000000	0.009325
				Center_of_Gravity	0.096400	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250000	0.000000	0.000000
				MDH_Parameters	0.250000	0.000000	0.000000	0.150000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightShank"

			RevoluteLink {
				Name	"RightAnkle1"
				Graphics_Model	"./humanoid_box_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.250000	0.000000	0.000000	-0.090000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightAnkle1"

			RevoluteLink {
				Name	"RightAnkle2"
				Graphics_Model	"./humanoid_box_model/foot.xan"
				Mass				0.630000
				Inertia				0.002100	0.000000	-0.000079
									0.000000	0.001596	0.000000
									-0.000079	0.000000	0.000546
				Center_of_Gravity	0.005000	0.000000	0.025000
				Number_of_Contact_Points	4
				Contact_Locations	0.010000	0.050000	-0.050000
									0.010000	-0.050000	-0.050000
									0.010000	0.050000	0.100000
									0.010000	-0.050000	0.100000
				MDH_Parameters	0.000000	1.570796	0.000000	0.000000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "RightAnkle2"
		}
		# End Branch

		Branch {

			QuaternionLink {
				Name	"LeftThigh"
				Graphics_Model	"./humanoid_box_model/thigh.xan"
				Mass				0.810000
				Inertia				0.000486	0.000000	0.000000
									0.000000	0.009440	0.000000
									0.000000	0.000000	0.009440
				Center_of_Gravity	0.078400	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250000	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	0.090000	0.000000
				Orientation_Quat	-0.514773	0.484777	0.484777	0.514773
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "LeftThigh"

			RevoluteLink {
				Name	"LeftShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				0.630000
				Inertia				0.000378	0.000000	0.000000
									0.000000	0.009325	0.000000
									0.000000	0.000000	0.009325
				Center_of_Gravity	0.096400	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.250000	0.000000	0.000000
				MDH_Parameters	0.250000	0.000000	0.000000	0.150000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftShank"

			RevoluteLink {
				Name	"LeftAnkle1"
				Graphics_Model	"./humanoid_box_model/ankle.xan"
				Mass				0.000000
				Inertia				0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
									0.000000	0.000000	0.000000
				Center_of_Gravity	0.000000	0.000000	0.000000
				Number_of_Contact_Points	0
				MDH_Parameters	0.250000	0.000000	0.000000	-0.090000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftAnkle1"

			RevoluteLink {
				Name	"LeftAnkle2"
				Graphics_Model	"./humanoid_box_model/foot.xan"
				Mass				0.630000
				Inertia				0.002100	0.000000	-0.000079
									0.000000	0.001596	0.000000
									-0.000079	0.000000	0.000546
				Center_of_Gravity	0.005000	0.000000	0.025000
				Number_of_Contact_Points	4
				Contact_Locations	0.010000	0.050000	-0.050000
									0.010000	-0.050000	-0.050000
									0.010000	0.050000	0.100000
									0.010000	-0.050000	0.100000
				MDH_Parameters	0.000000	1.570796	0.000000	0.000000
				Initial_Joint_Velocity	0
				Joint_Limits	0	0
				Joint_Limit_Spring_Constant	0
				Joint_Limit_Damper_Constant	0
				Actuator_Type	0
				Joint_Friction	0
			}
			# End "LeftAnkle2"
		}
		# End Branch
	}
	# End Branch

	Branch {

		QuaternionLink {
			Name	"RightArm"
			Graphics_Model	"./humanoid_box_model/upperArm.xan"
			Mass				0.535360
			Inertia				0.000143	0.000000	0.000000
								0.000000	0.009220	0.000000
								0.000000	0.000000	0.009220
			Center_of_Gravity	0.109000	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.250000	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	-0.090000	0.400000
			Orientation_Quat	0.411088	0.481323	-0.588681	0.502781
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "RightArm"

		RevoluteLink {
			Name	"RightForeArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				0.420640
			Inertia				0.000112	0.000000	0.000000
								0.000000	0.014475	0.000000
								0.000000	0.000000	0.014475
			Center_of_Gravity	0.170500	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.250000	0.000000	0.000000
			MDH_Parameters	0.250000	0.000000	0.000000	0.500000
			Initial_Joint_Velocity	0
			Joint_Limits	0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type	0
			Joint_Friction	0
		}
		# End "RightForeArm"
	}
	# End Branch

	Branch {

		QuaternionLink {
			Name	"LeftArm"
			Graphics_Model	"./humanoid_box_model/upperArm.xan"
			Mass				0.535360
			Inertia				0.000143	0.000000	0.000000
								0.000000	0.009220	0.000000
								0.000000	0.000000	0.009220
			Center_of_Gravity	0.109000	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.250000	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	0.090000	0.400000
			Orientation_Quat	0.502781	0.588681	-0.481323	0.411088
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "LeftArm"

		RevoluteLink {
			Name	"RightArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				0.420640
			Inertia				0.000112	0.000000	0.000000
								0.000000	0.014475	0.000000
								0.000000	0.000000	0.014475
			Center_of_Gravity	0.170500	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.250000	0.000000	0.000000
			MDH_Parameters	0.250000	0.000000	0.000000	0.500000
			Initial_Joint_Velocity	0
			Joint_Limits	0	0
			Joint_Limit_Spring_Constant	0
			Joint_Limit_Damper_Constant	0
			Actuator_Type	0
			Joint_Friction	0
		}
		# End "RightArm"
	}
	# End Branch
}
# End Articulation

