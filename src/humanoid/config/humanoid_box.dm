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
		Mass				41.948234
		Inertia				7.405012	0.000000	0.000000
							0.000000	8.623801	0.000000
							0.000000	0.000000	1.421026
		Center_of_Gravity	0.000000	0.000000	0.405555
		Number_of_Contact_Points	0
		Position	2.000000	2.000000	1.200000
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
				Mass				7.257480
				Inertia				0.022857	0.000000	0.000000
									0.000000	0.468064	0.000000
									0.000000	0.000000	0.471099
				Center_of_Gravity	0.209054	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.482803	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	-0.127102	0.053035
				Orientation_Quat	-0.547419	0.447585	0.447585	0.547419
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "RightThigh"

			RevoluteLink {
				Name	"RightShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				3.374728
				Inertia				0.006328	0.000000	0.000000
									0.000000	0.177767	0.000000
									0.000000	0.000000	0.177767
				Center_of_Gravity	0.189257	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.437083	0.000000	0.000000
				MDH_Parameters	0.482803	0.000000	0.000000	0.500000
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
				MDH_Parameters	0.437083	0.000000	0.000000	-0.300000
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
				Mass				1.052335
				Inertia				0.014130	0.000000	-0.002006
									0.000000	0.012914	0.000000
									-0.002006	0.000000	0.003327
				Center_of_Gravity	0.027432	0.000000	0.069494
				Number_of_Contact_Points	4
				Contact_Locations	0.054864	0.080467	-0.069494
									0.054864	-0.080467	-0.069494
									0.054864	0.080467	0.208483
									0.054864	-0.080467	0.208483
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
				Mass				7.257480
				Inertia				0.022857	0.000000	0.000000
									0.000000	0.468064	0.000000
									0.000000	0.000000	0.471099
				Center_of_Gravity	0.209054	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.482803	0.000000	0.000000
				Position_From_Inboard_Link	0.000000	0.127102	0.053035
				Orientation_Quat	-0.547419	0.447585	0.447585	0.547419
				Initial_Angular_Velocity	0	0	0
				Joint_Friction	0
			}
			# End "LeftThigh"

			RevoluteLink {
				Name	"LeftShank"
				Graphics_Model	"./humanoid_box_model/shank.xan"
				Mass				3.374728
				Inertia				0.006328	0.000000	0.000000
									0.000000	0.177767	0.000000
									0.000000	0.000000	0.177767
				Center_of_Gravity	0.189257	0.000000	0.000000
				Number_of_Contact_Points	1
				Contact_Locations	0.437083	0.000000	0.000000
				MDH_Parameters	0.482803	0.000000	0.000000	0.500000
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
				MDH_Parameters	0.437083	0.000000	0.000000	-0.300000
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
				Mass				1.052335
				Inertia				0.014130	0.000000	-0.002006
									0.000000	0.012914	0.000000
									-0.002006	0.000000	0.003327
				Center_of_Gravity	0.027432	0.000000	0.069494
				Number_of_Contact_Points	4
				Contact_Locations	0.054864	0.080467	-0.069494
									0.054864	-0.080467	-0.069494
									0.054864	0.080467	0.208483
									0.054864	-0.080467	0.208483
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
			Mass				2.032094
			Inertia				0.003553	0.000000	0.000000
								0.000000	0.042953	0.000000
								0.000000	0.000000	0.042010
			Center_of_Gravity	0.118009	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.270662	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	-0.187452	0.587045
			Orientation_Quat	0.563990	0.287367	-0.351464	0.689788
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "RightArm"

		RevoluteLink {
			Name	"RightForeArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				1.596646
			Inertia				0.001810	0.000000	0.000000
								0.000000	0.169722	0.000000
								0.000000	0.000000	0.169482
			Center_of_Gravity	0.299338	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.438912	0.000000	0.000000
			MDH_Parameters	0.270662	0.000000	0.000000	0.700000
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
			Mass				2.032094
			Inertia				0.003553	0.000000	0.000000
								0.000000	0.042953	0.000000
								0.000000	0.000000	0.042010
			Center_of_Gravity	0.118009	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.270662	0.000000	0.000000
			Position_From_Inboard_Link	0.000000	0.187452	0.587045
			Orientation_Quat	0.351464	0.689788	-0.563990	0.287367
			Initial_Angular_Velocity	0	0	0
			Joint_Friction	0
		}
		# End "LeftArm"

		RevoluteLink {
			Name	"RightArm"
			Graphics_Model	"./humanoid_box_model/foreArm.xan"
			Mass				1.596646
			Inertia				0.001810	0.000000	0.000000
								0.000000	0.169722	0.000000
								0.000000	0.000000	0.169482
			Center_of_Gravity	0.299338	0.000000	0.000000
			Number_of_Contact_Points	1
			Contact_Locations	0.438912	0.000000	0.000000
			MDH_Parameters	0.270662	0.000000	0.000000	0.300000
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

