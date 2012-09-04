# DynaMechs V 4.0 ascii
# humanoid parameter file

Articulation {
	Name				"Articulation"
	Graphics_Model		""
	Position			0	0	0
	Orientation_Quat	0	0	0	1

	MobileBaseLink {
	 	Name             		"Torso"
	 	Graphics_Model   		""
	 	Mass             		7.957477863
	 	Inertia          		0.102612972877134  -0.000090139110932   0.000354554573733
  								-0.000090139110932   0.071522231272454  -0.001404542150943
   								0.000354554573733  -0.001404542150943   0.075996371414740

	 	Center_of_Gravity   	-0.005494756 -0.000506882 -0.048924021
		Number_of_Contact_Points	0
		Position				2.0		2.0		1.5
		Orientation_Quat		0.0		0.0		0.0		1.0
		Velocity 				0 0	0 0	0 0
	}

	Branch {
		RevoluteLink {
		 	Name             		"Hip"
		 	Graphics_Model   		""
		 	Mass             		4.436339231
		 	Inertia          		0.031510422152986   0.000052645717638  -0.002489076633895
									0.000052645717638   0.018779965247395  -0.000013718231616
									-0.002489076633895  -0.000013718231616   0.027228685190794

		 	Center_of_Gravity   	-0.011804276 -0.000018681 -0.044394816
		 	Number_of_Contact_Points	0
		 	MDH_Parameters  		0	0	-0.1915	  0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	0 	0
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZScrewTorso"
			ZScrew_Parameters	0.0		1.5708
		}

		Branch {
			RevoluteLink {
				Name             		"leftHip"
			 	Graphics_Model   		""
			 	Mass             		0.483118742
			 	Inertia          	 	0.003449 	-0.000001 	0.000002
			 	                 	 	-0.000001 	0.002260 	-0.000219
			 	                 	 	0.000002 	-0.000219 	0.001495

			 	Center_of_Gravity    	 	0.000064 	-0.006790 	-0.053769

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0.0885	0	-0.086453	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZScrewLeftHip"
				ZScrew_Parameters	-0.0910		0
			}

			RevoluteLink {
				Name             		"leftHipPitchRoll"
			 	Graphics_Model   		""
			 	Mass             		2.644641101
			 	Inertia          	 	0.005520 	-0.000033 	-0.000175
			 	                 	 	-0.000033 	0.005220 	-0.000008
			 	                 	 	-0.000175 	-0.000008 	0.005079

			 	Center_of_Gravity    	 	0.010902 	-0.011973 	0.013849

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0	1.5708	0	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZScrewLeftHipPitchRoll"
				ZScrew_Parameters	0		-1.5708
			}

			RevoluteLink {
				Name             		"leftKneeUpper"
			 	Graphics_Model   		""
			 	Mass             		3.098799394
			 	Inertia          	 	0.021173 	0.010997 	-0.035346
			 	                 	 	0.010997 	0.157985 	0.003364
			 	                 	 	-0.035346 	0.003364 	0.146209

			 	Center_of_Gravity    	 	0.188775 	-0.019868 	0.058794

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0	-1.5708	 -0.054	 0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			RevoluteLink {
				Name             		"leftKneeLower"
			 	Graphics_Model   		""
			 	Mass             		1.559604102
			 	Inertia          	 	0.005860 	0.003439 	0.001270
			 	                 	 	0.003439 	0.058540 	-0.000511
			 	                 	 	0.001270 	-0.000511 	0.057037


			 	Center_of_Gravity    	 	0.152406 	-0.018120 	-0.013915

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0.300542	0	0.074999	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			RevoluteLink {
			 	Name             		"leftAnklePitch"
			 	Graphics_Model   		""
			 	Mass             		1.67491807
			 	Inertia          	 	0.006551 	0.000015 	-0.000782
			 	                 	 	0.000015 	0.005581 	-0.001492
			 	                 	 	-0.000782 	-0.001492 	0.003781


			 	Center_of_Gravity    	 	-0.009381 	-0.019150 	-0.042958

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0.299942	0	0.024755	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZScrewLeftAnklePitch"
				ZScrew_Parameters	-0.0465		0
			}

			RevoluteLink {
			 	Name             		"leftFoot"
			 	Graphics_Model   		""
			 	Mass             		0.525134235
			 	Inertia          	 	0.003741 	-0.000083 	0.001369
			 	                 	 	-0.000083 	0.005331 	0.000040
			 	                 	 	0.001369 	0.000040 	0.002945

			 	Center_of_Gravity    	 	0.055938 	0.001920 	-0.043322

			 	Number_of_Contact_Points	4
				Contact_Locations			0.05	-0.065	0.11
											0.05	0.065	0.11
											0.05	0.065	-0.11
											0.05	-0.065	-0.11
			 	MDH_Parameters  			0	1.5708	0.071244	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}


		}


		Branch {
			RevoluteLink {
			 	Name             		"rightHip"
			 	Graphics_Model   		""
			 	Mass             		0.483118742
			 	Inertia          	 	0.003449 	0.000001 	-0.000002
			 	                 	 	0.000001 	0.002260 	-0.000219
			 	                 	 	-0.000002 	-0.000219 	0.001495

			 	Center_of_Gravity    	 	-0.000064 	-0.006790 	-0.053769

			 	Number_of_Contact_Points	0  
			 	MDH_Parameters  			-0.0885	  0   -0.086453	  0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			ZScrewTxLink {
				Name	"ZScrewRightHip"
				ZScrew_Parameters	-0.0910		0
			}

			RevoluteLink {
			 	Name             		"rightHipPitchRoll"
			 	Graphics_Model   		""
			 	Mass             		2.644641101
			 	Inertia          	 	0.005520 	0.000033 	0.000175
			 	                 	 	0.000033 	0.005220 	-0.000008
			 	                 	 	0.000175 	-0.000008 	0.005079

			 	Center_of_Gravity    	 	-0.010902 	-0.011973 	0.013849

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0		1.5708	0	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			ZScrewTxLink {
				Name	"ZScrewRightHipPitchRoll"
				ZScrew_Parameters	0		-1.5708
			}

			RevoluteLink {
			 	Name             		"rightKneeUpper"
			 	Graphics_Model   		""
			 	Mass             		3.098799394
			 	Inertia          	 	0.021173 	0.010997 	0.035346
			 	                 	 	0.010997 	0.157985 	-0.003364
			 	                 	 	0.035346 	-0.003364 	0.146209

			 	Center_of_Gravity    	 	0.188775 	-0.019868 	-0.058794

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0		-1.5708		0.054	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			RevoluteLink {
			 	Name             		"rightKneeLower"
			 	Graphics_Model   		""
			 	Mass             		1.559604102
			 	Inertia          	 	0.005860 	0.003439 	-0.001270
			 	                 	 	0.003439 	0.058540 	0.000511
			 	                 	 	-0.001270 	0.000511 	0.057037

			 	Center_of_Gravity    	 	0.152406 	-0.018120 	0.013915

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0.300542	0	-0.074999	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			RevoluteLink {
			 	Name             		"rightAnklePitch"
			 	Graphics_Model   		""
			 	Mass             		1.67491807
			 	Inertia          	 	0.006551 	0.000015 	0.000782
			 	                 	 	0.000015 	0.005581 	0.001492
			 	                 	 	0.000782 	0.001492 	0.003781


			 	Center_of_Gravity    	 	-0.009381 	-0.019150 	0.042958

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0.299942	0	-0.024755	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			ZScrewTxLink {
				Name	"ZScrewRightAnklePitch"
				ZScrew_Parameters	0.0465		0
			}

			RevoluteLink {
			 	Name             		"rightFoot"
			 	Graphics_Model   		""
			 	Mass             		0.525134235
			 	Inertia          	 	0.003741 	0.000083 	0.001369
			 	                 	 	0.000083 	0.005331 	-0.000040
			 	                 	 	0.001369 	-0.000040 	0.002945

			 	Center_of_Gravity    	 	0.055938 	-0.001920 	-0.043322

			 	Number_of_Contact_Points	4
			 	Contact_Locations   		0.05	-0.065	0.11
											0.05	0.065	0.11
											0.05	0.065	-0.11
											0.05	-0.065	-0.11
			 	MDH_Parameters  			0	1.5708		0.071244	0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	0 	0
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}


		}

	}

}
