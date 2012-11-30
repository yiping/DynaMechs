# DynaMechs V 4.0 ascii
# huboplus parameter file

Articulation {
	Name				"Articulation"
	Graphics_Model		"  "
	Position			0	0	0
	Orientation_Quat	0	0	0	1

	MobileBaseLink {
	 	Name             		"BodyTorso"
	 	Graphics_Model   		"./huboplus_model/Body_Torso_1.xan|./huboplus_model/Body_Torso_2.xan"
	 	Mass             		7.957477863
	 	Inertia          		0.119813675847886  -0.000202926238281   0.002656875041078
		                        -0.000202926238281   0.102148420018588  -0.002219850704579
		                        0.002656875041078  -0.002219850704579   0.087666371065534

	 	Center_of_Gravity   	-0.012258116293  -0.002473149928  -0.048635606065
		Number_of_Contact_Points	0
		Position				2.0		2.0		1.1
		Orientation_Quat		0.0		0.0784591		0.0		0.996917
		Velocity 				0 0	0 0	0 0
	}

	# # Lower body

	Branch {
		RevoluteLink {
		 	Name             		"Body_Hip"
		 	Graphics_Model   		"./huboplus_model/Body_Hip_1.xan"
		 	Mass             		3.417193968804
		 	Inertia          	 	0.022332 	0.000032 	-0.002192
		 	                 	 	0.000032 	0.014076 	-0.000068
		 	                 	 	-0.002192 	-0.000068 	0.022707


		 	Center_of_Gravity    	 	-0.013850 	0.000008 	-0.035461

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	0	-0.201256	0	
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_Torso"
			ZScrew_Parameters	0.0		1.570796
		}


		# # # Right leg

		Branch {
			RevoluteLink {
			 	Name             		"Body_RHY"
			 	Graphics_Model   		"./huboplus_model/Body_RHY_1.xan|./huboplus_model/Body_RHY_2.xan"
			 	Mass             		0.826125012988
			 	Inertia          	 	0.008215 	0.000001 	-0.000002
			 	                 	 	0.000001 	0.005874 	-0.002516
			 	                 	 	-0.000002 	-0.002516 	0.003109

			 	Center_of_Gravity    	 	-0.000037 	-0.034707 	-0.072615

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			-0.0885		0	-0.0765		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-90 	90
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZS_RHY"
				ZScrew_Parameters	-0.0910		0.0
			}

			RevoluteLink {
			 	Name             		"Body_RHR"
			 	Graphics_Model   		"./huboplus_model/Body_RHR_1.xan"
			 	Mass             		1.93265668478
			 	Inertia          	 	0.008583 	0.000173 	-0.001284
			 	                 	 	0.000173 	0.008258 	-0.001629
			 	                 	 	-0.001284 	-0.001629 	0.004811


			 	Center_of_Gravity    	 	-0.012531 	-0.015644 	-0.049748

			 	Number_of_Contact_Points	0	
			 	MDH_Parameters  			0	1.570796	0.0520		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-28 	28
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZS_RHR"
				ZScrew_Parameters	-0.0529		1.570796
			}

			RevoluteLink {
			 	Name             		"Body_RHP"
			 	Graphics_Model   		"./huboplus_model/Body_RHP_1.xan|./huboplus_model/Body_RHP_2.xan"
			 	Mass             		2.820095294731
			 	Inertia          	 	0.019463 	0.009267 	0.028780
			 	                 	 	0.009267 	0.126085 	-0.003461
			 	                 	 	0.028780 	-0.003461 	0.115015


			 	Center_of_Gravity    	 	-0.175202 	0.019505 	0.059577

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0	1.570796	-0.0656		-0.557079
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-85 	92
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}


			RevoluteLink {
			 	Name             		"Body_RKP"
			 	Graphics_Model   		"./huboplus_model/Body_RKP_1.xan|./huboplus_model/Body_RKP_2.xan"
			 	Mass             		1.809116607048
			 	Inertia          	 	0.006314 	0.002684 	-0.000524
			 	                 	 	0.002684 	0.076479 	-0.000420
			 	                 	 	-0.000524 	-0.000420 	0.074299

			 	Center_of_Gravity    	 	-0.171431 	0.012825 	0.007276

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			-0.2800		0	0.0445		0.8	
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-4 	149
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			RevoluteLink {
			 	Name             		"Body_RAP"
			 	Graphics_Model   		"./huboplus_model/Body_RAP_1.xan"
			 	Mass             		1.635010626057
			 	Inertia          	 	0.006896 	-0.000004 	-0.000929
			 	                 	 	-0.000004 	0.005988 	-0.001512
			 	                 	 	-0.000929 	-0.001512 	0.004166


			 	Center_of_Gravity    	 	0.011507 	0.019870 	0.045969

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			-0.2799		0	-0.02476	-0.4
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-74 	97
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZS_RAP"
				ZScrew_Parameters	0.0466		0.0
			}


			RevoluteLink {
			 	Name             		"Body_RAR"
			 	Graphics_Model   		"./huboplus_model/Body_RAR_1.xan|./huboplus_model/Body_RAR_2.xan"
			 	Mass             		1.203176884538
			 	Inertia          	 	0.008366 	0.000240 	-0.004159
			 	                 	 	0.000240 	0.014233 	0.000166
			 	                 	 	-0.004159 	0.000166 	0.008750


			 	Center_of_Gravity    	 	-0.069388 	0.002164 	-0.051509

			 	Number_of_Contact_Points	4
			 	Contact_Locations   		-0.1064  -0.071	-0.1515	
											-0.1064  -0.071	0.0705
											-0.1064  0.081	0.0705
											-0.1064  0.081	-0.1515	
			 	MDH_Parameters  			0	-1.570796	0.07118		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-11 	11
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

		}

		# # # Left leg

		Branch {
			RevoluteLink {
			 	Name             		"Body_LHY"
			 	Graphics_Model   		"./huboplus_model/Body_LHY_1.xan|./huboplus_model/Body_LHY_2.xan"
			 	Mass             		0.826125012988
			 	Inertia          	 	0.008215 	-0.000001 	0.000002
			 	                 	 	-0.000001 	0.005874 	-0.002516
			 	                 	 	0.000002 	-0.002516 	0.003109

			 	Center_of_Gravity    	 	0.000037 	-0.034707 	-0.072615

			 	Number_of_Contact_Points	0 
			 	MDH_Parameters  			0.0885		0	-0.0765		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-90 	90
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0

			}

			ZScrewTxLink {
				Name	"ZS_LHY"
				ZScrew_Parameters	-0.0910		0.0
			}

			RevoluteLink {
			 	Name             		"Body_LHR"
			 	Graphics_Model   		"./huboplus_model/Body_LHR_1.xan"
			 	Mass             		1.93265668478
			 	Inertia          	 	0.008583 	-0.000173 	0.001284
			 	                 	 	-0.000173 	0.008258 	-0.001629
			 	                 	 	0.001284 	-0.001629 	0.004811


			 	Center_of_Gravity    	 	0.012531 	-0.015644 	-0.049748

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0	1.570796	0.0520		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-28 	28
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZS_LHR"
				ZScrew_Parameters	-0.0529		1.570796
			}

			RevoluteLink {
			 	Name             		"Body_LHP"
			 	Graphics_Model   		"./huboplus_model/Body_LHP_1.xan|./huboplus_model/Body_LHP_2.xan"
			 	Mass             		2.820095294731
			 	Inertia          	 	0.019463 	0.009267 	-0.028780
			 	                 	 	0.009267 	0.126085 	0.003461
			 	                 	 	-0.028780 	0.003461 	0.115015

			 	Center_of_Gravity    	 	-0.175202 	0.019505 	-0.059577

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			0	1.570796	0.0656		-0.557079
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-85 	92
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}


			RevoluteLink {
			 	Name             		"Body_LKP"
			 	Graphics_Model   		"./huboplus_model/Body_LKP_1.xan|./huboplus_model/Body_LKP_2.xan"
			 	Mass             		1.809116607048
			 	Inertia          	 	0.006314 	0.002684 	0.000524
			 	                 	 	0.002684 	0.076479 	0.000420
			 	                 	 	0.000524 	0.000420 	0.074299


			 	Center_of_Gravity    	 	-0.171431 	0.012825 	-0.007276

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			-0.2800		0	-0.0445		0.8
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-4 	149
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			RevoluteLink {
			 	Name             		"Body_LAP"
			 	Graphics_Model   		"./huboplus_model/Body_LAP_1.xan"
			 	Mass             		1.635010626057
			 	Inertia          	 	0.006896 	-0.000004 	0.000929
			 	                 	 	-0.000004 	0.005988 	0.001512
			 	                 	 	0.000929 	0.001512 	0.004166


			 	Center_of_Gravity    	 	0.011507 	0.019870 	-0.045969

			 	Number_of_Contact_Points	0
			 	MDH_Parameters  			-0.2799		0	0.02476		-0.4
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-74 	97
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}

			ZScrewTxLink {
				Name	"ZS_LAP"
				ZScrew_Parameters	-0.0466		0.0
			}


			RevoluteLink {
			 	Name             		"Body_LAR"
			 	Graphics_Model   		"./huboplus_model/Body_LAR_1.xan|./huboplus_model/Body_LAR_2.xan"
			 	Mass             		1.203176884538
			 	Inertia          	 	0.008366 	-0.000240 	-0.004159
			 	                 	 	-0.000240 	0.014233 	-0.000166
			 	                 	 	-0.004159 	-0.000166 	0.008750

			 	Center_of_Gravity    	 	-0.069388 	-0.002164 	-0.051509

			 	Number_of_Contact_Points	4
			 	Contact_Locations   		-0.1064  -0.081	-0.1515	
											-0.1064  -0.081	0.0705
											-0.1064  0.071	0.0705
											-0.1064  0.071	-0.1515	  
			 	MDH_Parameters 				0	-1.570796	0.07118		0
			 	Initial_Joint_Velocity   	0
			 	Joint_Limits     	-11 	11
			 	Joint_Limit_Spring_Constant  	0
			 	Joint_Limit_Damper_Constant  	0
			 	Actuator_Type    	0
			 	Joint_Friction   	0
			}


		}

	}

	# # Upper Body

	# # Right arm

	Branch {
		RevoluteLink {
		 	Name             		"Body_RSP"
		 	Graphics_Model   		"./huboplus_model/Body_RSP_1.xan"
		 	Mass             		0.646654803406
		 	Inertia          	 	0.002998 	-0.000000 	0.000773
		 	                 	 	-0.000000 	0.003528 	-0.000000
		 	                 	 	0.000773 	-0.000000 	0.001111

		 	Center_of_Gravity    	 	0.016721 	-0.000010 	-0.057041

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	-1.570796	-0.1415		0.2
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_RSP"
			ZScrew_Parameters	-0.0720		1.570796
		}

		RevoluteLink {
		 	Name             		"Body_RSR"
		 	Graphics_Model   		"./huboplus_model/Body_RSR_1.xan"
		 	Mass             		0.413367772877
		 	Inertia          	 	0.000902 	0.000001 	0.000016
		 	                 	 	0.000001 	0.000919 	0.000001
		 	                 	 	0.000016 	0.000001 	0.000270

		 	Center_of_Gravity    	 	0.001267 	0.000083 	-0.036208

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	1.570796	0.0269	-0.3
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-180 	4
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_RSR"
			ZScrew_Parameters	-0.0269		1.570796
		}

		RevoluteLink {
		 	Name             		"Body_RSY"
		 	Graphics_Model   		"./huboplus_model/Body_RSY_1.xan|./huboplus_model/Body_RSY_2.xan|./huboplus_model/Body_RSY_3.xan"
		 	Mass             		1.154608561577
		 	Inertia          	 	0.013187 	0.000002 	0.000028
		 	                 	 	0.000002 	0.013170 	-0.000897
		 	                 	 	0.000028 	-0.000897 	0.000630


		 	Center_of_Gravity    	 	-0.000031 	-0.004561 	-0.090782

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	-1.570796	-0.0245		0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_RSY"
			ZScrew_Parameters	-0.1575		-1.570796
		}

		RevoluteLink {
		 	Name             		"Body_REP"
		 	Graphics_Model   		"./huboplus_model/Body_REP_1.xan"
		 	Mass             		0.440940029986
		 	Inertia          	 	0.002288 	0.000600 	-0.000175
		 	                 	 	0.000600 	0.000480 	0.000488
		 	                 	 	-0.000175 	0.000488 	0.002348

		 	Center_of_Gravity    	 	-0.021735 	0.061087 	-0.017831

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0.0220	-1.570796	0.0180	-0.6
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-143 	4
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_REP"
			ZScrew_Parameters	-0.0180		0
		}

		RevoluteLink {
		 	Name             		"Body_RWY"
		 	Graphics_Model   		"./huboplus_model/Body_RWY_1.xan|./huboplus_model/Body_RWY_2.xan"
		 	Mass             		0.53773814955
		 	Inertia          	 	0.000939 	0.000000 	-0.000009
		 	                 	 	0.000000 	0.000869 	-0.000044
		 	                 	 	-0.000009 	-0.000044 	0.000306


		 	Center_of_Gravity    	 	-0.001220 	-0.000635 	-0.023804

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			-0.0220		1.570796	-0.1114	0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_RWY"
			ZScrew_Parameters	-0.0525		0
		}

		RevoluteLink {
		 	Name             		"Body_RWP"
		 	Graphics_Model   		"./huboplus_model/Body_RWP_1.xan"
		 	Mass             		0.164392998329
		 	Inertia          	 	0.000469 	0.000003 	0.000000
		 	                 	 	0.000003 	0.000144 	0.000096
		 	                 	 	0.000000 	0.000096 	0.000495

		 	Center_of_Gravity    	 	-0.000258 	0.046147 	-0.011442

		 	Number_of_Contact_Points	1
		 	Contact_Locations   		0	0.1		0
		 	MDH_Parameters				0	 -1.570796	0.0100	0 
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-90 	90
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}
	}

	# # Left arm

	Branch {
		RevoluteLink {
		 	Name             		"Body_LSP"
		 	Graphics_Model   		"./huboplus_model/Body_LSP_1.xan"
		 	Mass             		0.646654803406
		 	Inertia          	 	0.002998 	-0.000000 	-0.000773
		 	                 	 	-0.000000 	0.003528 	0.000000
		 	                 	 	-0.000773 	0.000000 	0.001111


		 	Center_of_Gravity    	 	0.016721 	-0.000010 	0.057041

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	-1.570796	0.1415		0.20
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_LSP"
			ZScrew_Parameters	0.0720		1.570796
		}

		RevoluteLink {
		 	Name             		"Body_LSR"
		 	Graphics_Model   		"./huboplus_model/Body_LSR_1.xan"
		 	Mass             		0.413367772877
		 	Inertia          	 	0.000902 	-0.000001 	0.000016
		 	                 	 	-0.000001 	0.000919 	-0.000001
		 	                 	 	0.000016 	-0.000001 	0.000270


		 	Center_of_Gravity    	 	0.001267 	-0.000083 	-0.036208

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	1.570796	0.0269	0.3
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-4 	180
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_LSR"
			ZScrew_Parameters	-0.0269		1.570796
		}

		RevoluteLink {
		 	Name             		"Body_LSY"
		 	Graphics_Model   		"./huboplus_model/Body_LSY_1.xan|./huboplus_model/Body_LSY_2.xan"
		 	Mass             		1.154608561577
		 	Inertia          	 	0.013187 	-0.000002 	-0.000028
		 	                 	 	-0.000002 	0.013170 	-0.000897
		 	                 	 	-0.000028 	-0.000897 	0.000630


		 	Center_of_Gravity    	 	0.000031 	-0.004561 	-0.090782

		 	Number_of_Contact_Points	0

		 	MDH_Parameters  			0	-1.570796	-0.0245		0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_LSY"
			ZScrew_Parameters	-0.1575		-1.570796
		}

		RevoluteLink {
		 	Name             		"Body_LEP"
		 	Graphics_Model   		"./huboplus_model/Body_LEP_1.xan"
		 	Mass             		0.440940029986
		 	Inertia          	 	0.002288 	0.000600 	0.000175
		 	                 	 	0.000600 	0.000480 	-0.000488
		 	                 	 	0.000175 	-0.000488 	0.002348

		 	Center_of_Gravity    	 	-0.021735 	0.061087 	0.017831

		 	Number_of_Contact_Points	0

		 	MDH_Parameters  			0.0220	-1.570796	-0.0180		-0.6
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-143 	4
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_LEP"
			ZScrew_Parameters	0.0180		0
		}

		RevoluteLink {
		 	Name             		"Body_LWY"
		 	Graphics_Model   		"./huboplus_model/Body_LWY_1.xan|./huboplus_model/Body_LWY_2.xan"
		 	Mass             		0.53773814955
		 	Inertia          	 	0.000939 	-0.000000 	-0.000009
		 	                 	 	-0.000000 	0.000869 	0.000044
		 	                 	 	-0.000009 	0.000044 	0.000306


		 	Center_of_Gravity    	 	-0.001220 	0.000635 	-0.023804

		 	Number_of_Contact_Points	0

		 	MDH_Parameters  			-0.0220		1.570796	-0.1114	0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-179.999 	179.999
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}

		ZScrewTxLink {
			Name	"ZS_LWY"
			ZScrew_Parameters	-0.0525		0
		}

		RevoluteLink {
		 	Name             		"Body_LWP"
		 	Graphics_Model   		"./huboplus_model/Body_LWP_1.xan"
		 	Mass             		0.164392998329
		 	Inertia          	 	0.000469 	0.000003 	-0.000000
		 	                 	 	0.000003 	0.000144 	-0.000096
		 	                 	 	-0.000000 	-0.000096 	0.000495

		 	Center_of_Gravity    	 	-0.000258 	0.046147 	0.011442

		 	Number_of_Contact_Points	1
		 	Contact_Locations   		0	0.1		0
		 	MDH_Parameters  			0	 -1.570796	-0.0100	0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	-90 	90
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0
		}
	}

	# # head
	Branch {
		RevoluteLink {
		 	Name             		"Body_Head"
		 	Graphics_Model   		"./huboplus_model/Body_HNP_1.xan"
		 	Mass             		0.374250901076
		 	Inertia          	 	0.002026514230393   0.000000012181153  -0.000223896398772
									0.000000012181153   0.001968368032387   0.000001615458624
									-0.000223896398772   0.000001615458624   0.001946682068534

		 	Center_of_Gravity    	 	-0.010688581590 0.000030086449 -0.032122953618

		 	Number_of_Contact_Points	0
		 	MDH_Parameters  			0	 0	0.1483	0
		 	Initial_Joint_Velocity   	0
		 	Joint_Limits     	0 	0
		 	Joint_Limit_Spring_Constant  	0
		 	Joint_Limit_Damper_Constant  	0
		 	Actuator_Type    	0
		 	Joint_Friction   	0			
		}
	}

}
