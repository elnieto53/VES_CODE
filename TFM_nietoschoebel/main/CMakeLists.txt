# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(	SRCS 	"main.c"
								"src/fsm.c"
								"src/bsp.c"
								"src/quatOps.c"
								"src/vector.c"
    							"drivers/src/esp32-i2cDriver.c"
    							"drivers/src/esp32-spiDriver.c"
    							"drivers/components/src/bhi360/bhi3.c"
    							"drivers/components/src/bhi360/bhy2.c"
    							"drivers/components/src/bhi360/common.c"
    							"drivers/components/src/bhi360/bhy2_hif.c"
    							"drivers/components/src/bhi360/bhy2_parse.c"
							    "drivers/components/src/bhi360/quaternion.c"		
    							"drivers/components/src/lsm6dso/lsm6dso.c" 	
    							"drivers/components/src/lsm6dso/lsm6dso_reg.c"
    							"drivers/components/src/drv2605l/drv2605L_Actuators.c" 			
    							"drivers/components/src/drv2605l/drv2605L.c" 
    							"drivers/components/src/w25n01gv/w25n01gv.c"				
    							"drivers/components/src/w25n01gv/lfs.c"			
    							"drivers/components/src/w25n01gv/lfs_util.c"	  				# list the source files of this component
 					
 						INCLUDE_DIRS    "."
 										"inc/."
 										"drivers/inc/."
 										"drivers/components/inc/."
 										"drivers/components/inc/bhi360/."
 										"drivers/components/inc/lsm6dso/."
 										"drivers/components/inc/drv2605l/."
 										"drivers/components/inc/w25n01gv/."					# optional, add here public include directories
										)
