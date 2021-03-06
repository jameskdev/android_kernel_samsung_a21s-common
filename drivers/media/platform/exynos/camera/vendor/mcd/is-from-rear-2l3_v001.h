#ifndef IS_FROM_REAR_2L3_V001_H
#define IS_FROM_REAR_2L3_V001_H

/* Header referenced section */
#define ROM_HEADER_CAL_DATA_START_ADDR           	     0x00
#define ROM_HEADER_CAL_DATA_END_ADDR 	          	     0x020F
#define ROM_HEADER_VERSION_START_ADDR           0x40
#define ROM_HEADER2_VERSION_START_ADDR          0x30 /* REAR SECOND SENSOR (TELE) */
#define ROM_HEADER_MODULE_ID_START_ADDR         0x0
#define ROM_HEADER_MODULE_ID_SIZE               0x2
#define ROM_HEADER_CAL_MAP_VER_START_ADDR       0x60
#define ROM_HEADER_ISP_SETFILE_VER_START_ADDR   0x64
#define ROM_HEADER_PROJECT_NAME_START_ADDR      0x6E
#define ROM_HEADER_CONCORD_HEADER_VER_START_ADDR    0x50

#define ROM_HEADER_CONCORD_BINARY_LY_START_ADDR    	0xE0  	/* rear : SONY(imx333) front : S.LSI(3H1) */
#define ROM_HEADER_CONCORD_BINARY_LY_END_ADDR   		0xE4    /* rear : SONY(imx333) front : LSI(3H1) */
#define ROM_HEADER_CONCORD_BINARY_LS_START_ADDR    	0xE8	/* rear : SONY(imx333) front : Sony(IMX320) */
#define ROM_HEADER_CONCORD_BINARY_LS_END_ADDR      	0xEC    /* rear : SONY(imx333) front : Sony(IMX320) */

#define ROM_HEADER_ISP_BINARY_START_ADDR        0xF0	/*DDK*/
#define ROM_HEADER_ISP_BINARY_END_ADDR          0xF4	/*DDK*/
#define ROM_HEADER_RTA_BINARY_LY_START_ADDR        0xF8	/*RTA*/	/* rear : SONY(imx333) front : S.LSI(3H1) */
#define ROM_HEADER_RTA_BINARY_LY_END_ADDR          0xFC	/*RTA*/	/* rear : SONY(imx333) front : LSI(3H1) */
#define ROM_HEADER_RTA_BINARY_LS_START_ADDR        0x120	/*RTA*/	/* rear : SONY(imx333) front : Sony(IMX320) */
#define ROM_HEADER_RTA_BINARY_LS_END_ADDR          0x124	/*RTA*/	/* rear : SONY(imx333) front : Sony(IMX320) */

#define ROM_HEADER_ISP_SETFILE_LL_START_ADDR       0x100   /* 2L2 */
#define ROM_HEADER_ISP_SETFILE_LL_END_ADDR         0x104   /* 2L2 */
#define ROM_HEADER_ISP_SETFILE_LS_START_ADDR       0x108   /* IMX333 */
#define ROM_HEADER_ISP_SETFILE_LS_END_ADDR         0x10C   /* IMX333 */

#define ROM_HEADER_ISP_FRONT_SETFILE_LL_START_ADDR 	0x110   /* 3H1 */
#define ROM_HEADER_ISP_FRONT_SETFILE_LL_END_ADDR   	0x114   /* 3H1 */
#define ROM_HEADER_ISP_FRONT_SETFILE_LS_START_ADDR     0x118   /* IMX320 */
#define ROM_HEADER_ISP_FRONT_SETFILE_LS_END_ADDR       0x11C   /* IMX320 */

#define ROM_HEADER_HIFI_TUNNINGFILE_LS_START_ADDR 0x148    /* IMX333 */
#define ROM_HEADER_HIFI_TUNNINGFILE_LS_END_ADDR   0x14C    /* IMX333 */

#define ROM_HEADER_SHADING_START_ADDR           0x00
#define ROM_HEADER_SHADING_END_ADDR             0x04
#define ROM_HEADER_PDAF_CAL_START_ADDR          0x08
#define ROM_HEADER_PDAF_CAL_END_ADDR            0x0C
#define ROM_HEADER_MODULE_ID_ADDR               0xAE
#define ROM_HEADER_SENSOR_ID_ADDR               0xB8
#define ROM_HEADER_SENSOR2_ID_ADDR              0xC8  /*TELE Sensor ID*/
#define ROM_HEADER_SENSOR_VERSION_ADDR          0xC0
#define ROM_HEADER_MTF_DATA_ADDR	     		 0x170
#define ROM_HEADER_MTF_DATA2_ADDR	     		 0x1A6

#define ROM_HEADER_CAL_DATA_START_ADDR          0x1000

/* Shading referenced section */
#define ROM_SHADING_LSC_I0_GAIN_ADDR            0x2702
#define ROM_SHADING_LSC_J0_GAIN_ADDR            0x270A
#define ROM_SHADING_LSC_A_GAIN_ADDR             0x2712
#define ROM_SHADING_LSC_K4_GAIN_ADDR            0x2722
#define ROM_SHADING_LSC_SCALE_GAIN_ADDR         0x2732
#define ROM_SHADING_GRASTUNING_AWB_ASH_CORD_ADDR        0x273A
#define ROM_SHADING_GRASTUNING_AWB_ASH_CORD_INDEX_ADDR  0x2748
#define ROM_SHADING_GRASTUNING_GAS_ALPHA_ADDR           0x2756
#define ROM_SHADING_GRASTUNING_GAS_BETA_ADDR            0x278E
#define ROM_SHADING_GRASTUNING_GAS_OUTDOOR_ALPHA_ADDR   0x27C6
#define ROM_SHADING_GRASTUNING_GAS_OUTDOOR_BETA_ADDR    0x27CE
#define ROM_SHADING_GRASTUNING_GAS_INDOOR_ALPHA_ADDR    0x27D6
#define ROM_SHADING_GRASTUNING_GAS_INDOOR_BETA_ADDR     0x27DE
#define ROM_SHADING_LSC_GAIN_START_ADDR         0x1006
#define ROM_SHADING_LSC_GAIN_END_ADDR           0x26FD
#define ROM_CAL_PAF_ERROR_CHECK_CODE_ADDR       0x2804
#define ROM_CONCORD_CAL_PDAF_SHADING_START_ADDR 0x37F4
#define ROM_CONCORD_CAL_PDAF_SHADING_END_ADDR   0x3C13

/* Shading referenced section - REAR2 (TELE) */
#define ROM_REAR2_SHADING_LSC_I0_GAIN_ADDR            0x4006
#define ROM_REAR2_SHADING_LSC_J0_GAIN_ADDR            0x4008
#define ROM_REAR2_SHADING_LSC_A_GAIN_ADDR             0x400A
#define ROM_REAR2_SHADING_LSC_K4_GAIN_ADDR            0x400E
#define ROM_REAR2_SHADING_LSC_SCALE_GAIN_ADDR         0x4012
#define ROM_REAR2_SHADING_LSC_GAIN_START_ADDR         0x4014
#define ROM_REAR2_SHADING_LSC_GAIN_END_ADDR           0x59FB
#define ROM_REAR2_CAL_PAF_ERROR_CHECK_CODE_ADDR       0x5A04
#define ROM_REAR2_CONCORD_CAL_PDAF_SHADING_START_ADDR 0x63F0
#define ROM_REAR2_CONCORD_CAL_PDAF_SHADING_END_ADDR   0x680F

/* Companion Checksum referenced section */
#define ROM_SHADING_LSC_GAIN_CRC_ADDR           0x26FE
#define ROM_CONCORD_PDAF_CRC_ADDR               0x37F0
#define ROM_CONCORD_PDAF_SHAD_CRC_ADDR          0x3C14
#define ROM_SHADING_LSC_PARAMETER_CRC_ADDR      0x27E6

/* Companion Checksum referenced section - REAR2 (TELE) */
#define ROM_REAR2_SHADING_LSC_GAIN_CRC_ADDR           0x59DC
#define ROM_REAR2_CONCORD_PDAF_SHAD_CRC_ADDR          0x6810
#define ROM_REAR2_DUAR_CAM_CAL_DATA_CRC_ADDR          0x6ABC

/* AWB referenced section */
#define ROM_REAR_AWB_MASTER_ADDR                   0x3C60
#define ROM_REAR_AWB_MODULE_ADDR                   0x3C84

/* Rear2 Cal Dual Calibration Data2 */
#define ROM_REAR2_DUAL_CAL2					0x6BC0
#define ROM_REAR2_DUAL_CAL2_SIZE				512

#define ROM_REAR2_DUAL_TILT_X					0x6C1C
#define ROM_REAR2_DUAL_TILT_Y					0x6C20
#define ROM_REAR2_DUAL_TILT_Z					0x6C24
#define ROM_REAR2_DUAL_TILT_SX					0x6C7C
#define ROM_REAR2_DUAL_TILT_SY					0x6C80
#define ROM_REAR2_DUAL_TILT_RANGE				0x6DA0
#define ROM_REAR2_DUAL_TILT_MAX_ERR			0x6DA4
#define ROM_REAR2_DUAL_TILT_AVG_ERR			0x6DA8
#define ROM_REAR2_DUAL_TILT_DLL_VERSION		0x6D9C

/* ISP binary referenced section */
#define ROM_ISP_BINARY_SETFILE_START_ADDR       0x44000
#define ROM_ISP_BINARY_SETFILE_END_ADDR         0x3FFFFF

/* Checksum referenced section */
#define ROM_CHECKSUM_HEADER_ADDR            	 0xFFC
#define ROM_CHECKSUM_CAL_DATA_ADDR              0x3FFC
#define ROM_CAL_DATA_START_ADDR                 0x1000
#define ROM_CAL_DATA_END_ADDR                   0x3C9F

#define ROM_REAR2_CHECKSUM_CAL_DATA_ADDR        0x7FFC
#define ROM_REAR2_CAL_DATA_START_ADDR           0x4000
#define ROM_REAR2_CAL_DATA_END_ADDR             0x733F

/* etc section */
#define IS_MAX_CAL_SIZE                (64 * 1024)
#define IS_MAX_COMPANION_FW_SIZE	    (110 * 1024)
#define IS_FROM_ERASE_SIZE             (64 * 1024)
#define IS_MAX_FW_SIZE                 (1600 * 1024)
#define IS_MAX_RTA_FW_SIZE             (950 * 1024)
#define IS_MAX_SETFILE_SIZE_LS	        (230 * 1024)
#define IS_MAX_SETFILE_SIZE_LL         (230 * 1024)
#define IS_MAX_SETFILE_SIZE_FRONT_LL   (230 * 1024)
#define IS_MAX_SETFILE_SIZE_FRONT_LS   (230 * 1024)
#define IS_MAX_HIFI_TUNNING_SIZE_LS    (150 * 1024)


#define HEADER_CRC32_LEN                    (0x170)
#define ROM_AF_CAL_D10_ADDR                0x3C30
/* #define ROM_AF_CAL_D20_ADDR */
/* #define ROM_AF_CAL_D30_ADDR */
#define ROM_AF_CAL_D40_ADDR                0x3C2C
#define ROM_AF_CAL_D50_ADDR                0x3C28
/* #define ROM_AF_CAL_D60_ADDR */
/* #define ROM_AF_CAL_D70_ADDR */
#define ROM_AF_CAL_D80_ADDR                0x3C20
#define ROM_AF_CAL_PAN_ADDR                0x3C24

/* REAR SECOND SENSOR (TELE) */
/* #define ROM_AF2_CAL_D10_ADDR */
/* #define ROM_AF2_CAL_D20_ADDR */
/* #define ROM_AF2_CAL_D30_ADDR */
#define ROM_AF2_CAL_D40_ADDR               0x682C
#define ROM_AF2_CAL_D50_ADDR               0x6828
/* #define ROM_AF2_CAL_D60_ADDR */
/* #define ROM_AF2_CAL_D70_ADDR */
#define ROM_AF2_CAL_D80_ADDR               0x6820
#define ROM_AF2_CAL_PAN_ADDR               0x6824
#define ROM_AF_CAL_MACRO_ADDR              0x3C30
#define ROM_AF_CAL_D1_ADDR                 0x3C2C
#define ROM_AF2_CAL_MACRO_ADDR             0x6830	/* REAR SECOND SENSOR (TELE) */
#define ROM_AF2_CAL_D1_ADDR                0x682C	/* REAR SECOND SENSOR (TELE) */

#define CHECKSUM_SEED_COMP_FW_LY		0x21EFC  	/* rear : SONY(imx333) front : S.LSI(3H1) */
#define CHECKSUM_SEED_COMP_FW_LS		0x3BFFC		/* rear : SONY(imx333) front : Sony(IMX320) */

#define CHECKSUM_SEED_SETF_LL		    0x73FFC		/* 2L2 */
#define CHECKSUM_SEED_SETF_LS	    	0xABFFC		/* IMX333 */
#define CHECKSUM_SEED_FRONT_SETF_LL		    0xE3FFC 	/* 3H1 */
#define CHECKSUM_SEED_FRONT_SETF_LS		    0x11BFFC	/* IMX320 */


#define CHECKSUM_SEED_ISP_FW_RTA_LY		0x1BBFFC
#define CHECKSUM_SEED_ISP_FW_RTA_LS		0x25BFFC
#define CHECKSUM_SEED_ISP_FW		    0x38FFFC

#define CHECKSUM_SEED_HIFI_TUNNING_LS		0x39FFFC

#define ROM_WRITE_CHECKSUM_SETF_LS	    0x156FFF
#define ROM_WRITE_CHECKSUM_COMP_LS	    0x407FF
#define ROM_WRITE_CHECKSUM_SETF_LL	    0x18DFFF
#define ROM_WRITE_CHECKSUM_COMP_LL	    0x41FFF

/* TUNNING HIFI LLS DATA */
#define IS_TUNNING_HIFILLS			"libhifills_LSIRe_imx333.dat"

#endif /* IS_FROM_REAR_2L3_V001_H */

