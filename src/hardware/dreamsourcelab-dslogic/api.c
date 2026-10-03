/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 * Copyright (C) 2013 DreamSourceLab <support@dreamsourcelab.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <math.h>
#include "protocol.h"

/* --- Test mode values (fork SR_TEST_* not in upstream) --- */
#define DSL_TEST_NONE		0
#define DSL_TEST_INTERNAL	1

/* --- Buffer option values (fork SR_BUF_* not in upstream) --- */
#define DSL_BUF_STOP		0
#define DSL_BUF_UPLOAD		1

/* ===========================================================================
 * 1. Profile table — 25 devices (14 DSLogic + 11 DSCope)
 *    Data copied from old dsl.h supported_DSLogic[] + supported_DSCope[].
 *
 *    !! THE .firmware FIELD IS *NOT* PER-MODEL — READ BEFORE "FIXING" IT !!
 *
 *    The old fork's dsl.h listed a distinct FX2 .fw name per profile
 *    (DSLogicPlus.fw, DSLogicU2Basic.fw, DSCopeC20.fw, ...), but those files
 *    were never shipped by anyone: not in the upstream DreamSourceLab/DSView
 *    repo, not in the DSView installer, not in sigrok-firmware. Copying that
 *    table verbatim gave 25 profiles 25 different .fw names of which only
 *    FOUR existed on disk, so 21 profiles failed their ezusb upload and the
 *    user saw a bogus "interface already claimed / driver problem" pair.
 *
 *    Only four FX2 firmware blobs exist at all, and all four are 8120 bytes
 *    (converged builds). The FX2 (Cypress CY7C68013A, 8051) firmware only
 *    performs the USB re-enumeration; the model-specific part is the FPGA
 *    bitstream in fpga_bit33/fpga_bit50, which stays per-profile and is
 *    present for all 21 bitstreams. sigrok's own wiki documents the same
 *    granularity: one fx2 firmware per family, mapped by rename:
 *      DSLogic.fw    -> dreamsourcelab-dslogic-fx2.fw
 *      DSLogicPro.fw -> dreamsourcelab-dslogic-pro-fx2.fw
 *      DSCope.fw     -> dreamsourcelab-dscope-fx2.fw
 *
 *    Hence each profile borrows the family's surviving blob:
 *      DSLogic family (incl. Basic/Plus/U2Basic/U3Pro/U2Pro/pgl12 variants)
 *                                                            -> DSLogic.fw
 *      DSLogic Pro                                             -> DSLogicPro.fw (exact)
 *      DSCope family  (incl. B20/C20/C20P/C20B/U2B20/U2P20/
 *                      U2B100/U3P100)                          -> DSCope20.fw
 *      DSLogic v1 / DSCope v1 -> DSLogic.fw / DSCope.fw        (exact)
 *
 *    UNVERIFIED: that a family-shared fx2 blob re-enumerates every PID in
 *    that family. This needs a real device to confirm — see the 0x0030
 *    (DSLogic Plus, pgl12) report. If a PID still fails to enumerate, the
 *    fix is that PID's .firmware field, NOT reverting the table to the
 *    non-existent per-model names.
 * =========================================================================== */
static const struct DSL_profile supported_device[] = {
	/*
	 * DSLogic (14 profiles)
	 */
	{DS_VENDOR_ID, 0x0001, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic", NULL,
	 "DSLogic.fw",
	 "DSLogic33.bin",
	 "DSLogic50.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_SEEP | CAPS_FEATURE_BUF,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4) |
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  16,
	  SR_MB(256),
	  SR_Mn(2),
	  DSL_BUFFER100x16,
	  vdivs10to2000,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{DS_VENDOR_ID, 0x0003, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic Pro", NULL,
	 "DSLogicPro.fw",	/* exact */
	 "DSLogicPro.bin",
	 "DSLogicPro.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_SEEP | CAPS_FEATURE_VTH | CAPS_FEATURE_BUF,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4),
	  16,
	  SR_MB(256),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{DS_VENDOR_ID, 0x0020, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic PLus", NULL,
	 "DSLogic.fw",	/* family FX2 blob */
	 "DSLogicPlus.bin",
	 "DSLogicPlus.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4),
	  16,
	  SR_MB(256),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{DS_VENDOR_ID, 0x0021, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic Basic", NULL,
	 "DSLogic.fw",
	 "DSLogicBasic.bin",
	 "DSLogicBasic.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4),
	  16,
	  SR_KB(256),
	  0,
	  DSL_STREAM20x16,
	  0,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{DS_VENDOR_ID, 0x0029, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U2Basic", NULL,
	 "DSLogic.fw",
	 "DSLogicU2Basic.bin",
	 "DSLogicU2Basic.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16),
	  16,
	  SR_MB(64),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates100,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{DS_VENDOR_ID, 0x002A, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U3Pro16", NULL,
	 "DSLogic.fw",
	 "DSLogicU3Pro16.bin",
	 "DSLogicU3Pro16.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_USB30 | CAPS_FEATURE_ADF4360,
	  (1 << DSL_STREAM20x16_3DN2) | (1 << DSL_STREAM25x12_3DN2) | (1 << DSL_STREAM50x6_3DN2) | (1 << DSL_STREAM100x3_3DN2) |
	  (1 << DSL_BUFFER500x16) | (1 << DSL_BUFFER1000x8),
	  16,
	  SR_GB(2),
	  0,
	  DSL_BUFFER500x16,
	  0,
	  samplerates1000,
	  0,
	  DSL_STREAM20x16_3DN2,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(500),
	  SR_GHZ(1)}
	},

	{DS_VENDOR_ID, 0x002A, LIBUSB_SPEED_SUPER, "DreamSourceLab", "DSLogic U3Pro16", NULL,
	 "DSLogic.fw",
	 "DSLogicU3Pro16.bin",
	 "DSLogicU3Pro16.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_USB30 | CAPS_FEATURE_ADF4360,
	  (1 << DSL_STREAM125x16_16) | (1 << DSL_STREAM250x12_16) | (1 << DSL_STREAM500x6) | (1 << DSL_STREAM1000x3) |
	  (1 << DSL_BUFFER500x16) | (1 << DSL_BUFFER1000x8),
	  16,
	  SR_GB(2),
	  0,
	  DSL_BUFFER500x16,
	  0,
	  samplerates1000,
	  0,
	  DSL_STREAM125x16_16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(500),
	  SR_GHZ(1)}
	},

	{DS_VENDOR_ID, 0x002C, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U3Pro32", NULL,
	 "DSLogic.fw",
	 "DSLogicU3Pro32.bin",
	 "DSLogicU3Pro32.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_USB30 | CAPS_FEATURE_ADF4360 | CAPS_FEATURE_LA_CH32,
	  (1 << DSL_STREAM10x32_32_3DN2) | (1 << DSL_STREAM20x16_32_3DN2) | (1 << DSL_STREAM25x12_32_3DN2) | (1 << DSL_STREAM50x6_32_3DN2) | (1 << DSL_STREAM100x3_32_3DN2) |
	  (1 << DSL_BUFFER250x32) | (1 << DSL_BUFFER500x16) | (1 << DSL_BUFFER1000x8),
	  32,
	  SR_GB(2),
	  0,
	  DSL_BUFFER250x32,
	  0,
	  samplerates1000,
	  0,
	  DSL_STREAM10x32_32_3DN2,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(500),
	  SR_GHZ(1)}
	},

	{DS_VENDOR_ID, 0x002C, LIBUSB_SPEED_SUPER, "DreamSourceLab", "DSLogic U3Pro32", NULL,
	 "DSLogic.fw",
	 "DSLogicU3Pro32.bin",
	 "DSLogicU3Pro32.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_USB30 | CAPS_FEATURE_ADF4360 | CAPS_FEATURE_LA_CH32,
	  (1 << DSL_STREAM50x32) | (1 << DSL_STREAM100x30) | (1 << DSL_STREAM250x12) | (1 << DSL_STREAM500x6) | (1 << DSL_STREAM1000x3) |
	  (1 << DSL_BUFFER250x32) | (1 << DSL_BUFFER500x16) | (1 << DSL_BUFFER1000x8),
	  32,
	  SR_GB(2),
	  0,
	  DSL_BUFFER250x32,
	  0,
	  samplerates1000,
	  0,
	  DSL_STREAM50x32,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(500),
	  SR_GHZ(1)}
	},

	{DS_VENDOR_ID, 0x002D, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U2Pro16", NULL,
	 "DSLogic.fw",
	 "DSLogicU2Pro16.bin",
	 "DSLogicU2Pro16.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_ADF4360 | CAPS_FEATURE_SECURITY,
	  (1 << DSL_STREAM20x16_3DN2) | (1 << DSL_STREAM25x12_3DN2) | (1 << DSL_STREAM50x6_3DN2) | (1 << DSL_STREAM100x3_3DN2) |
	  (1 << DSL_BUFFER500x16) | (1 << DSL_BUFFER1000x8),
	  16,
	  SR_GB(4),
	  0,
	  DSL_BUFFER500x16,
	  0,
	  samplerates1000,
	  0,
	  DSL_STREAM20x16_3DN2,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(500),
	  SR_GHZ(1)}
	},

	{0x2A0E, 0x0030, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic PLus", NULL,
	 "DSLogic.fw",	/* family FX2 blob -- 0x0030 report, UNVERIFIED */
	 "DSLogicPlus-pgl12.bin",
	 "DSLogicPlus-pgl12.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_MAX25_VTH | CAPS_FEATURE_SECURITY,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4),
	  16,
	  SR_MB(256),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{0x2A0E, 0x0031, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U2Basic", NULL,
	 "DSLogic.fw",
	 "DSLogicU2Basic-pgl12.bin",
	 "DSLogicU2Basic-pgl12.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_MAX25_VTH | CAPS_FEATURE_SECURITY,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16),
	  16,
	  SR_MB(64),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates100,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{0x2A0E, 0x0034, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic PLus", NULL,
	 "DSLogic.fw",
	 "DSLogicPlus-pgl12-2.bin",
	 "DSLogicPlus-pgl12-2.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_MAX25_VTH | CAPS_FEATURE_SECURITY,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16) | (1 << DSL_BUFFER200x8) | (1 << DSL_BUFFER400x4),
	  16,
	  SR_MB(256),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates400,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	{0x2A0E, 0x0035, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSLogic U2Basic", NULL,
	 "DSLogic.fw",
	 "DSLogicU2Basic-pgl12-2.bin",
	 "DSLogicU2Basic-pgl12-2.bin",
	 {CAPS_MODE_LOGIC,
	  CAPS_FEATURE_VTH | CAPS_FEATURE_BUF | CAPS_FEATURE_MAX25_VTH | CAPS_FEATURE_SECURITY,
	  (1 << DSL_STREAM20x16) | (1 << DSL_STREAM25x12) | (1 << DSL_STREAM50x6) | (1 << DSL_STREAM100x3) |
	  (1 << DSL_BUFFER100x16),
	  16,
	  SR_MB(64),
	  0,
	  DSL_BUFFER100x16,
	  0,
	  samplerates100,
	  0,
	  DSL_STREAM20x16,
	  SR_MHZ(1),
	  SR_Mn(1),
	  0,
	  0,
	  0,
	  0,
	  0,
	  SR_MHZ(200),
	  SR_MHZ(400)}
	},

	/*
	 * DSCope (11 profiles)
	 */
	{DS_VENDOR_ID, 0x0002, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope", NULL,
	 "DSCope.fw",
	 "DSCope.bin",
	 "DSCope.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_PREOFF | CAPS_FEATURE_SEEP | CAPS_FEATURE_BUF,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  1,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  (129 << 8) + 167,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0004, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope20", NULL,
	 "DSCope20.fw",
	 "DSCope20.bin",
	 "DSCope20.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_SEEP | CAPS_FEATURE_BUF,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  2,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  920,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0022, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope B20", NULL,
	 "DSCope20.fw",
	 "DSCope20.bin",
	 "DSCope20.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_BUF,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  2,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  920,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0023, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope C20", NULL,
	 "DSCope20.fw",	/* family FX2 blob */
	 "DSCopeC20P.bin",
	 "DSCopeC20P.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_BUF,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  3,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  920,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0024, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope C20P", NULL,
	 "DSCope20.fw",
	 "DSCopeC20P.bin",
	 "DSCopeC20P.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_BUF | CAPS_FEATURE_POGOPIN,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  3,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  920,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0025, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope C20", NULL,
	 "DSCope20.fw",
	 "DSCopeC20B.bin",
	 "DSCopeC20B.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_KB(256),
	  SR_Kn(20),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  3,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Kn(10),
	  920,
	  1024 - 920,
	  1,
	  255,
	  0,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0026, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope U2B20", NULL,
	 "DSCope20.fw",
	 "DSCopeU2B20.bin",
	 "DSCopeU2B20.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_AUTO_VGAIN,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_KB(256),
	  SR_Kn(20),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  4,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Kn(10),
	  930,
	  1024 - 930,
	  10,
	  245,
	  22,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0027, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope U2P20", NULL,
	 "DSCope20.fw",
	 "DSCopeU2P20.bin",
	 "DSCopeU2P20.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_BUF | CAPS_FEATURE_POGOPIN | CAPS_FEATURE_AUTO_VGAIN,
	  (1 << DSL_ANALOG10x2) |
	  (1 << DSL_DSO200x2),
	  2,
	  SR_MB(256),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates400,
	  4,
	  DSL_DSO200x2,
	  SR_MHZ(100),
	  SR_Mn(1),
	  930,
	  1024 - 930,
	  10,
	  245,
	  22,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x0028, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope U2B100", NULL,
	 "DSCope20.fw",
	 "DSCopeU2B100.bin",
	 "DSCopeU2B100.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_HMCAD1511 | CAPS_FEATURE_20M,
	  (1 << DSL_ANALOG10x2_500) |
	  (1 << DSL_DSO1000x2),
	  2,
	  SR_KB(256),
	  SR_Kn(20),
	  0,
	  vdivs10to2000,
	  samplerates1000,
	  4,
	  DSL_DSO1000x2,
	  SR_MHZ(500),
	  SR_Kn(10),
	  835,
	  1024 - 835,
	  10,
	  245,
	  60,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x002B, LIBUSB_SPEED_HIGH, "DreamSourceLab", "DSCope U3P100", NULL,
	 "DSCope20.fw",	/* family FX2 blob */
	 "DSCopeU3P100.bin",
	 "DSCopeU3P100.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_POGOPIN | CAPS_FEATURE_FLASH | CAPS_FEATURE_USB30 | CAPS_FEATURE_HMCAD1511 | CAPS_FEATURE_20M,
	  (1 << DSL_ANALOG10x2_500) |
	  (1 << DSL_DSO1000x2),
	  2,
	  SR_GB(2),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates1000,
	  5,
	  DSL_DSO1000x2,
	  SR_MHZ(500),
	  SR_Mn(1),
	  780,
	  1024 - 780,
	  10,
	  245,
	  60,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	{DS_VENDOR_ID, 0x002B, LIBUSB_SPEED_SUPER, "DreamSourceLab", "DSCope U3P100", NULL,
	 "DSCope20.fw",
	 "DSCopeU3P100.bin",
	 "DSCopeU3P100.bin",
	 {CAPS_MODE_ANALOG | CAPS_MODE_DSO,
	  CAPS_FEATURE_ZERO | CAPS_FEATURE_POGOPIN | CAPS_FEATURE_FLASH | CAPS_FEATURE_USB30 | CAPS_FEATURE_HMCAD1511 | CAPS_FEATURE_20M,
	  (1 << DSL_ANALOG10x2_500) |
	  (1 << DSL_DSO1000x2),
	  2,
	  SR_GB(2),
	  SR_Mn(2),
	  0,
	  vdivs10to2000,
	  samplerates1000,
	  5,
	  DSL_DSO1000x2,
	  SR_MHZ(500),
	  SR_Mn(1),
	  780,
	  1024 - 780,
	  10,
	  245,
	  60,
	  SR_HZ(0),
	  SR_HZ(0)}
	},

	/* Terminator — ALL_ZERO equivalent (ALL_ZERO not defined in protocol.h) */
	{0, 0, LIBUSB_SPEED_UNKNOWN, NULL, NULL, NULL, NULL, NULL, NULL, {0}}
};

/* ===========================================================================
 * 2. Static data arrays
 * =========================================================================== */
static const uint32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CONTINUOUS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
	SR_CONF_CAPTUREFILE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_NUM_VDIV | SR_CONF_GET,
	SR_CONF_CLOCK_TYPE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_CLOCK_EDGE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_EXTERNAL_CLOCK | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_OPERATION_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFER_OPTIONS | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_CHANNEL_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_VTH | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_FILTER | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_MAX_HEIGHT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_MAX_HEIGHT_VALUE | SR_CONF_GET,
	SR_CONF_RLE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_RLE_SUPPORT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_INSTANT | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DEVICE_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LOOP_MODE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PROBE_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_EN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_CHANNEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_HOLDOFF | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_MARGIN | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_MAX_DSO_SAMPLERATE | SR_CONF_GET,
	SR_CONF_MAX_DSO_SAMPLELIMITS | SR_CONF_GET,
	SR_CONF_HW_DEPTH | SR_CONF_GET,
	SR_CONF_VLD_CH_NUM | SR_CONF_GET,
	SR_CONF_TOTAL_CH_NUM | SR_CONF_GET,
	SR_CONF_STREAM | SR_CONF_GET,
	SR_CONF_WAIT_UPLOAD | SR_CONF_GET,
	SR_CONF_BANDWIDTH_LIMIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_USB_SPEED | SR_CONF_GET,
	SR_CONF_USB30_SUPPORT | SR_CONF_GET,
	/* DSO/ANALOG-specific keys (added to match demo driver coverage). */
	SR_CONF_TRIGGER_VALUE | SR_CONF_GET | SR_CONF_SET,    /* DSO trig level (per-channel) */
	SR_CONF_PROBE_HW_OFFSET | SR_CONF_GET | SR_CONF_SET,  /* hardware offset (per-channel) */
	SR_CONF_MAX_TIMEBASE | SR_CONF_GET,                  /* max DSO timebase (ns) */
	SR_CONF_MIN_TIMEBASE | SR_CONF_GET,                  /* min DSO timebase (ns) */
	SR_CONF_REF_MIN | SR_CONF_GET,                       /* ADC reference min */
	SR_CONF_REF_MAX | SR_CONF_GET,                       /* ADC reference max */
	SR_CONF_UNIT_BITS | SR_CONF_GET,                     /* sample bits per channel */
	SR_CONF_PROBE_MAP_DEFAULT | SR_CONF_GET | SR_CONF_SET,    /* use default probe mapping */
	SR_CONF_PROBE_MAP_UNIT | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,  /* probe map unit */
	SR_CONF_PROBE_MAP_MIN | SR_CONF_GET | SR_CONF_SET,    /* probe map min */
	SR_CONF_PROBE_MAP_MAX | SR_CONF_GET | SR_CONF_SET,    /* probe map max */
	SR_CONF_PROBE_CONFIGS | SR_CONF_LIST,                 /* probe config key list */
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,  /* waveform pattern */
};

/* ===========================================================================
 * 3. String arrays for config_list
 * =========================================================================== */
static const char *opmode_strs[] = {
	"Buffer Mode",
	"Stream Mode",
	"Internal Test",
};

static const char *bufoption_strs[] = {
	"Stop immediately",
	"Upload captured data",
};

static const char *threshold_strs[] = {
	"1.8/2.5/3.3V Level",
	"5.0V Level",
};

static const char *filter_strs[] = {
	"None",
	"1 Sample Clock",
};

static const char *max_height_strs[] = {
	"1X", "2X", "3X", "4X", "5X",
};

static const char *signal_edges[] = {
	"rising",
	"falling",
};

static const char *coupling_strs[] = {
	"GND", "DC", "AC",
};

static const char *device_mode_strs[] = {
	"Logic Analyzer",
	"Oscilloscope",
};

static const char *bandwidth_strs[] = {
	"Full Bandwidth",
	"20MHz",
};

static const double thresholds[][2] = {
	{0.7, 1.4},
	{1.4, 3.6},
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

/* DSO/ANALOG pattern-mode strings. Mirrors demo driver's dso_pattern_strs[]
 * so the GUI pattern dropdown shows the same options. For real DSLogic
 * hardware the waveform is supplied by the device itself (these strings
 * are accepted/stored for GUI compat but have no device-side effect). */
static const char *dso_pattern_strs[] = {
	"random",
	"sine",
	"square",
	"sawtooth",
	"triangle",
};

/* Probe mapping units — units selectable for the probe user-defined
 * voltage/current/etc mapping (SR_CONF_PROBE_MAP_UNIT). Mirrors demo
 * driver's dso_map_units[] so the GUI dropdown is consistent. */
static const char *probe_map_units[] = { "V", "A", "°C", "°F", "g", "m", "m/s" };

/* Probe-config keys exposed via SR_CONF_PROBE_CONFIGS for ProbeOptions
 * binding. The binding iterates this list and creates widgets for
 * VDIV/COUPLING/MAP_* keys. Applies to both DSO (oscilloscope) and
 * ANALOG (DAQ) channels. */
static const int32_t probe_configs[] = {
	SR_CONF_PROBE_VDIV,
	SR_CONF_PROBE_COUPLING,
	SR_CONF_PROBE_MAP_DEFAULT,
	SR_CONF_PROBE_MAP_UNIT,
	SR_CONF_PROBE_MAP_MIN,
	SR_CONF_PROBE_MAP_MAX,
};

/* ===========================================================================
 * Helper functions
 * =========================================================================== */

/* Count entries in a 0-terminated uint64_t array. */
static unsigned int u64_array_count(const uint64_t *arr)
{
	unsigned int n = 0;

	if (!arr)
		return 0;
	while (arr[n])
		n++;
	return n;
}

/* Check whether a USB device descriptor matches any known profile. */
static gboolean is_plausible(const struct libusb_device_descriptor *des,
			     enum libusb_speed speed)
{
	unsigned int i;

	for (i = 0; supported_device[i].vid; i++) {
		if (des->idVendor != supported_device[i].vid)
			continue;
		if (des->idProduct != supported_device[i].pid)
			continue;
		if (speed == supported_device[i].usb_speed)
			return TRUE;
	}
	return FALSE;
}

/* ===========================================================================
 * 4. scan()
 * =========================================================================== */
static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct sr_config *src;
	const struct DSL_profile *prof;
	GSList *l, *devices, *conn_devices;
	gboolean has_firmware;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	struct libusb_device_handle *hdl;
	int ret, i, j;
	const char *conn;
	char manufacturer[64], product[64], serial_num[64], connection_id[64];
	char channel_name[16];
	enum libusb_speed usb_speed;
	struct dsl_channel_priv *ch_priv;
	int num_ch;

	drvc = di->context;

	conn = NULL;
	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	/* Find all DSLogic/DSCope compatible devices and upload firmware. */
	devices = NULL;
	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if (conn) {
			usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
				    && usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				continue;
		}

		libusb_get_device_descriptor(devlist[i], &des);

		usb_speed = libusb_get_device_speed(devlist[i]);
		if (usb_speed != LIBUSB_SPEED_HIGH && usb_speed != LIBUSB_SPEED_SUPER) {
			sr_info("USB speed too low: %d", usb_speed);
			continue;
		}

		if (!is_plausible(&des, usb_speed))
			continue;

		if ((ret = libusb_open(devlist[i], &hdl)) < 0) {
			sr_warn("Failed to open potential device with "
				"VID:PID %04x:%04x: %s.", des.idVendor,
				des.idProduct, libusb_error_name(ret));
			continue;
		}

		if (des.iManufacturer == 0) {
			manufacturer[0] = '\0';
		} else if ((ret = libusb_get_string_descriptor_ascii(hdl,
				des.iManufacturer, (unsigned char *)manufacturer,
				sizeof(manufacturer))) < 0) {
			sr_warn("Failed to get manufacturer string descriptor: %s.",
				libusb_error_name(ret));
			continue;
		}

		if (des.iProduct == 0) {
			product[0] = '\0';
		} else if ((ret = libusb_get_string_descriptor_ascii(hdl,
				des.iProduct, (unsigned char *)product,
				sizeof(product))) < 0) {
			sr_warn("Failed to get product string descriptor: %s.",
				libusb_error_name(ret));
			continue;
		}

		if (des.iSerialNumber == 0) {
			serial_num[0] = '\0';
		} else if ((ret = libusb_get_string_descriptor_ascii(hdl,
				des.iSerialNumber, (unsigned char *)serial_num,
				sizeof(serial_num))) < 0) {
			sr_warn("Failed to get serial number string descriptor: %s.",
				libusb_error_name(ret));
			continue;
		}

		libusb_close(hdl);

		if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
			continue;

		/* Match against supported_device[] by vid/pid/usb_speed. */
		prof = NULL;
		for (j = 0; supported_device[j].vid; j++) {
			if (des.idVendor == supported_device[j].vid &&
			    des.idProduct == supported_device[j].pid &&
			    usb_speed == supported_device[j].usb_speed) {
				prof = &supported_device[j];
				break;
			}
		}
		if (!prof)
			continue;

		sdi = g_malloc0(sizeof(struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup(prof->vendor);
		sdi->model = g_strdup(prof->model);
		sdi->version = g_strdup(prof->model_version);
		sdi->serial_num = g_strdup(serial_num);
		sdi->connection_id = g_strdup(connection_id);

		/* Create dev_context and set defaults from profile. */
		devc = dslogic_dev_new();
		devc->profile = prof;
		sdi->priv = devc;

		devc->cur_samplerate = prof->dev_caps.default_samplerate;
		devc->limit_samples = prof->dev_caps.default_samplelimit;
		devc->ch_mode = prof->dev_caps.default_channelmode;
		devc->mode = channel_modes[devc->ch_mode].mode;
		devc->op_mode = LO_OP_STREAM;
		devc->stream = TRUE;
		devc->status = DSL_FINISH;
		devc->vth = 1.0;
		devc->th_level = 0;	/* SR_TH_3V3 */
		devc->filter = 0;	/* SR_FILTER_NONE */
		devc->buf_options = DSL_BUF_UPLOAD;
		devc->timebase = 10000;
		devc->trigger_slope = 0;	/* DSO_TRIGGER_RISING */
		devc->trigger_source = 0;	/* DSO_TRIGGER_AUTO */
		devc->trigger_hpos = 0;
		devc->trigger_hrate = 0;
		devc->trigger_holdoff = 0;
		devc->trigger_margin = 8;
		devc->trigger_channel = 0;
		devc->rle_support = TRUE;
		devc->max_height = 0;
		devc->is_loop = 0;
		devc->test_mode = DSL_TEST_NONE;

		/* Create channels for the default channel mode. */
		num_ch = channel_modes[devc->ch_mode].num;
		cg = sr_channel_group_new(sdi,
			(channel_modes[devc->ch_mode].type == SR_CHANNEL_DSO) ? "DSO" :
			(channel_modes[devc->ch_mode].type == SR_CHANNEL_ANALOG) ? "Analog" :
			"Logic", NULL);
		for (j = 0; j < num_ch; j++) {
			snprintf(channel_name, sizeof(channel_name), "%d", j);
			ch = sr_channel_new(sdi, j,
				channel_modes[devc->ch_mode].type, TRUE, channel_name);
			ch_priv = g_malloc0(sizeof(struct dsl_channel_priv));
			ch_priv->vdiv = (prof->dev_caps.vdivs) ? prof->dev_caps.vdivs[0] : 0;
			ch_priv->vfactor = 1;
			ch->priv = ch_priv;
			cg->channels = g_slist_append(cg->channels, ch);
		}

		/* Check if firmware is already loaded.
		 *
		 * Two product strings must be accepted; testing only one is what
		 * made every already-flashed device look unflashed. See the
		 * DSL_PROD_STRING_* definitions in protocol.h for how each was
		 * established (the DreamSourceLab one is decoded straight out of
		 * the shipped .fw blobs).
		 *
		 * The comparison is strcmp-derived (usb_match_manuf_prod() in
		 * usb.c), i.e. exact -- it does NOT prefix-match, so the shorter
		 * upstream string does not subsume the DreamSourceLab one. */
		has_firmware = usb_match_manuf_prod(devlist[i],
			DSL_MANUF_STRING, DSL_PROD_STRING_DSVIEW);
		if (!has_firmware)
			has_firmware = usb_match_manuf_prod(devlist[i],
				DSL_MANUF_STRING, DSL_PROD_STRING_UPSTREAM);
		/* Fall back to the fork's own probe. Unlike usb_match_manuf_prod()
		 * this one reports TRUE when the handle cannot be opened at all
		 * ("maybe the device is busy; assume it matches"), which keeps a
		 * device that is temporarily claimed by something else from being
		 * misdiagnosed as unflashed and re-flashed. */
		if (!has_firmware)
			has_firmware = dsl_check_conf_profile(devlist[i]);

		if (has_firmware) {
			sr_dbg("Found a DSLogic/DSCope device.");
			sdi->status = SR_ST_INACTIVE;
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i]), NULL);
		} else {
			/* The FX2 has not been flashed yet, so it must be uploaded
			 * now. Verify the file exists BEFORE handing the device to
			 * libusb: ezusb_upload_firmware() on a missing file leaves
			 * the handle claimed and the device unrenumerated, which the
			 * user only sees as "interface already claimed" / "driver
			 * problem" -- with no mention of the real cause. */
			if (!dslogic_firmware_exists(drvc->sr_ctx, prof->firmware)) {
				dslogic_report_missing_firmware(drvc->sr_ctx, prof,
								prof->firmware);
				/* Do NOT publish this device: it can never be
				 * opened, and listing it would invite the user to
				 * select it and hit the very same bogus error.
				 * dsl_destroy_device() frees sdi->priv and
				 * sdi->conn; sr_dev_inst_free() (called inside it)
				 * frees channels, channel groups and the strings.
				 * sdi->driver is still unset here, so the
				 * dev_close() inside dsl_destroy_device() is a
				 * no-op -- the device was never opened. */
				dsl_destroy_device(sdi);
				continue;
			}

			if (ezusb_upload_firmware(drvc->sr_ctx, devlist[i],
					USB_CONFIGURATION, prof->firmware) == SR_OK) {
				devc->fw_updated = g_get_monotonic_time();
			} else {
				sr_err("Firmware upload failed for "
				       "device %d.%d (logical), name %s.",
				       libusb_get_bus_number(devlist[i]),
				       libusb_get_device_address(devlist[i]),
				       prof->firmware);
			}
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(
				libusb_get_bus_number(devlist[i]),
				0xff, NULL);
		}

		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

/* ===========================================================================
 * 5. dev_open()
 * =========================================================================== */
static int dev_open(struct sr_dev_inst *sdi)
{
	struct sr_dev_driver *di = sdi->driver;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	int ret;
	int64_t timediff_us, timediff_ms;

	devc = sdi->priv;
	usb = sdi->conn;

	/*
	 * If the firmware was recently uploaded, wait up to MAX_RENUM_DELAY_MS
	 * milliseconds for the FX2 to renumerate.
	 */
	ret = SR_ERR;
	if (devc->fw_updated > 0) {
		sr_info("Waiting for device to reset.");
		g_usleep(300 * 1000);
		timediff_ms = 0;
		while (timediff_ms < MAX_RENUM_DELAY_MS) {
			if ((ret = dslogic_dev_open(sdi, di)) == SR_OK)
				break;
			g_usleep(100 * 1000);

			timediff_us = g_get_monotonic_time() - devc->fw_updated;
			timediff_ms = timediff_us / 1000;
			sr_spew("Waited %" PRIi64 "ms.", timediff_ms);
		}
		if (ret != SR_OK) {
			sr_err("Device failed to renumerate.");
			return SR_ERR;
		}
		sr_info("Device came back after %" PRIi64 "ms.", timediff_ms);
	} else {
		sr_info("Firmware upload was not needed.");
		ret = dslogic_dev_open(sdi, di);
	}

	if (ret != SR_OK) {
		sr_err("Unable to open device.");
		return SR_ERR;
	}

	ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (ret != 0) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err("Unable to claim USB interface. Another "
			       "program or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.",
			       libusb_error_name(ret));
			break;
		}
		return SR_ERR;
	}

	if ((ret = dslogic_fpga_firmware_upload(sdi)) != SR_OK)
		return ret;

	if (devc->cur_samplerate == 0)
		devc->cur_samplerate = devc->profile->dev_caps.default_samplerate;

	if (devc->vth == 0.0) {
		devc->vth = thresholds[1][0];
		ret = dslogic_set_voltage_threshold(sdi, devc->vth);
		if (ret != SR_OK)
			return ret;
	}

	return SR_OK;
}

/* ===========================================================================
 * 6. dev_close()
 * =========================================================================== */
static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;

	if (!usb->devhdl)
		return SR_ERR_BUG;

	sr_info("Closing device on %d.%d (logical) / %s (physical) interface %d.",
		usb->bus, usb->address, sdi->connection_id, USB_INTERFACE);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

/* ===========================================================================
 * 7. config_get()
 * =========================================================================== */
static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_channel *ch;
	int idx;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	ch = (cg && cg->channels) ? cg->channels->data : NULL;

	switch (key) {
	case SR_CONF_CONN:
		if (!sdi->conn)
			return SR_ERR_ARG;
		usb = sdi->conn;
		if (usb->address == 255)
			return SR_ERR;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_CONTINUOUS:
		*data = g_variant_new_boolean(devc->continuous_mode);
		break;
	case SR_CONF_EXTERNAL_CLOCK:
		*data = g_variant_new_boolean(devc->external_clock);
		break;
	case SR_CONF_CLOCK_TYPE:
		*data = g_variant_new_boolean(devc->clock_type);
		break;
	case SR_CONF_CLOCK_EDGE:
		/* Return string to match config_list signal_edges[] and GUI bind_list. */
		*data = g_variant_new_string(signal_edges[devc->clock_edge ? 1 : 0]);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_string(opmode_strs[devc->op_mode]);
		break;
	case SR_CONF_BUFFER_OPTIONS:
		*data = g_variant_new_string(bufoption_strs[devc->buf_options]);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_string(channel_modes[devc->ch_mode].descr);
		break;
	case SR_CONF_THRESHOLD:
		*data = g_variant_new_string(threshold_strs[devc->th_level]);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_string(filter_strs[devc->filter]);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(max_height_strs[devc->max_height]);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		*data = g_variant_new_byte(devc->max_height);
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(devc->rle_mode);
		break;
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(devc->rle_support);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(devc->stream);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(devc->is_loop);
		break;
	case SR_CONF_DEVICE_MODE:
		/* hwdriver.c declares SR_T_INT16. Return int16 to match the table
		 * and the demo/pxlogic drivers. The string list is still available
		 * via config_list (SR_CONF_LIST path below). */
		*data = g_variant_new_int16((int16_t)devc->mode);
		break;
	case SR_CONF_PROBE_VDIV:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_uint64(DSL_CH_PRIV(ch)->vdiv);
		break;
	case SR_CONF_PROBE_FACTOR:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_uint64(DSL_CH_PRIV(ch)->vfactor);
		break;
	case SR_CONF_PROBE_EN:
		if (!ch)
			return SR_ERR_ARG;
		*data = g_variant_new_boolean(ch->enabled);
		break;
	case SR_CONF_PROBE_COUPLING:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		/* Return int32 to match hwdriver.c SR_T_INT32 and demo driver. */
		*data = g_variant_new_int32((int32_t)DSL_CH_PRIV(ch)->coupling);
		break;
	case SR_CONF_PROBE_OFFSET:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_uint16(DSL_CH_PRIV(ch)->offset);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		*data = g_variant_new_byte(devc->trigger_slope);
		break;
	case SR_CONF_TRIGGER_SOURCE:
		*data = g_variant_new_byte(devc->trigger_source & 0x0f);
		break;
	case SR_CONF_TRIGGER_CHANNEL:
		*data = g_variant_new_byte(devc->trigger_source >> 4);
		break;
	case SR_CONF_TRIGGER_HOLDOFF:
		*data = g_variant_new_uint64(devc->trigger_holdoff);
		break;
	case SR_CONF_TRIGGER_MARGIN:
		*data = g_variant_new_byte(devc->trigger_margin);
		break;
	case SR_CONF_HORIZ_TRIGGERPOS:
		/* hwdriver.c declares SR_T_FLOAT (GVariant 'd' double).
		 * DSL drivers store percentage as uint8_t; convert to/from double. */
		if (devc->mode == DSL_MODE_DSO)
			*data = g_variant_new_double((double)devc->trigger_hrate);
		else
			*data = g_variant_new_double((double)devc->trigger_hpos);
		break;
	case SR_CONF_MAX_DSO_SAMPLERATE:
		*data = g_variant_new_uint64(channel_modes[devc->ch_mode].max_samplerate);
		break;
	case SR_CONF_MAX_DSO_SAMPLELIMITS:
		*data = g_variant_new_uint64(devc->profile->dev_caps.dso_depth);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(dsl_channel_depth(sdi));
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int32(channel_modes[devc->ch_mode].vld_num);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(devc->profile->dev_caps.total_ch_num);
		break;
	case SR_CONF_NUM_HDIV:
		*data = g_variant_new_int32(10);
		break;
	case SR_CONF_NUM_VDIV:
		*data = g_variant_new_int32(10);
		break;
	case SR_CONF_USB_SPEED:
		*data = g_variant_new_string(
			devc->profile->usb_speed == LIBUSB_SPEED_SUPER ? "super" : "high");
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(
			(devc->profile->dev_caps.feature_caps & CAPS_FEATURE_USB30) != 0);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		if (!strcmp(devc->profile->model, "DSLogic")) {
			if ((idx = std_double_tuple_idx_d0(devc->vth,
					ARRAY_AND_SIZE(thresholds))) < 0)
				return SR_ERR_BUG;
			*data = std_gvar_tuple_double(thresholds[idx][0],
					thresholds[idx][1]);
		} else {
			*data = std_gvar_tuple_double(devc->vth, devc->vth);
		}
		break;
	case SR_CONF_WAIT_UPLOAD:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_BANDWIDTH_LIMIT:
		*data = g_variant_new_string(bandwidth_strs[devc->bw_limit]);
		break;
	case SR_CONF_TIMEBASE:
		*data = g_variant_new_uint64(devc->timebase);
		break;
	case SR_CONF_INSTANT:
		*data = g_variant_new_boolean(devc->instant);
		break;
	case SR_CONF_TRIGGER_VALUE:
		/* DSO trigger level (per-channel). Returns the software-side trig
		 * value stored in dsl_channel_priv.trig_value (8-bit). The
		 * hardware-side value is updated by get_measure() on receive.
		 * Return int32 to match hwdriver.c SR_T_INT32. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_int32((int32_t)DSL_CH_PRIV(ch)->trig_value);
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		/* Hardware offset (per-channel). Read-only from GUI's perspective
		 * in normal operation (updated by get_measure()), but we expose
		 * SET so the GUI can override during manual calibration. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_uint16(DSL_CH_PRIV(ch)->hw_offset);
		break;
	case SR_CONF_MAX_TIMEBASE:
		/* Maximum timebase in ns: derived from dso_depth, channel-mode
		 * num, and min_samplerate across 10 horizontal divisions. */
		*data = g_variant_new_uint64(
			UINT64_C(1000000000) * devc->profile->dev_caps.dso_depth /
			channel_modes[devc->ch_mode].num /
			channel_modes[devc->ch_mode].min_samplerate / 10);
		break;
	case SR_CONF_MIN_TIMEBASE:
		/* Minimum timebase in ns: 1 / hw_max_samplerate, in ns. */
		*data = g_variant_new_uint64(
			UINT64_C(1000000000) / channel_modes[devc->ch_mode].hw_max_samplerate);
		break;
	case SR_CONF_REF_MIN:
		/* ADC reference minimum — compile-time constant from profile. */
		*data = g_variant_new_uint32(devc->profile->dev_caps.ref_min);
		break;
	case SR_CONF_REF_MAX:
		/* ADC reference maximum — compile-time constant from profile. */
		*data = g_variant_new_uint32(devc->profile->dev_caps.ref_max);
		break;
	case SR_CONF_UNIT_BITS:
		/* Sample resolution in bits for the current channel mode. */
		*data = g_variant_new_byte(channel_modes[devc->ch_mode].unit_bits);
		break;
	case SR_CONF_PROBE_MAP_DEFAULT:
		/* Whether the probe uses the default (built-in) mapping. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_boolean(DSL_CH_PRIV(ch)->map_default);
		break;
	case SR_CONF_PROBE_MAP_UNIT:
		/* Probe mapping unit (string from probe_map_units[]). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_string(probe_map_units[DSL_CH_PRIV(ch)->map_unit]);
		break;
	case SR_CONF_PROBE_MAP_MIN:
		/* Probe mapping minimum value (user-defined scale min). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_double(DSL_CH_PRIV(ch)->map_min);
		break;
	case SR_CONF_PROBE_MAP_MAX:
		/* Probe mapping maximum value (user-defined scale max). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		*data = g_variant_new_double(DSL_CH_PRIV(ch)->map_max);
		break;
	case SR_CONF_PATTERN_MODE:
		/* Waveform pattern selection (random/sine/square/sawtooth/triangle).
		 * For real hardware the device supplies the waveform; this is
		 * stored for GUI compat (mirrors demo driver behavior). */
		*data = g_variant_new_string(dso_pattern_strs[devc->pattern_mode]);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* ===========================================================================
 * 8. config_set()
 * =========================================================================== */
static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	int idx, ret = SR_OK;
	gdouble low, high;
	unsigned int i;
	int nv;
	int num_probes;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;
	ch = (cg && cg->channels) ? cg->channels->data : NULL;

	switch (key) {
	case SR_CONF_SAMPLERATE:
	{
		unsigned int n = u64_array_count(devc->profile->dev_caps.samplerates);
		if ((idx = std_u64_idx(data, devc->profile->dev_caps.samplerates, n)) < 0)
			return SR_ERR_ARG;
		devc->cur_samplerate = devc->profile->dev_caps.samplerates[idx];
		if (devc->test_mode == DSL_TEST_NONE && devc->mode != DSL_MODE_LOGIC)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_SAMPLERATE));
		break;
	}
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_CONTINUOUS:
		devc->continuous_mode = g_variant_get_boolean(data);
		break;
	case SR_CONF_EXTERNAL_CLOCK:
		devc->external_clock = g_variant_get_boolean(data);
		break;
	case SR_CONF_CLOCK_TYPE:
		devc->clock_type = g_variant_get_boolean(data);
		break;
	case SR_CONF_CLOCK_EDGE:
		/* Accept string from config_list dropdown. Validate via std_str_idx
		 * against signal_edges[]. devc->clock_edge stores 0/1. */
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(signal_edges))) < 0)
			return SR_ERR_ARG;
		devc->clock_edge = idx;
		break;
	case SR_CONF_OPERATION_MODE:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(opmode_strs))) < 0)
			return SR_ERR_ARG;
		nv = idx;
		if (devc->mode == DSL_MODE_LOGIC && devc->op_mode != nv) {
			if (nv == LO_OP_BUFFER) {
				devc->op_mode = LO_OP_BUFFER;
				devc->test_mode = DSL_TEST_NONE;
				devc->stream = FALSE;
				for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
					if (channel_modes[i].mode == DSL_MODE_LOGIC &&
					    channel_modes[i].stream == devc->stream &&
					    (devc->profile->dev_caps.channels & (1ULL << i))) {
						devc->ch_mode = channel_modes[i].id;
						break;
					}
				}
			} else if (nv == LO_OP_STREAM) {
				devc->op_mode = LO_OP_STREAM;
				devc->test_mode = DSL_TEST_NONE;
				devc->stream = TRUE;
				for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
					if (channel_modes[i].mode == DSL_MODE_LOGIC &&
					    channel_modes[i].stream == devc->stream &&
					    (devc->profile->dev_caps.channels & (1ULL << i))) {
						devc->ch_mode = channel_modes[i].id;
						break;
					}
				}
			} else if (nv == LO_OP_INTEST) {
				devc->op_mode = LO_OP_INTEST;
				devc->test_mode = DSL_TEST_INTERNAL;
				devc->ch_mode = devc->profile->dev_caps.intest_channel;
				devc->stream = !(devc->profile->dev_caps.feature_caps & CAPS_FEATURE_BUF);
			} else {
				ret = SR_ERR;
			}
			dsl_adjust_probes(sdi, channel_modes[devc->ch_mode].num);
			dsl_adjust_samplerate(devc);
			if (devc->op_mode == LO_OP_INTEST) {
				devc->cur_samplerate = devc->stream ?
					channel_modes[devc->ch_mode].max_samplerate / 10 : SR_MHZ(100);
				devc->limit_samples = devc->stream ?
					devc->cur_samplerate * 3 :
					devc->profile->dev_caps.hw_depth / dsl_en_ch_num(sdi);
			}
		}
		break;
	case SR_CONF_BUFFER_OPTIONS:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(bufoption_strs))) < 0)
			return SR_ERR_ARG;
		nv = idx;
		if (devc->mode == DSL_MODE_LOGIC &&
		    (nv == DSL_BUF_STOP || nv == DSL_BUF_UPLOAD))
			devc->buf_options = nv;
		break;
	case SR_CONF_CHANNEL_MODE:
	{
		const char *str = g_variant_get_string(data, NULL);
		if (devc->mode == DSL_MODE_LOGIC) {
			for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
				if ((devc->profile->dev_caps.channels & (1ULL << i)) &&
				    channel_modes[i].stream == devc->stream &&
				    !strcmp(str, channel_modes[i].descr)) {
					devc->ch_mode = channel_modes[i].id;
					dsl_adjust_probes(sdi, channel_modes[devc->ch_mode].num);
					dsl_adjust_samplerate(devc);
					break;
				}
			}
		}
		break;
	}
	case SR_CONF_THRESHOLD:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(threshold_strs))) < 0)
			return SR_ERR_ARG;
		nv = idx;
		if (devc->mode == DSL_MODE_LOGIC && nv != devc->th_level) {
			devc->th_level = nv;
			/* Reload FPGA bitstream for the new voltage threshold. */
			ret = dslogic_fpga_firmware_upload(sdi);
			if (ret != SR_OK)
				sr_err("FPGA reconfiguration for threshold change failed.");
		}
		break;
	case SR_CONF_VTH:
		devc->vth = g_variant_get_double(data);
		ret = dslogic_set_voltage_threshold(sdi, devc->vth);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		if (!strcmp(devc->profile->model, "DSLogic")) {
			if ((idx = std_double_tuple_idx(data, ARRAY_AND_SIZE(thresholds))) < 0)
				return SR_ERR_ARG;
			devc->vth = thresholds[idx][0];
			ret = dslogic_fpga_firmware_upload(sdi);
		} else {
			g_variant_get(data, "(dd)", &low, &high);
			devc->vth = (low + high) / 2.0;
			ret = dslogic_set_voltage_threshold(sdi, devc->vth);
		}
		break;
	case SR_CONF_FILTER:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(filter_strs))) < 0)
			return SR_ERR_ARG;
		devc->filter = idx;
		break;
	case SR_CONF_MAX_HEIGHT:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(max_height_strs))) < 0)
			return SR_ERR_ARG;
		devc->max_height = idx;
		break;
	case SR_CONF_RLE:
		devc->rle_mode = g_variant_get_boolean(data);
		break;
	case SR_CONF_RLE_SUPPORT:
		devc->rle_support = g_variant_get_boolean(data);
		break;
	case SR_CONF_INSTANT:
		devc->instant = g_variant_get_boolean(data);
		if (devc->mode == DSL_MODE_DSO && dsl_en_ch_num(sdi) != 0) {
			if (devc->instant)
				devc->limit_samples = devc->profile->dev_caps.hw_depth /
					channel_modes[devc->ch_mode].unit_bits / dsl_en_ch_num(sdi);
			else
				devc->limit_samples = devc->profile->dev_caps.dso_depth /
					dsl_en_ch_num(sdi);
		}
		break;
	case SR_CONF_DEVICE_MODE:
	{
		int new_mode = g_variant_get_int16(data);
		num_probes = 0;
		devc->mode = new_mode;
		if (new_mode == DSL_MODE_LOGIC) {
			dsl_wr_reg(sdi, CTR0_ADDR, bmSCOPE_CLR);
			for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
				if (channel_modes[i].mode == DSL_MODE_LOGIC &&
				    (devc->profile->dev_caps.channels & (1ULL << i))) {
					devc->ch_mode = channel_modes[i].id;
					num_probes = channel_modes[i].num;
					devc->stream = channel_modes[i].stream;
					dsl_adjust_samplerate(devc);
					break;
				}
			}
		} else if (new_mode == DSL_MODE_DSO) {
			dsl_wr_reg(sdi, CTR0_ADDR, bmSCOPE_SET);
			dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_DSO_SYNC));
			for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
				if (channel_modes[i].mode == DSL_MODE_DSO &&
				    (devc->profile->dev_caps.channels & (1ULL << i))) {
					devc->ch_mode = channel_modes[i].id;
					num_probes = channel_modes[i].num;
					devc->stream = channel_modes[i].stream;
					devc->cur_samplerate =
						channel_modes[i].max_samplerate / num_probes;
					dsl_adjust_samplerate(devc);
					break;
				}
			}
			if (num_probes > 0)
				devc->limit_samples =
					devc->profile->dev_caps.dso_depth / num_probes;
		} else if (new_mode == DSL_MODE_ANALOG) {
			dsl_wr_reg(sdi, CTR0_ADDR, bmSCOPE_SET);
			dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_DSO_SYNC));
			devc->op_mode = LO_OP_STREAM;
			devc->test_mode = DSL_TEST_NONE;
			for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
				if (channel_modes[i].mode == DSL_MODE_ANALOG &&
				    (devc->profile->dev_caps.channels & (1ULL << i))) {
					devc->ch_mode = channel_modes[i].id;
					num_probes = channel_modes[i].num;
					devc->stream = channel_modes[i].stream;
					dsl_adjust_samplerate(devc);
					break;
				}
			}
		} else {
			ret = SR_ERR;
		}
		if (num_probes > 0)
			dsl_setup_probes(sdi, num_probes);
		/* Send full DSO configuration after mode switch to DSO/ANALOG. */
		if (new_mode != DSL_MODE_LOGIC && num_probes > 0)
			dslogic_dso_init(sdi);
		break;
	}
	case SR_CONF_LOOP_MODE:
		devc->is_loop = g_variant_get_boolean(data);
		break;
	case SR_CONF_STREAM:
		devc->stream = g_variant_get_boolean(data);
		break;
	case SR_CONF_TIMEBASE:
		devc->timebase = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_VDIV:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->vdiv = g_variant_get_uint64(data);
		if (devc->mode != DSL_MODE_LOGIC)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, ch, DSO_CMD_PROBE_VDIV));
		break;
	case SR_CONF_PROBE_FACTOR:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->vfactor = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_COUPLING:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		/* Accept int32 to match hwdriver.c SR_T_INT32 and demo driver. */
		DSL_CH_PRIV(ch)->coupling = (uint8_t)g_variant_get_int32(data);
		/* GND coupling is treated as DC by the hardware. */
		if (DSL_CH_PRIV(ch)->coupling == DSL_COUPLING_GND)
			DSL_CH_PRIV(ch)->coupling = DSL_COUPLING_DC;
		if (devc->mode != DSL_MODE_LOGIC)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, ch, DSO_CMD_PROBE_COUPLING));
		break;
	case SR_CONF_PROBE_OFFSET:
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->offset = g_variant_get_uint16(data);
		break;
	case SR_CONF_PROBE_EN:
		if (!ch)
			return SR_ERR_ARG;
		ch->enabled = g_variant_get_boolean(data);
		if (devc->mode == DSL_MODE_DSO) {
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, ch, DSO_CMD_PROBE_EN));
			/* Recalculate samplerate divider when channel count changes. */
			if (dsl_en_ch_num(sdi) != 0)
				ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_SAMPLERATE));
		}
		break;
	case SR_CONF_TRIGGER_SLOPE:
		devc->trigger_slope = g_variant_get_byte(data);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_SLOPE));
		break;
	case SR_CONF_TRIGGER_SOURCE:
		devc->trigger_source = (devc->trigger_source & 0xf0) |
			(g_variant_get_byte(data) & 0x0f);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_SOURCE));
		break;
	case SR_CONF_TRIGGER_CHANNEL:
		devc->trigger_source = (g_variant_get_byte(data) << 4) |
			(devc->trigger_source & 0x0f);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_SOURCE));
		break;
	case SR_CONF_TRIGGER_HOLDOFF:
		devc->trigger_holdoff = g_variant_get_uint64(data);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_HOLDOFF));
		break;
	case SR_CONF_TRIGGER_MARGIN:
		devc->trigger_margin = g_variant_get_byte(data);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_TRIGGER_MARGIN));
		break;
	case SR_CONF_HORIZ_TRIGGERPOS:
		/* Accept double (SR_T_FLOAT) from GUI, store as uint8_t. */
		if (devc->mode == DSL_MODE_DSO) {
			devc->trigger_hrate = (uint8_t)g_variant_get_double(data);
			devc->trigger_hpos = devc->trigger_hrate *
				dsl_en_ch_num(sdi) * devc->limit_samples / 200.0;
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, NULL, DSO_CMD_HORIZ_TRIGGERPOS));
		} else {
			devc->trigger_hpos = (uint8_t)g_variant_get_double(data) *
				devc->limit_samples / 100.0;
		}
		break;
	case SR_CONF_BANDWIDTH_LIMIT:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(bandwidth_strs))) < 0)
			return SR_ERR_ARG;
		nv = idx;
		devc->bw_limit = nv;
		if (nv == 0)
			dsl_wr_reg(sdi, CTR0_ADDR, bmBW20M_CLR);
		else
			dsl_wr_reg(sdi, CTR0_ADDR, bmBW20M_SET);
		break;
	case SR_CONF_CAPTUREFILE:
		/* Stored by the session; no device-side action needed. */
		break;
	case SR_CONF_TRIGGER_VALUE:
		/* DSO trigger level (per-channel). 8-bit value stored in
		 * dsl_channel_priv.trig_value. Sent to hardware via dsl_wr_dso().
		 * Accept int32 to match hwdriver.c SR_T_INT32. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->trig_value = (uint8_t)g_variant_get_int32(data);
		if (devc->mode == DSL_MODE_DSO)
			ret = dsl_wr_dso(sdi, dslogic_dso_cmd_gen(sdi, ch, DSO_CMD_TRIGGER_VALUE));
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		/* Hardware offset (per-channel). Normally auto-updated by
		 * get_measure() on DSO receive; exposing SET allows manual
		 * override during calibration/debugging. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->hw_offset = g_variant_get_uint16(data);
		break;
	case SR_CONF_PROBE_MAP_DEFAULT:
		/* Toggle whether probe uses default (built-in) mapping. */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->map_default = g_variant_get_boolean(data);
		break;
	case SR_CONF_PROBE_MAP_UNIT:
		/* Probe mapping unit (string from probe_map_units[]). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(probe_map_units))) < 0)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->map_unit = idx;
		break;
	case SR_CONF_PROBE_MAP_MIN:
		/* Probe mapping minimum value (user-defined scale min). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->map_min = g_variant_get_double(data);
		break;
	case SR_CONF_PROBE_MAP_MAX:
		/* Probe mapping maximum value (user-defined scale max). */
		if (!ch || !ch->priv)
			return SR_ERR_ARG;
		DSL_CH_PRIV(ch)->map_max = g_variant_get_double(data);
		break;
	case SR_CONF_PATTERN_MODE:
		/* Waveform pattern selection (random/sine/square/sawtooth/triangle).
		 * For real DSLogic hardware the device supplies the waveform —
		 * stored for GUI compat. Validated against dso_pattern_strs[]. */
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(dso_pattern_strs))) < 0)
			return SR_ERR_ARG;
		devc->pattern_mode = (uint8_t)idx;
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

/* ===========================================================================
 * 9. config_list()
 * =========================================================================== */
static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	unsigned int i;

	devc = (sdi) ? sdi->priv : NULL;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	case SR_CONF_SAMPLERATE:
	{
		GVariantBuilder gvb;
		GVariant *gvar;
		unsigned int n;
		if (!devc || !devc->profile)
			return SR_ERR_ARG;
		n = u64_array_count(devc->profile->dev_caps.samplerates);
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"),
			devc->profile->dev_caps.samplerates,
			n * sizeof(uint64_t), TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_VOLTAGE_THRESHOLD:
		if (!devc || !devc->profile)
			return SR_ERR_ARG;
		if (!strcmp(devc->profile->model, "DSLogic"))
			*data = std_gvar_thresholds(ARRAY_AND_SIZE(thresholds));
		else
			*data = std_gvar_min_max_step_thresholds(0.0, 5.0, 0.1);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(opmode_strs));
		break;
	case SR_CONF_BUFFER_OPTIONS:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(bufoption_strs));
		break;
	case SR_CONF_CHANNEL_MODE:
	{
		GVariantBuilder gvb;
		if (!devc || !devc->profile)
			return SR_ERR_ARG;
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("as"));
		for (i = 0; i < ARRAY_SIZE(channel_modes); i++) {
			if ((devc->profile->dev_caps.channels & (1ULL << i)) &&
			    channel_modes[i].stream == devc->stream)
				g_variant_builder_add(&gvb, "s", channel_modes[i].descr);
		}
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_THRESHOLD:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(threshold_strs));
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(filter_strs));
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(max_height_strs));
		break;
	case SR_CONF_CLOCK_EDGE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(signal_edges));
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;
	case SR_CONF_PROBE_COUPLING:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(coupling_strs));
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(device_mode_strs));
		break;
	case SR_CONF_BANDWIDTH_LIMIT:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(bandwidth_strs));
		break;
	case SR_CONF_PROBE_VDIV:
	{
		GVariantBuilder gvb;
		GVariant *gvar;
		unsigned int n;
		if (!devc || !devc->profile)
			return SR_ERR_ARG;
		n = u64_array_count(devc->profile->dev_caps.vdivs);
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"),
			devc->profile->dev_caps.vdivs,
			n * sizeof(uint64_t), TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "vdivs", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_TIMEBASE:
		/*
		 * Build a 1-2-5 timebase ladder from the min timebase
		 * (based on hw_max_samplerate) to the max timebase
		 * (based on dso_depth, num channels, min_samplerate, 10 hdivs).
		 */
	{
		GVariantBuilder gvb;
		uint64_t max_tb, min_tb, tb;
		if (!devc || !devc->profile)
			return SR_ERR_ARG;
		min_tb = UINT64_C(1000000000) / channel_modes[devc->ch_mode].hw_max_samplerate;
		max_tb = UINT64_C(1000000000) * devc->profile->dev_caps.dso_depth /
			channel_modes[devc->ch_mode].num /
			channel_modes[devc->ch_mode].min_samplerate / 10;
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("at"));
		tb = min_tb;
		while (tb <= max_tb) {
			g_variant_builder_add(&gvb, "t", tb);
			/* Step through 1-2-5 sequence. */
			if (tb < 10)
				tb = 10;
			else {
				uint64_t m = tb;
				while (m >= 10)
					m /= 10;
				uint64_t p = tb / m;
				if (m == 1)
					tb = 2 * p;
				else if (m == 2)
					tb = 5 * p;
				else
					tb = 10 * p;
			}
		}
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_PROBE_MAP_UNIT:
		/* Probe mapping unit list — mirrors demo driver's dso_map_units[]. */
		*data = g_variant_new_strv(ARRAY_AND_SIZE(probe_map_units));
		break;
	case SR_CONF_PROBE_CONFIGS:
		/* Returns the list of probe-config keys supported by this driver.
		 * ProbeOptions binding iterates this list and creates widgets for
		 * VDIV/COUPLING/MAP_* keys. Applies to both DSO and ANALOG. */
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(probe_configs));
		break;
	case SR_CONF_PATTERN_MODE:
		/* Waveform pattern list (random/sine/square/sawtooth/triangle).
		 * Mirrors demo driver's dso_pattern_strs[] for GUI compat. */
		*data = g_variant_new_strv(ARRAY_AND_SIZE(dso_pattern_strs));
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* ===========================================================================
 * 11. Driver registration
 * =========================================================================== */
static struct sr_dev_driver dreamsourcelab_dslogic_driver_info = {
	.name = "dreamsourcelab-dslogic",
	.longname = "DreamSourceLab DSLogic/DSCope",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dslogic_acquisition_start,
	.dev_acquisition_stop = dslogic_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(dreamsourcelab_dslogic_driver_info);
