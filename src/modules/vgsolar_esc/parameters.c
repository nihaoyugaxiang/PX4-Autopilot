/**
 * Enable read-only 2BLD6010 telemetry
 * @min 0
 * @max 1
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_EN, 0);
/**
 * Number of controllers
 * @min 1
 * @max 4
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_NUM, 1);
/**
 * Requested polling frequency per controller in Hz
 * @min 1
 * @max 20
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_RATE, 10);
/**
 * Telemetry health timeout in ms
 * @min 100
 * @max 5000
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_TO, 500);
/**
 * First Modbus address
 * @min 1
 * @max 247
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_ADDR1, 2);
/**
 * Second Modbus address
 * @min 1
 * @max 247
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_ADDR2, 1);
/**
 * Third Modbus address
 * @min 1
 * @max 247
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_ADDR3, 3);
/**
 * Fourth Modbus address
 * @min 1
 * @max 247
 * @reboot_required true
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_ADDR4, 4);

/**
 * Standard ESC index offset.
 * @min 0
 * @max 4
 * @group VGSolar ESC
 */
PARAM_DEFINE_INT32(VGS_ESC_OFS, 0);
