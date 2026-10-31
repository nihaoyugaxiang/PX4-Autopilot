# VGSolar吸盘输入诊断；仅传感数据，不驱动执行器。
uint64 timestamp                 # publication time, microseconds since boot
uint64 timestamp_sample          # latest pressure ADC report time; 0 when absent
int32 adc_raw                    # raw PC0 ADC count; -1 if channel absent
uint32 adc_resolution
float32 adc_v_ref                # V, provided by board ADC driver
float32 pressure_voltage_v       # last measured V; diagnostic only, may be stale
float32 pressure_raw_kpa         # instantaneous conversion; NaN if invalid
float32 pressure_kpa             # filtered gauge pressure; NaN unless pressure_valid
uint8 pressure_status
uint8 PRESSURE_WAITING = 0
uint8 PRESSURE_WARMING_UP = 1
uint8 PRESSURE_HEALTHY = 2
uint8 PRESSURE_INVALID_VOLTAGE = 3
uint8 PRESSURE_STALE = 4
uint8 PRESSURE_INVALID_CONFIG = 5
uint8 PRESSURE_MISSING_CHANNEL = 6
uint8 PRESSURE_ADC_ERROR = 7
uint8 PRESSURE_DISABLED = 8
bool pressure_valid
bool ir_raw                      # PB5 raw GPIO level
bool ir_debounced                # debounced GPIO level
bool ir_blade_present            # polarity applied; check ir_sample_valid first
bool ir_sample_valid             # fresh debounced sampling, NOT proof of sensor connected or lift position
bool enabled
uint8 pressure_adc_channel       # ADC1 channel 10 = PC0 on this board
