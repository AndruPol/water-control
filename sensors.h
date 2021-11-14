/*
 * sensors.h
 */
#ifndef SENSORS_H_
#define SENSORS_H_

#define MCUTEMP				1		// MCU internal temperature
#define SENSORSALL			2

#ifdef __cplusplus
extern "C" {
#endif

#if MCUTEMP
#define TEMPINT				0		// номер канала внутр.температуры
#define VREFINT				1		// номер канала VREFINT

#define V25					(1.43)		// V
#define AVG_SLOPE			(0.0043)	// V/C
#define VREF_INT			(1200)		// V * 1000
#define TEMPCALC(d,v)		((V25 - d * VREF_INT / v / 1000.0) / AVG_SLOPE + 25.0)
#endif

typedef enum {
	ADC_NO_ERROR,
	ADC_CONV_ERROR,
	ADC_TIMEOUT,
} adc_error_t;

void sensors_init(void);
adc_error_t sensors_read(void);

extern float mcu_temp;

#ifdef __cplusplus
}
#endif
#endif /* SENSORS_H_ */
